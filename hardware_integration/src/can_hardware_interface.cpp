//================================================================================================
/// @file can_hardware_interface.cpp
///
/// @brief An interface for using socket CAN on linux. Mostly for testing, but it could be
/// used in any application to get the stack hooked up to the bus.
/// @author Adrian Del Grosso
///
/// @copyright 2022 Adrian Del Grosso
//================================================================================================
#include "isobus/hardware_integration/can_hardware_interface.hpp"
#include "isobus/isobus/can_stack_logger.hpp"
#include "isobus/utility/system_timing.hpp"
#include "isobus/utility/to_string.hpp"

#include <algorithm>

namespace isobus
{
	std::unique_ptr<std::thread> CANHardwareInterface::updateThread;
	std::unique_ptr<std::thread> CANHardwareInterface::wakeupThread;
	std::condition_variable CANHardwareInterface::updateThreadWakeupCondition;
	std::atomic_bool CANHardwareInterface::stackNeedsUpdate = { false };
	std::uint32_t CANHardwareInterface::periodicUpdateInterval = PERIODIC_UPDATE_INTERVAL;

	isobus::EventDispatcher<const isobus::CANMessageFrame &> CANHardwareInterface::frameReceivedEventDispatcher;
	isobus::EventDispatcher<const isobus::CANMessageFrame &> CANHardwareInterface::frameTransmittedEventDispatcher;
	isobus::EventDispatcher<> CANHardwareInterface::periodicUpdateEventDispatcher;

	std::unordered_map<std::shared_ptr<const CANNetworkManager>, std::unique_ptr<CANHardwareInterface::CANHardware>> CANHardwareInterface::hardwareChannels;
	std::mutex CANHardwareInterface::hardwareChannelsMutex;
	std::mutex CANHardwareInterface::updateMutex;
	std::atomic_bool CANHardwareInterface::threadsStarted = { false };

	CANHardwareInterface CANHardwareInterface::SINGLETON;

	CANHardwareInterface::~CANHardwareInterface()
	{
		stop_threads();
	}

	bool CANHardwareInterface::assign_can_channel_frame_handler(std::shared_ptr<const CANNetworkManager> network, std::shared_ptr<CANHardwarePlugin> canDriver)
	{
		std::lock_guard<std::mutex> lock(hardwareChannelsMutex);

		if (nullptr == network)
		{
			isobus::CANStackLogger::error("[HardwareInterface] Unable to assign frame handler for network, because the network is null.");
			return false;
		}

		if (0 == hardwareChannels.count(network))
		{
			hardwareChannels[network] = std::make_unique<CANHardware>();
		}
		hardwareChannels[network]->frameHandler = canDriver;

		if (threadsStarted)
		{
			hardwareChannels[network]->frameHandler->open();

			if (hardwareChannels[network]->frameHandler->get_is_valid())
			{
				hardwareChannels[network]->receiveMessageThread = std::make_unique<std::thread>(receive_can_frame_thread_function, network);
			}
		}

		return true;
	}

	std::size_t CANHardwareInterface::get_number_of_can_channels()
	{
		return hardwareChannels.size();
	}

	bool CANHardwareInterface::unassign_can_channel_frame_handler(std::shared_ptr<const CANNetworkManager> network)
	{
		std::lock_guard<std::mutex> lock(hardwareChannelsMutex);

		if (0 == hardwareChannels.count(network))
		{
			//! @todo find a way to add a identifying string to the network manager
			isobus::CANStackLogger::error("[HardwareInterface] Unable to remove frame handler for network, because the network is not assigned.");
			return false;
		}

		if (nullptr == hardwareChannels[network]->frameHandler)
		{
			//! @todo find a way to add a identifying string to the network manager
			isobus::CANStackLogger::error("[HardwareInterface] Unable to remove frame handler for network, because there is no frame handler assigned.");
			return false;
		}

		if (threadsStarted)
		{
			if (nullptr != hardwareChannels[network]->frameHandler)
			{
				hardwareChannels[network]->frameHandler->close();
			}
			if (nullptr != hardwareChannels[network]->receiveMessageThread)
			{
				if (hardwareChannels[network]->receiveMessageThread->joinable())
				{
					hardwareChannels[network]->receiveMessageThread->join();
				}
			}
		}

		hardwareChannels.erase(network);
		return true;
	}

	bool CANHardwareInterface::start()
	{
		std::lock_guard<std::mutex> lock(hardwareChannelsMutex);

		if (threadsStarted)
		{
			isobus::CANStackLogger::error("[HardwareInterface] Cannot start interface more than once.");
			return false;
		}

		updateThread = std::make_unique<std::thread>(update_thread_function);
		wakeupThread = std::make_unique<std::thread>(periodic_update_function);

		threadsStarted = true;

		std::for_each(hardwareChannels.begin(), hardwareChannels.end(), [](const std::unordered_map<std::shared_ptr<const CANNetworkManager>, std::unique_ptr<CANHardware>>::value_type &hardware) {
			if (nullptr != hardware.second->frameHandler)
			{
				hardware.second->frameHandler->open();

				if (hardware.second->frameHandler->get_is_valid())
				{
					hardware.second->receiveMessageThread = std::make_unique<std::thread>(receive_can_frame_thread_function, hardware.first);
				}
			}
		});

		return true;
	}

	bool CANHardwareInterface::stop()
	{
		if (!threadsStarted)
		{
			isobus::CANStackLogger::error("[HardwareInterface] Cannot stop interface before it is started.");
			return false;
		}
		stop_threads();

		std::lock_guard<std::mutex> channelsLock(hardwareChannelsMutex);
		std::for_each(hardwareChannels.begin(), hardwareChannels.end(), [](const std::unordered_map<std::shared_ptr<const CANNetworkManager>, std::unique_ptr<CANHardware>>::value_type &hardware) {
			if (nullptr != hardware.second->frameHandler)
			{
				hardware.second->frameHandler = nullptr;
			}
			std::unique_lock<std::mutex> transmittingLock(hardware.second->messagesToBeTransmittedMutex);
			hardware.second->messagesToBeTransmitted.clear();
			transmittingLock.unlock();

			std::unique_lock<std::mutex> receivingLock(hardware.second->receivedMessagesMutex);
			hardware.second->receivedMessages.clear();
			receivingLock.unlock();
		});
		return true;
	}
	bool CANHardwareInterface::is_running()
	{
		return threadsStarted;
	}

	bool send_can_message_frame_to_hardware(const std::weak_ptr<const CANNetworkManager> associatedNetwork, const CANMessageFrame &frame)
	{
		if (auto network = associatedNetwork.lock())
		{
			return CANHardwareInterface::transmit_can_frame(network, frame);
		}
		return false;
	}

	bool CANHardwareInterface::transmit_can_frame(std::shared_ptr<const CANNetworkManager> network, const CANMessageFrame &frame)
	{
		if (!threadsStarted)
		{
			isobus::CANStackLogger::error("[HardwareInterface] Cannot transmit message before interface is started.");
			return false;
		}

		if (0 == hardwareChannels.count(network))
		{
			//! @todo find a way to add a identifying string to the network manager
			isobus::CANStackLogger::warn("[HardwareInterface] Unable to transmit message on network, because the network is not assigned.");
			return false;
		}

		const std::unique_ptr<CANHardware> &channel = hardwareChannels[network];
		if ((nullptr != channel->frameHandler) && channel->frameHandler->get_is_valid())
		{
			std::lock_guard<std::mutex> lock(channel->messagesToBeTransmittedMutex);
			channel->messagesToBeTransmitted.push_back(frame);

			updateThreadWakeupCondition.notify_all();
			return true;
		}
		return false;
	}

	isobus::EventDispatcher<const isobus::CANMessageFrame &> &CANHardwareInterface::get_can_frame_received_event_dispatcher()
	{
		return frameReceivedEventDispatcher;
	}

	isobus::EventDispatcher<const isobus::CANMessageFrame &> &CANHardwareInterface::get_can_frame_transmitted_event_dispatcher()
	{
		return frameTransmittedEventDispatcher;
	}

	isobus::EventDispatcher<> &CANHardwareInterface::get_periodic_update_event_dispatcher()
	{
		return periodicUpdateEventDispatcher;
	}

	void CANHardwareInterface::set_periodic_update_interval(std::uint32_t value)
	{
		periodicUpdateInterval = value;
	}

	std::uint32_t CANHardwareInterface::get_periodic_update_interval()
	{
		return periodicUpdateInterval;
	}

	void CANHardwareInterface::update_thread_function()
	{
		std::unique_lock<std::mutex> channelsLock(hardwareChannelsMutex);
		// Wait until everything is running
		channelsLock.unlock();

		while (threadsStarted)
		{
			std::unique_lock<std::mutex> threadLock(updateMutex);
			updateThreadWakeupCondition.wait_for(threadLock, std::chrono::seconds(1)); // Timeout after 1 second

			if (threadsStarted)
			{
				// Stage 1 - Receiving messages from hardware
				channelsLock.lock();
				std::for_each(hardwareChannels.begin(), hardwareChannels.end(), [](const std::unordered_map<std::shared_ptr<const CANNetworkManager>, std::unique_ptr<CANHardware>>::value_type &hardware) {
					std::lock_guard<std::mutex> lock(hardware.second->receivedMessagesMutex);
					while (!hardware.second->receivedMessages.empty())
					{
						const auto &frame = hardware.second->receivedMessages.front();

						frameReceivedEventDispatcher.invoke(frame);
						isobus::receive_can_message_frame_from_hardware(hardware.first, frame);

						hardware.second->receivedMessages.pop_front();
					}
				});
				channelsLock.unlock();

				// Stage 2 - Sending messages
				if (stackNeedsUpdate)
				{
					stackNeedsUpdate = false;
					periodicUpdateEventDispatcher.invoke();
					std::for_each(hardwareChannels.begin(), hardwareChannels.end(), [](const std::unordered_map<std::shared_ptr<const CANNetworkManager>, std::unique_ptr<CANHardware>>::value_type &hardware) {
						isobus::periodic_update_from_hardware(hardware.first);
					});
				}

				// Stage 3 - Transmitting messages to hardware
				channelsLock.lock();
				std::for_each(hardwareChannels.begin(), hardwareChannels.end(), [](const std::unordered_map<std::shared_ptr<const CANNetworkManager>, std::unique_ptr<CANHardware>>::value_type &hardware) {
					std::lock_guard<std::mutex> lock(hardware.second->messagesToBeTransmittedMutex);
					while (!hardware.second->messagesToBeTransmitted.empty())
					{
						const auto &frame = hardware.second->messagesToBeTransmitted.front();

						if ((hardware.second->frameHandler != nullptr) && hardware.second->frameHandler->write_frame(frame))
						{
							frameTransmittedEventDispatcher.invoke(frame);
							isobus::on_transmit_can_message_frame_from_hardware(hardware.first, frame);
							hardware.second->messagesToBeTransmitted.pop_front();
						}
						else
						{
							break;
						}
					}
				});
				channelsLock.unlock();
			}
		}
	}

	void CANHardwareInterface::receive_can_frame_thread_function(std::weak_ptr<const CANNetworkManager> associatedNetwork)
	{
		std::unique_lock<std::mutex> channelsLock(hardwareChannelsMutex);
		// Wait until everything is running
		channelsLock.unlock();

		isobus::CANMessageFrame frame;
		while (threadsStarted &&
		       associatedNetwork.lock() &&
		       (0 != hardwareChannels.count(associatedNetwork.lock())))
		{
			auto network = associatedNetwork.lock();
			if ((nullptr != hardwareChannels[network]) && hardwareChannels[network]->frameHandler->get_is_valid())
			{
				// Socket or other hardware still open
				if (hardwareChannels[network]->frameHandler->read_frame(frame))
				{
					std::unique_lock<std::mutex> receiveLock(hardwareChannels[network]->receivedMessagesMutex);
					hardwareChannels[network]->receivedMessages.push_back(frame);
					receiveLock.unlock();
					updateThreadWakeupCondition.notify_all();
				}
			}
			else
			{
				//! @todo find a way to add a identifying string to the network manager
				CANStackLogger::error("[CAN Rx Thread] Frame handler is unassigned, or invalid.");
				std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // Arbitrary, but don't want to infinite loop on the validity check.
			}
		}
	}

	void CANHardwareInterface::periodic_update_function()
	{
		std::unique_lock<std::mutex> channelsLock(hardwareChannelsMutex);
		// Wait until everything is running
		channelsLock.unlock();

		while (threadsStarted)
		{
			stackNeedsUpdate = true;
			updateThreadWakeupCondition.notify_all();
			std::this_thread::sleep_for(std::chrono::milliseconds(periodicUpdateInterval));
		}
	}

	void CANHardwareInterface::stop_threads()
	{
		threadsStarted = false;
		if (nullptr != updateThread)
		{
			if (updateThread->joinable())
			{
				updateThreadWakeupCondition.notify_all();
				updateThread->join();
			}
			updateThread = nullptr;
		}

		if (nullptr != wakeupThread)
		{
			if (wakeupThread->joinable())
			{
				wakeupThread->join();
			}
			wakeupThread = nullptr;
		}

		std::for_each(hardwareChannels.begin(), hardwareChannels.end(), [](const std::unordered_map<std::shared_ptr<const CANNetworkManager>, std::unique_ptr<CANHardware>>::value_type &hardware) {
			if (nullptr != hardware.second->frameHandler)
			{
				hardware.second->frameHandler->close();
			}
			if (nullptr != hardware.second->receiveMessageThread)
			{
				if (hardware.second->receiveMessageThread->joinable())
				{
					hardware.second->receiveMessageThread->join();
				}
				hardware.second->receiveMessageThread = nullptr;
			}
		});
	}
}