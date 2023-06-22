//================================================================================================
/// @file can_network_manager.cpp
///
/// @brief The main class that manages the ISOBUS stack including: callbacks, Name to Address
/// management, making control functions, and driving the various protocols.
/// @author Adrian Del Grosso
/// @author Daan Steenbergen
///
/// @copyright 2022 Adrian Del Grosso
//================================================================================================

#include "isobus/isobus/can_network_manager.hpp"
#include "isobus/isobus/can_constants.hpp"
#include "isobus/isobus/can_general_parameter_group_numbers.hpp"
#include "isobus/isobus/can_hardware_abstraction.hpp"
#include "isobus/isobus/can_message.hpp"
#include "isobus/isobus/can_partnered_control_function.hpp"
#include "isobus/isobus/can_protocol.hpp"
#include "isobus/isobus/can_stack_logger.hpp"
#include "isobus/utility/system_timing.hpp"
#include "isobus/utility/to_string.hpp"

#include <algorithm>
#include <cstring>
#include <numeric>

namespace isobus
{
	void CANNetworkManager::initialize()
	{
		if (!initialized)
		{
			receiveMessageList.clear();
			transportProtocol.initialize(shared_from_this(), {});
			fastPacketProtocol.initialize(shared_from_this(), {});
			extendedTransportProtocol.initialize(shared_from_this(), {});
			initialized = true;
		}
		else
		{
			CANStackLogger::warn("[Network] Network already initialized");
		}
	}

	std::shared_ptr<ControlFunction> CANNetworkManager::get_control_function(std::uint8_t address, CANLibBadge<AddressClaimStateMachine>) const
	{
		return get_control_function(address);
	}

	void CANNetworkManager::add_global_parameter_group_number_callback(std::uint32_t parameterGroupNumber, CANLibCallback callback, void *parent)
	{
		globalParameterGroupNumberCallbacks.emplace_back(parameterGroupNumber, callback, parent, nullptr);
	}

	void CANNetworkManager::remove_global_parameter_group_number_callback(std::uint32_t parameterGroupNumber, CANLibCallback callback, void *parent)
	{
		ParameterGroupNumberCallbackData tempObject(parameterGroupNumber, callback, parent, nullptr);
		auto callbackLocation = std::find(globalParameterGroupNumberCallbacks.begin(), globalParameterGroupNumberCallbacks.end(), tempObject);
		if (globalParameterGroupNumberCallbacks.end() != callbackLocation)
		{
			globalParameterGroupNumberCallbacks.erase(callbackLocation);
		}
	}

	void CANNetworkManager::add_any_control_function_parameter_group_number_callback(std::uint32_t parameterGroupNumber, CANLibCallback callback, void *parent)
	{
		std::lock_guard<std::mutex> lock(anyControlFunctionCallbacksMutex);
		anyControlFunctionParameterGroupNumberCallbacks.emplace_back(parameterGroupNumber, callback, parent, nullptr);
	}

	void CANNetworkManager::remove_any_control_function_parameter_group_number_callback(std::uint32_t parameterGroupNumber, CANLibCallback callback, void *parent)
	{
		ParameterGroupNumberCallbackData tempObject(parameterGroupNumber, callback, parent, nullptr);
		std::lock_guard<std::mutex> lock(anyControlFunctionCallbacksMutex);
		auto callbackLocation = std::find(anyControlFunctionParameterGroupNumberCallbacks.begin(), anyControlFunctionParameterGroupNumberCallbacks.end(), tempObject);
		if (anyControlFunctionParameterGroupNumberCallbacks.end() != callbackLocation)
		{
			anyControlFunctionParameterGroupNumberCallbacks.erase(callbackLocation);
		}
	}

	float CANNetworkManager::get_estimated_busload()
	{
		const std::lock_guard<std::mutex> lock(busloadUpdateMutex);
		constexpr float ISOBUS_BAUD_RATE_BPS = 250000.0f;

		float totalTimeInAccumulatorWindow = (busloadMessageBitsHistory.size() * BUSLOAD_UPDATE_FREQUENCY_MS) / 1000.0f;
		std::uint32_t totalBitCount = std::accumulate(busloadMessageBitsHistory.begin(), busloadMessageBitsHistory.end(), 0);
		float retVal = (0 != totalTimeInAccumulatorWindow) ? ((totalBitCount / (totalTimeInAccumulatorWindow * ISOBUS_BAUD_RATE_BPS)) * 100.0f) : 0.0f;
		return retVal;
	}

	bool CANNetworkManager::send_can_message(std::uint32_t parameterGroupNumber,
	                                         const std::uint8_t *dataBuffer,
	                                         std::uint32_t dataLength,
	                                         std::shared_ptr<InternalControlFunction> sourceControlFunction,
	                                         std::shared_ptr<ControlFunction> destinationControlFunction,
	                                         CANIdentifier::CANPriority priority,
	                                         TransmitCompleteCallback transmitCompleteCallback,
	                                         void *parentPointer,
	                                         DataChunkCallback frameChunkCallback)
	{
		bool retVal = false;

		if (((nullptr != dataBuffer) ||
		     (nullptr != frameChunkCallback)) &&
		    (dataLength > 0) &&
		    (dataLength <= CANMessage::ABSOLUTE_MAX_MESSAGE_LENGTH) &&
		    (nullptr != sourceControlFunction) &&
		    ((parameterGroupNumber == static_cast<std::uint32_t>(CANLibParameterGroupNumber::AddressClaim)) ||
		     (sourceControlFunction->get_address_valid())))
		{
			CANLibProtocol *currentProtocol;

			// See if any transport layer protocol can handle this message
			for (std::uint32_t i = 0; i < CANLibProtocol::get_number_protocols(); i++)
			{
				if (CANLibProtocol::get_protocol(i, currentProtocol))
				{
					retVal = currentProtocol->protocol_transmit_message(parameterGroupNumber,
					                                                    dataBuffer,
					                                                    dataLength,
					                                                    sourceControlFunction,
					                                                    destinationControlFunction,
					                                                    transmitCompleteCallback,
					                                                    parentPointer,
					                                                    frameChunkCallback);

					if (retVal)
					{
						break;
					}
				}
			}

			//! @todo Allow sending 8 byte message with the frameChunkCallback
			if ((!retVal) &&
			    (nullptr != dataBuffer))
			{
				if (auto network = sourceControlFunction->get_associated_network().lock())
				{
					if (nullptr == destinationControlFunction)
					{
						//! @todo move binding of dest address to hardware layer
						retVal = network->send_can_message_raw(sourceControlFunction->get_address(), 0xFF, parameterGroupNumber, priority, dataBuffer, dataLength);
					}
					else if (destinationControlFunction->get_address_valid())
					{
						retVal = network->send_can_message_raw(sourceControlFunction->get_address(), destinationControlFunction->get_address(), parameterGroupNumber, priority, dataBuffer, dataLength);
					}

					if ((retVal) &&
					    (nullptr != transmitCompleteCallback))
					{
						// Message was not sent via a protocol, so handle the tx callback now
						transmitCompleteCallback(parameterGroupNumber, dataLength, sourceControlFunction, destinationControlFunction, retVal, parentPointer);
					}
				}
			}
		}
		return retVal;
	}

	void CANNetworkManager::receive_can_message(const CANMessage &message)
	{
		if (initialized)
		{
			std::lock_guard<std::mutex> lock(receiveMessageMutex);

			receiveMessageList.push_back(message);
		}
	}

	void CANNetworkManager::update()
	{
		const std::lock_guard<std::mutex> lock(ControlFunction::controlFunctionProcessingMutex);

		if (!initialized)
		{
			initialize();
		}

		update_new_partners();

		process_rx_messages();

		InternalControlFunction::update_address_claiming({});

		if (InternalControlFunction::get_any_internal_control_function_changed_address({}))
		{
			for (std::size_t i = 0; i < InternalControlFunction::get_number_internal_control_functions(); i++)
			{
				std::shared_ptr<InternalControlFunction> currentInternalControlFunction = InternalControlFunction::get_internal_control_function(i);

				if (nullptr != currentInternalControlFunction)
				{
					if (inactiveControlFunctions.end() == std::find_if(inactiveControlFunctions.begin(), inactiveControlFunctions.end(), [currentInternalControlFunction](const std::shared_ptr<ControlFunction> &cf_ptr) {
						    return (cf_ptr == currentInternalControlFunction);
					    }))
					{
						inactiveControlFunctions.push_back(std::shared_ptr<InternalControlFunction>(currentInternalControlFunction));
					}
					if (currentInternalControlFunction->get_changed_address_since_last_update({}))
					{
						update_address_table(currentInternalControlFunction->get_address());
					}
				}
			}
		}

		for (std::size_t i = 0; i < CANLibProtocol::get_number_protocols(); i++)
		{
			CANLibProtocol *currentProtocol = nullptr;

			if (CANLibProtocol::get_protocol(i, currentProtocol))
			{
				if (!currentProtocol->get_is_initialized())
				{
					currentProtocol->initialize(shared_from_this(), {});
				}
				currentProtocol->update({});
			}
		}
		update_busload_history();
		updateTimestamp_ms = SystemTiming::get_timestamp_ms();
	}

	bool CANNetworkManager::send_can_message_raw(std::uint8_t sourceAddress,
	                                             std::uint8_t destAddress,
	                                             std::uint32_t parameterGroupNumber,
	                                             std::uint8_t priority,
	                                             const void *data,
	                                             std::uint32_t size,
	                                             CANLibBadge<AddressClaimStateMachine>) const
	{
		return send_can_message_raw(sourceAddress, destAddress, parameterGroupNumber, priority, data, size);
	}

	ParameterGroupNumberCallbackData CANNetworkManager::get_global_parameter_group_number_callback(std::uint32_t index) const
	{
		ParameterGroupNumberCallbackData retVal(0, nullptr, nullptr, nullptr);

		if (index < globalParameterGroupNumberCallbacks.size())
		{
			retVal = globalParameterGroupNumberCallbacks[index];
		}
		return retVal;
	}

	void receive_can_message_frame_from_hardware(const std::weak_ptr<CANNetworkManager> associatedNetwork, const CANMessageFrame &rxFrame)
	{
		if (auto network = associatedNetwork.lock())
		{
			std::const_pointer_cast<CANNetworkManager>(network)->process_receive_can_message_frame(rxFrame);
		}
	}

	void on_transmit_can_message_frame_from_hardware(const std::weak_ptr<CANNetworkManager> associatedNetwork, const CANMessageFrame &txFrame)
	{
		if (auto network = associatedNetwork.lock())
		{
			std::const_pointer_cast<CANNetworkManager>(network)->process_transmitted_can_message_frame(txFrame);
		}
	}

	void periodic_update_from_hardware(std::shared_ptr<const CANNetworkManager> network)
	{
		std::const_pointer_cast<CANNetworkManager>(network)->update();
	}

	void CANNetworkManager::process_receive_can_message_frame(const CANMessageFrame &rxFrame)
	{
		CANMessage tempCANMessage;
		update_control_functions(rxFrame);

		tempCANMessage.set_identifier(CANIdentifier(rxFrame.identifier));

		tempCANMessage.set_source_control_function(get_control_function(tempCANMessage.get_identifier().get_source_address()));
		tempCANMessage.set_destination_control_function(get_control_function(tempCANMessage.get_identifier().get_destination_address()));
		tempCANMessage.set_data(rxFrame.data, rxFrame.dataLength);

		update_busload(rxFrame.get_number_bits_in_message());

		receive_can_message(tempCANMessage);
	}

	void CANNetworkManager::process_transmitted_can_message_frame(const CANMessageFrame &txFrame)
	{
		update_busload(txFrame.get_number_bits_in_message());
	}

	void CANNetworkManager::on_control_function_destroyed(std::shared_ptr<ControlFunction> controlFunction, CANLibBadge<ControlFunction>)
	{
		auto result = std::find(inactiveControlFunctions.begin(), inactiveControlFunctions.end(), controlFunction);
		if (result != inactiveControlFunctions.end())
		{
			inactiveControlFunctions.erase(result);
		}

		for (std::uint8_t i = 0; i < NULL_CAN_ADDRESS; i++)
		{
			if (controlFunctionTable[i] == controlFunction)
			{
				if (i != controlFunction->get_address())
				{
					CANStackLogger::warn("[NM]: %s control function with address '%d' was at address '%d' in the lookup table prior to deletion.", controlFunction->get_type_string().c_str(), controlFunction->get_address(), i);
				}

				if (initialized)
				{
					// The control function was active, replace it with an new external control function
					controlFunctionTable[controlFunction->address] = ControlFunction::create(controlFunction->get_NAME(), controlFunction->get_address(), shared_from_this());
				}
				else
				{
					// The network manager is not initialized yet, just remove the control function from the table
					controlFunctionTable[i] = nullptr;
				}
			}
		}
		CANStackLogger::debug("[NM]: %s control function with address '%d' is deleted.", controlFunction->get_type_string().c_str(), controlFunction->get_address());
	}

	void CANNetworkManager::on_control_function_created(std::shared_ptr<ControlFunction>, CANLibBadge<ControlFunction>)
	{
		//! @todo implement this when we stop using the dedicated internal/partner control functions lists in their respective classes
	}

	FastPacketProtocol &CANNetworkManager::get_fast_packet_protocol()
	{
		return fastPacketProtocol;
	}

	bool CANNetworkManager::add_protocol_parameter_group_number_callback(std::uint32_t parameterGroupNumber, CANLibCallback callback, void *parentPointer)
	{
		bool retVal = false;
		ParameterGroupNumberCallbackData callbackInfo(parameterGroupNumber, callback, parentPointer, nullptr);

		const std::lock_guard<std::mutex> lock(protocolPGNCallbacksMutex);

		if ((nullptr != callback) && (protocolPGNCallbacks.end() == find(protocolPGNCallbacks.begin(), protocolPGNCallbacks.end(), callbackInfo)))
		{
			protocolPGNCallbacks.push_back(callbackInfo);
			retVal = true;
		}
		return retVal;
	}

	bool CANNetworkManager::remove_protocol_parameter_group_number_callback(std::uint32_t parameterGroupNumber, CANLibCallback callback, void *parentPointer)
	{
		bool retVal = false;
		ParameterGroupNumberCallbackData callbackInfo(parameterGroupNumber, callback, parentPointer, nullptr);

		const std::lock_guard<std::mutex> lock(protocolPGNCallbacksMutex);

		if (nullptr != callback)
		{
			std::list<ParameterGroupNumberCallbackData>::iterator callbackLocation;
			callbackLocation = find(protocolPGNCallbacks.begin(), protocolPGNCallbacks.end(), callbackInfo);

			if (protocolPGNCallbacks.end() != callbackLocation)
			{
				protocolPGNCallbacks.erase(callbackLocation);
				retVal = true;
			}
		}
		return retVal;
	}

	void CANNetworkManager::update_address_table(const CANMessage &message)
	{
		if (static_cast<std::uint32_t>(CANLibParameterGroupNumber::AddressClaim) == message.get_identifier().get_parameter_group_number())
		{
			std::uint8_t messageSourceAddress = message.get_identifier().get_source_address();

			update_address_table(messageSourceAddress);
		}
	}

	void CANNetworkManager::update_address_table(std::uint8_t claimedAddress)
	{
		if ((nullptr != controlFunctionTable[claimedAddress]) &&
		    (CANIdentifier::NULL_ADDRESS == controlFunctionTable[claimedAddress]->get_address()))
		{
			// Someone is at that spot in the table, but their address was stolen
			// Need to evict them from the table and move them to the inactive list
			controlFunctionTable[claimedAddress]->address = NULL_CAN_ADDRESS;
			inactiveControlFunctions.push_back(controlFunctionTable[claimedAddress]);
			controlFunctionTable[claimedAddress] = nullptr;
			CANStackLogger::debug("[NM]: %s CF '%d' was evicted from address '%d' in the lookup table.",
			                      controlFunctionTable[claimedAddress]->get_type_string().c_str(),
			                      controlFunctionTable[claimedAddress]->get_NAME().get_full_name(),
			                      claimedAddress);
		}

		// Now, check for either a free spot in the table or recent eviction and populate if needed
		if (nullptr == controlFunctionTable[claimedAddress])
		{
			// Look through all active CFs, maybe one of them has switched addresses
			for (std::uint8_t i = 0; i < NULL_CAN_ADDRESS; i++)
			{
				if ((nullptr != controlFunctionTable[i]) &&
				    (i != claimedAddress) &&
				    (controlFunctionTable[i]->get_address() == claimedAddress))
				{
					controlFunctionTable[claimedAddress] = controlFunctionTable[i];
					controlFunctionTable[i] = nullptr;
					CANStackLogger::debug("[NM]: %s CF '%d' moved from address '%d' to address '%d' in the lookup table.",
					                      controlFunctionTable[claimedAddress]->get_type_string().c_str(),
					                      controlFunctionTable[claimedAddress]->get_NAME().get_full_name(),
					                      i,
					                      claimedAddress);
					break;
				}
			}

			// Look through all inactive CFs, maybe one of them has freshly claimed an address
			for (auto currentControlFunction : inactiveControlFunctions)
			{
				if (currentControlFunction->get_address() == claimedAddress)
				{
					controlFunctionTable[claimedAddress] = currentControlFunction;
					CANStackLogger::debug("[NM]: %s CF '%d' moved from inactive to address '%d' in the lookup table.",
					                      currentControlFunction->get_type_string().c_str(),
					                      currentControlFunction->get_NAME().get_full_name(),
					                      claimedAddress);
					break;
				}
			}
		}
	}

	void CANNetworkManager::update_busload(std::uint32_t numberOfBitsProcessed)
	{
		const std::lock_guard<std::mutex> lock(busloadUpdateMutex);

		currentBusloadBitAccumulator += numberOfBitsProcessed;
	}

	void CANNetworkManager::update_busload_history()
	{
		const std::lock_guard<std::mutex> lock(busloadUpdateMutex);

		if (SystemTiming::time_expired_ms(busloadUpdateTimestamp_ms, BUSLOAD_UPDATE_FREQUENCY_MS))
		{
			busloadMessageBitsHistory.push_back(currentBusloadBitAccumulator);

			while (busloadMessageBitsHistory.size() > (BUSLOAD_SAMPLE_WINDOW_MS / BUSLOAD_UPDATE_FREQUENCY_MS))
			{
				busloadMessageBitsHistory.pop_front();
			}
			currentBusloadBitAccumulator = 0;
			busloadUpdateTimestamp_ms = SystemTiming::get_timestamp_ms();
		}
	}

	void CANNetworkManager::update_control_functions(const CANMessageFrame &rxFrame)
	{
		if ((static_cast<std::uint32_t>(CANLibParameterGroupNumber::AddressClaim) == CANIdentifier(rxFrame.identifier).get_parameter_group_number()) &&
		    (CAN_DATA_LENGTH == rxFrame.dataLength))
		{
			std::uint64_t claimedNAME;
			std::shared_ptr<ControlFunction> foundControlFunction = nullptr;

			claimedNAME = rxFrame.data[0];
			claimedNAME |= (static_cast<std::uint64_t>(rxFrame.data[1]) << 8);
			claimedNAME |= (static_cast<std::uint64_t>(rxFrame.data[2]) << 16);
			claimedNAME |= (static_cast<std::uint64_t>(rxFrame.data[3]) << 24);
			claimedNAME |= (static_cast<std::uint64_t>(rxFrame.data[4]) << 32);
			claimedNAME |= (static_cast<std::uint64_t>(rxFrame.data[5]) << 40);
			claimedNAME |= (static_cast<std::uint64_t>(rxFrame.data[6]) << 48);
			claimedNAME |= (static_cast<std::uint64_t>(rxFrame.data[7]) << 56);

			// Check if the claimed NAME is someone we already know about
			auto activeResult = std::find_if(controlFunctionTable.begin(),
			                                 controlFunctionTable.end(),
			                                 [claimedNAME](const std::shared_ptr<ControlFunction> &cf) {
				                                 return (nullptr != cf) && (cf->controlFunctionNAME.get_full_name() == claimedNAME);
			                                 });
			if (activeResult != controlFunctionTable.end())
			{
				foundControlFunction = *activeResult;
			}
			else
			{
				auto inActiveResult = std::find_if(inactiveControlFunctions.begin(),
				                                   inactiveControlFunctions.end(),
				                                   [claimedNAME, &rxFrame](const std::shared_ptr<ControlFunction> &cf) {
					                                   return cf->controlFunctionNAME.get_full_name() == claimedNAME;
				                                   });
				if (inActiveResult != inactiveControlFunctions.end())
				{
					foundControlFunction = *inActiveResult;
				}
			}

			if (nullptr == foundControlFunction)
			{
				// If we still haven't found it, it might be a partner. Check the list of partners.
				for (std::size_t i = 0; i < PartneredControlFunction::partneredControlFunctionList.size(); i++)
				{
					if ((nullptr != PartneredControlFunction::partneredControlFunctionList[i]) &&
					    (PartneredControlFunction::partneredControlFunctionList[i]->check_matches_name(NAME(claimedNAME))))
					{
						PartneredControlFunction::partneredControlFunctionList[i]->address = CANIdentifier(rxFrame.identifier).get_source_address();
						PartneredControlFunction::partneredControlFunctionList[i]->controlFunctionNAME = NAME(claimedNAME);
						foundControlFunction = std::shared_ptr<ControlFunction>(PartneredControlFunction::partneredControlFunctionList[i]);
						controlFunctionTable[foundControlFunction->get_address()] = foundControlFunction;
						break;
					}
				}
			}

			// Remove any CF that has the same address as the one claiming
			std::for_each(controlFunctionTable.begin(),
			              controlFunctionTable.end(),
			              [&rxFrame, &foundControlFunction](const std::shared_ptr<ControlFunction> &cf) {
				              if ((nullptr != cf) && (foundControlFunction != cf) && (cf->address == CANIdentifier(rxFrame.identifier).get_source_address()))
					              cf->address = CANIdentifier::NULL_ADDRESS;
			              });

			std::for_each(inactiveControlFunctions.begin(),
			              inactiveControlFunctions.end(),
			              [&rxFrame, &foundControlFunction](const std::shared_ptr<ControlFunction> &cf) {
				              if ((foundControlFunction != cf) && (cf->address == CANIdentifier(rxFrame.identifier).get_source_address()))
					              cf->address = CANIdentifier::NULL_ADDRESS;
			              });

			if (nullptr == foundControlFunction)
			{
				// New device, need to start keeping track of it
				foundControlFunction = ControlFunction::create(NAME(claimedNAME), CANIdentifier(rxFrame.identifier).get_source_address(), shared_from_this());
				controlFunctionTable[foundControlFunction->get_address()] = foundControlFunction;
				CANStackLogger::debug("[NM]: New Control function %d", foundControlFunction->get_address());
			}
			else
			{
				CANStackLogger::debug("[NM]: A %s control function (re-)claimed '%d'.",
				                      foundControlFunction->get_type_string().c_str(),
				                      foundControlFunction->get_address());
			}

			if (nullptr != foundControlFunction)
			{
				foundControlFunction->address = CANIdentifier(rxFrame.identifier).get_source_address();
			}
		}
	}

	void CANNetworkManager::update_new_partners()
	{
		if (PartneredControlFunction::anyPartnerNeedsInitializing)
		{
			for (auto &partner : PartneredControlFunction::partneredControlFunctionList)
			{
				if ((nullptr != partner) && (!partner->initialized))
				{
					bool foundReplaceableControlFunction = false;

					// Check this partner against the existing CFs
					for (auto currentInactiveControlFunction = inactiveControlFunctions.begin(); currentInactiveControlFunction != inactiveControlFunctions.end(); currentInactiveControlFunction++)
					{
						if ((partner->check_matches_name((*currentInactiveControlFunction)->get_NAME())) &&
						    (partner->get_associated_network().lock() == shared_from_this()) &&
						    (ControlFunction::Type::External == (*currentInactiveControlFunction)->get_type()))
						{
							foundReplaceableControlFunction = true;

							// This CF matches the filter and is not an internal or already partnered CF
							CANStackLogger::CAN_stack_log(CANStackLogger::LoggingLevel::Debug, "[NM]: Remapping new partner control function to inactive external control function at address " + isobus::to_string(static_cast<int>((*currentInactiveControlFunction)->get_address())));

							// Populate the partner's data
							partner->address = (*currentInactiveControlFunction)->get_address();
							partner->controlFunctionNAME = (*currentInactiveControlFunction)->get_NAME();
							partner->initialized = true;
							inactiveControlFunctions.erase(currentInactiveControlFunction);
							break;
						}
					}

					if (!foundReplaceableControlFunction)
					{
						for (auto currentActiveControlFunction = controlFunctionTable.begin(); currentActiveControlFunction != controlFunctionTable.end(); currentActiveControlFunction++)
						{
							if ((nullptr != (*currentActiveControlFunction)) &&
							    (partner->check_matches_name((*currentActiveControlFunction)->get_NAME())) &&
							    (partner->get_associated_network().lock() == shared_from_this()) &&
							    (ControlFunction::Type::External == (*currentActiveControlFunction)->get_type()))
							{
								foundReplaceableControlFunction = true;

								// This CF matches the filter and is not an internal or already partnered CF
								CANStackLogger::CAN_stack_log(CANStackLogger::LoggingLevel::Debug, "[NM]: Remapping new partner control function to an active external control function at address " + isobus::to_string(static_cast<int>((*currentActiveControlFunction)->get_address())));

								// Populate the partner's data
								partner->address = (*currentActiveControlFunction)->get_address();
								partner->controlFunctionNAME = (*currentActiveControlFunction)->get_NAME();
								partner->initialized = true;
								controlFunctionTable[partner->address] = partner;
								break;
							}
						}
					}
					partner->initialized = true;
				}
			}
			PartneredControlFunction::anyPartnerNeedsInitializing = false;
		}
	}

	CANMessageFrame CANNetworkManager::construct_frame(std::uint8_t sourceAddress,
	                                                   std::uint8_t destAddress,
	                                                   std::uint32_t parameterGroupNumber,
	                                                   std::uint8_t priority,
	                                                   const void *data,
	                                                   std::uint32_t size) const
	{
		CANMessageFrame txFrame;
		txFrame.identifier = DEFAULT_IDENTIFIER;

		if ((NULL_CAN_ADDRESS != destAddress) && (priority <= static_cast<std::uint8_t>(CANIdentifier::CANPriority::PriorityLowest7)) && (size <= CAN_DATA_LENGTH) && (nullptr != data))
		{
			std::uint32_t identifier = 0;

			// Manually encode an identifier
			identifier |= ((priority & 0x07) << 26);
			identifier |= (sourceAddress & 0xFF);

			if (BROADCAST_CAN_ADDRESS == destAddress)
			{
				if ((parameterGroupNumber & 0xF000) >= 0xF000)
				{
					identifier |= ((parameterGroupNumber & 0x3FFFF) << 8);
				}
				else
				{
					identifier |= (destAddress << 8);
					identifier |= ((parameterGroupNumber & 0x3FF00) << 8);
				}
			}
			else
			{
				if ((parameterGroupNumber & 0xF000) < 0xF000)
				{
					identifier |= (destAddress << 8);
					identifier |= ((parameterGroupNumber & 0x3FF00) << 8);
				}
				else
				{
					CANStackLogger::warn("[NM]: Cannot send a message with PGN " +
					                     isobus::to_string(static_cast<int>(parameterGroupNumber)) +
					                     " as a destination specific message. " +
					                     "Try resending it using nullptr as your destination control function.");
					identifier = DEFAULT_IDENTIFIER;
				}
			}

			if (DEFAULT_IDENTIFIER != identifier)
			{
				memcpy(reinterpret_cast<void *>(txFrame.data), data, size);
				txFrame.dataLength = size;
				txFrame.isExtendedFrame = true;
				txFrame.identifier = identifier & 0x1FFFFFFF;
			}
		}
		return txFrame;
	}

	std::shared_ptr<ControlFunction> CANNetworkManager::get_control_function(std::uint8_t address) const
	{
		std::shared_ptr<ControlFunction> retVal = nullptr;

		if ((address < NULL_CAN_ADDRESS))
		{
			retVal = controlFunctionTable[address];
		}
		return retVal;
	}

	CANMessage CANNetworkManager::get_next_can_message_from_rx_queue()
	{
		std::lock_guard<std::mutex> lock(receiveMessageMutex);
		CANMessage retVal = receiveMessageList.front();
		receiveMessageList.pop_front();
		return retVal;
	}

	std::size_t CANNetworkManager::get_number_can_messages_in_rx_queue()
	{
		std::lock_guard<std::mutex> lock(receiveMessageMutex);
		return receiveMessageList.size();
	}

	void CANNetworkManager::process_any_control_function_pgn_callbacks(const CANMessage &currentMessage)
	{
		const std::lock_guard<std::mutex> lock(anyControlFunctionCallbacksMutex);
		for (const auto &currentCallback : anyControlFunctionParameterGroupNumberCallbacks)
		{
			if ((currentCallback.get_parameter_group_number() == currentMessage.get_identifier().get_parameter_group_number()) &&
			    ((nullptr == currentMessage.get_destination_control_function()) ||
			     (ControlFunction::Type::Internal == currentMessage.get_destination_control_function()->get_type())))
			{
				currentCallback.get_callback()(currentMessage, currentCallback.get_parent());
			}
		}
	}

	void CANNetworkManager::process_protocol_pgn_callbacks(const CANMessage &currentMessage)
	{
		const std::lock_guard<std::mutex> lock(protocolPGNCallbacksMutex);
		for (auto &currentCallback : protocolPGNCallbacks)
		{
			if (currentCallback.get_parameter_group_number() == currentMessage.get_identifier().get_parameter_group_number())
			{
				currentCallback.get_callback()(currentMessage, currentCallback.get_parent());
			}
		}
	}

	void CANNetworkManager::process_can_message_for_global_and_partner_callbacks(const CANMessage &message)
	{
		std::shared_ptr<ControlFunction> messageDestination = message.get_destination_control_function();
		if ((nullptr == messageDestination) &&
		    ((nullptr != message.get_source_control_function()) ||
		     ((static_cast<std::uint32_t>(CANLibParameterGroupNumber::ParameterGroupNumberRequest) == message.get_identifier().get_parameter_group_number()) &&
		      (NULL_CAN_ADDRESS == message.get_identifier().get_source_address()))))
		{
			// Message destined to global
			for (std::size_t i = 0; i < globalParameterGroupNumberCallbacks.size(); i++)
			{
				if ((message.get_identifier().get_parameter_group_number() == get_global_parameter_group_number_callback(i).get_parameter_group_number()) &&
				    (nullptr != get_global_parameter_group_number_callback(i).get_callback()))
				{
					// We have a callback that matches this PGN
					get_global_parameter_group_number_callback(i).get_callback()(message, get_global_parameter_group_number_callback(i).get_parent());
				}
			}
		}
		else
		{
			// Message is destination specific
			for (std::size_t i = 0; i < InternalControlFunction::get_number_internal_control_functions(); i++)
			{
				if (messageDestination == InternalControlFunction::get_internal_control_function(i))
				{
					// Message is destined to us
					for (std::size_t j = 0; j < PartneredControlFunction::get_number_partnered_control_functions(); j++)
					{
						std::shared_ptr<PartneredControlFunction> currentControlFunction = PartneredControlFunction::get_partnered_control_function(j);

						if ((nullptr != currentControlFunction) &&
						    (currentControlFunction->get_associated_network().lock() == shared_from_this()))
						{
							// Message matches CAN port for a partnered control function
							for (std::size_t k = 0; k < currentControlFunction->get_number_parameter_group_number_callbacks(); k++)
							{
								if ((message.get_identifier().get_parameter_group_number() == currentControlFunction->get_parameter_group_number_callback(k).get_parameter_group_number()) &&
								    (nullptr != currentControlFunction->get_parameter_group_number_callback(k).get_callback()) &&
								    ((nullptr == currentControlFunction->get_parameter_group_number_callback(k).get_internal_control_function()) ||
								     (currentControlFunction->get_parameter_group_number_callback(k).get_internal_control_function()->get_address() == message.get_identifier().get_destination_address())))
								{
									// We have a callback matching this message
									currentControlFunction->get_parameter_group_number_callback(k).get_callback()(message, currentControlFunction->get_parameter_group_number_callback(k).get_parent());
								}
							}
						}
					}
				}
			}
		}
	}

	void CANNetworkManager::process_can_message_for_commanded_address(const CANMessage &message)
	{
		constexpr std::uint8_t COMMANDED_ADDRESS_LENGTH = 9;

		if ((nullptr == message.get_destination_control_function()) &&
		    (static_cast<std::uint32_t>(CANLibParameterGroupNumber::CommandedAddress) == message.get_identifier().get_parameter_group_number()) &&
		    (COMMANDED_ADDRESS_LENGTH == message.get_data_length()))
		{
			std::uint64_t targetNAME = message.get_uint64_at(0);

			for (std::size_t i = 0; i < InternalControlFunction::get_number_internal_control_functions(); i++)
			{
				// This is not a particularly efficient search, but this should be pretty rare
				auto currentICF = InternalControlFunction::get_internal_control_function(i);

				if ((nullptr != currentICF) &&
				    (currentICF->get_associated_network().lock() == shared_from_this()) &&
				    (currentICF->get_NAME().get_full_name() == targetNAME))
				{
					currentICF->process_commanded_address(message.get_uint8_at(8), {});
				}
			}
		}
	}

	void CANNetworkManager::process_rx_messages()
	{
		while (0 != get_number_can_messages_in_rx_queue())
		{
			CANMessage currentMessage = get_next_can_message_from_rx_queue();

			update_address_table(currentMessage);

			// Update Special Callbacks, like protocols and non-cf specific ones
			process_protocol_pgn_callbacks(currentMessage);
			process_any_control_function_pgn_callbacks(currentMessage);

			// Update Others
			process_can_message_for_global_and_partner_callbacks(currentMessage);
		}
	}

	bool CANNetworkManager::send_can_message_raw(std::uint8_t sourceAddress, std::uint8_t destAddress, std::uint32_t parameterGroupNumber, std::uint8_t priority, const void *data, std::uint32_t size) const
	{
		CANMessageFrame tempFrame = construct_frame(sourceAddress, destAddress, parameterGroupNumber, priority, data, size);
		bool retVal = false;

		if ((DEFAULT_IDENTIFIER != tempFrame.identifier))
		{
			retVal = send_can_message_frame_to_hardware(shared_from_this(), tempFrame);
		}
		return retVal;
	}

	void CANNetworkManager::protocol_message_callback(const CANMessage &message)
	{
		process_can_message_for_global_and_partner_callbacks(message);
		process_can_message_for_commanded_address(message);
	}

} // namespace isobus
