#include "isobus/hardware_integration/can_hardware_interface.hpp"
#include "isobus/hardware_integration/socket_can_interface.hpp"
#include "isobus/hardware_integration/windows_serial_interface.hpp"
#include "isobus/isobus/can_network_manager.hpp"
#include "isobus/isobus/can_partnered_control_function.hpp"
#include "isobus/isobus/can_warning_logger.hpp"

#include <csignal>
#include <iostream>
#include <memory>

static constexpr std::uint8_t COM_PORT_INDEX = 10; ///< The COM port index to use for the serial interface (e.g. index 10 for COM10)

// A log sink for the CAN stack
class CustomLogger : public isobus::CANStackLogger
{
public:
	void LogCANLibWarning(const std::string &text) override
	{
		std::cout << text << std::endl; // Write the text to stdout
	}
};

static CustomLogger logger;

void update_CAN_network()
{
	isobus::CANNetworkManager::CANNetwork.update();
}

void raw_can_glue(isobus::HardwareInterfaceCANFrame &rawFrame, void *parentPointer)
{
	std::cout << "Received: " << (int)rawFrame.identifier << ", length: " << (int)rawFrame.dataLength << std::endl; // TODO: remove debug
	isobus::CANNetworkManager::CANNetwork.can_lib_process_rx_message(rawFrame, parentPointer);
}

void signal_handler(int signum)
{
	CANHardwareInterface::stop(); // Clean up the threads
	exit(signum);
}

int main()
{
	isobus::CANStackLogger::set_can_stack_logger_sink(&logger);

	WindowsSerialInterface serialDriver(COM_PORT_INDEX);

	// Set up the hardware layer to use serial driver ("COM10" by default)
	CANHardwareInterface::set_number_of_can_channels(1);
	CANHardwareInterface::assign_can_channel_frame_handler(0, &serialDriver);

	if ((!CANHardwareInterface::start()) || (!serialDriver.get_is_valid()))
	{
		std::cout << "Failed to connect to the socket. The interface might be down." << std::endl;
	}

	// Handle control+c
	std::signal(SIGINT, signal_handler);

	CANHardwareInterface::add_can_lib_update_callback(update_CAN_network, nullptr);
	CANHardwareInterface::add_raw_can_message_rx_callback(raw_can_glue, nullptr);

	// ...
	// any other code you want to run
	// ...

	while (true)
	{
		// CAN stack runs in other threads. Do nothing forever.
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	}

	// Clean up the threads
	CANHardwareInterface::stop();

	return 0;
}