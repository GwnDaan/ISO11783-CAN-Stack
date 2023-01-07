#include "isobus/hardware_integration/can_hardware_interface.hpp"
#include "isobus/hardware_integration/linux_serial_interface.hpp"
#include "isobus/hardware_integration/socket_can_interface.hpp"
#include "isobus/isobus/can_warning_logger.hpp"

#include <string.h>
#include <csignal>
#include <iostream>

static const std::string socketCANDevice = "can0";
static const std::string serialDevice = "/dev/ttyGS0";

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

void signal_handler(int signum)
{
	CANHardwareInterface::stop(); // Clean up the threads
	exit(signum);
}

void gateway_glue(isobus::HardwareInterfaceCANFrame &rawFrame, void *)
{
	std::cout << "[Channel " << (int)rawFrame.channel << "]: Received: " << (int)rawFrame.identifier << ", length: " << (int)rawFrame.dataLength << std::endl; // TODO: remove debug
	// Relay the message to the other CAN interface
	if (0 == rawFrame.channel)
	{
		rawFrame.channel = 1;
		CANHardwareInterface::transmit_can_message(rawFrame);
	}
	else
	{
		rawFrame.channel = 0;
		CANHardwareInterface::transmit_can_message(rawFrame);
	}
}

int main()
{
	isobus::CANStackLogger::set_can_stack_logger_sink(&logger);

	SocketCANInterface canDriver(socketCANDevice);
	LinuxSerialInterface serialDriver(serialDevice);

	// Set up the hardware layer to use both our canDriver and serialDriver ("can0" and "/dev/ttyGS0" by default)
	CANHardwareInterface::set_number_of_can_channels(2);
	CANHardwareInterface::assign_can_channel_frame_handler(0, &canDriver);
	CANHardwareInterface::assign_can_channel_frame_handler(1, &serialDriver);

	if ((!CANHardwareInterface::start()) || (!serialDriver.get_is_valid()) || (!canDriver.get_is_valid()))
	{
		std::cout << "Failed to initialize. An interface might not have started." << std::endl;
	}

	// Handle control+c
	std::signal(SIGINT, signal_handler);

	CANHardwareInterface::add_raw_can_message_rx_callback(gateway_glue, nullptr);

	while (true)
	{
		// CAN stack runs in other threads. Do nothing forever.
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	}

	// Clean up the threads
	CANHardwareInterface::stop();

	return 0;
}