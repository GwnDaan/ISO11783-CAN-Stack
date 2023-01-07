//================================================================================================
/// @file linux_serial_interface.hpp
///
/// @brief An interface for sending raw HardwareInterfaceCANFrames over the serial port on Linux
/// @note Only designed for testing purposes where two SerialInterfaces of this stack are
/// Connected to each other.
/// @author Daan Steenbergen
///
/// @copyright 2022 Adrian Del Grosso
//================================================================================================
#ifndef LINUX_SERIAL_INTERFACE_HPP
#define LINUX_SERIAL_INTERFACE_HPP

#include <string>

#include "isobus/hardware_integration/can_hardware_plugin.hpp"
#include "isobus/isobus/can_frame.hpp"

//================================================================================================
/// @class LinuxSerialInterface
///
/// @brief A CAN Driver for Linux socket CAN
//================================================================================================
class LinuxSerialInterface : public CANHardwarePlugin
{
public:
	static constexpr std::size_t FRAME_LENGTH = sizeof(isobus::HardwareInterfaceCANFrame); ///< The length of a hardare frame in bytes

	/// @brief Constructor for the serial linux CAN driver
	/// @param[in] portFileName The file name of the port to use, like "/dev/ttyUSB0" or "/dev/ttyS0"
	explicit LinuxSerialInterface(const std::string portFileName);

	/// @brief The destructor for LinuxSerialInterface
	~LinuxSerialInterface();

	/// @brief Returns if the socket connection is valid
	/// @returns `true` if connected, `false` if not connected
	bool get_is_valid() const override;

	/// @brief Closes the socket
	void close() override;

	/// @brief Connects to the socket
	void open() override;

	/// @brief Returns a frame from the serial port (synchronous), or `false` if no frame can be read.
	/// @param[in, out] canFrame The CAN frame that was read
	/// @returns `true` if a CAN frame was read, otherwise `false`
	bool read_frame(isobus::HardwareInterfaceCANFrame &canFrame) override;

	/// @brief Writes a frame to the serial port (synchronous)
	/// @param[in] canFrame The frame to write to the bus
	/// @returns `true` if the frame was written, otherwise `false`
	bool write_frame(const isobus::HardwareInterfaceCANFrame &canFrame) override;

private:
	const std::string portFileName; ///< The file name of the port
	int fileDescriptor; ///< File descriptor for the serial port
};

#endif // LINUX_SERIAL_INTERFACE_HPP
