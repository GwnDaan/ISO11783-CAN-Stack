//================================================================================================
/// @file windows_serial_interface.hpp
///
/// @brief An interface for sending raw HardwareInterfaceCANFrames over the serial port on Windows
/// @note Only designed for testing purposes where two SerialInterfaces of this stack are
/// Connected to each other.
/// @author Daan Steenbergen
///
/// @copyright 2022 Adrian Del Grosso
//================================================================================================
#ifndef WINDOWS_SERIAL_INTERFACE_HPP
#define WINDOWS_SERIAL_INTERFACE_HPP

#include <Windows.h>
#include <string>

#include "isobus/hardware_integration/can_hardware_plugin.hpp"
#include "isobus/isobus/can_frame.hpp"

//================================================================================================
/// @class WindowsSerialInterface
///
/// @brief A CAN Driver for Windows socket CAN
//================================================================================================
class WindowsSerialInterface : public CANHardwarePlugin
{
public:
	static constexpr std::size_t FRAME_LENGTH = sizeof(isobus::HardwareInterfaceCANFrame); ///< The length of a hardare frame in bytes

	/// @brief Constructor for the serial windows CAN driver
	/// @param[in] portFileName The port number of the port to use, like `1` for "COM1"
	explicit WindowsSerialInterface(const std::uint8_t portNumber);

	/// @brief The destructor for WindowsSerialInterface
	~WindowsSerialInterface();

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
	const std::uint8_t portNumber; ///< The file name of the port
	HANDLE handle; ///< The handler for the serial port
	bool connected; ///< If the serial port is connected
};

#endif // WINDOWS_SERIAL_INTERFACE_HPP
