#include "isobus/hardware_integration/windows_serial_interface.hpp"
#include "isobus/isobus/can_warning_logger.hpp"
#include "isobus/utility/to_string.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <chrono>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>

WindowsSerialInterface::WindowsSerialInterface(const std::uint8_t portNumber) :
  portNumber(portNumber),
  connected(false)
{
}

WindowsSerialInterface::~WindowsSerialInterface()
{
	close();
}

bool WindowsSerialInterface::get_is_valid() const
{
	return connected;
}

void WindowsSerialInterface::open()
{
	handler = CreateFileA(static_cast<LPCSTR>(("\\\\.\\COM" + std::to_string(portNumber)).c_str()),
	                      GENERIC_READ | GENERIC_WRITE,
	                      0,
	                      NULL,
	                      OPEN_EXISTING,
	                      FILE_ATTRIBUTE_NORMAL,
	                      NULL);

	if (INVALID_HANDLE_VALUE != handler)
	{
		DCB state;

		if (GetCommState(handler, &state))
		{
			state.BaudRate = CBR_115200;
			state.ByteSize = 8;
			state.StopBits = ONESTOPBIT;
			state.Parity = NOPARITY;
			if (SetCommState(handler, &state))
			{
				PurgeComm(handler, PURGE_RXCLEAR | PURGE_TXCLEAR);
				connected = true;
			}
			else
			{
				isobus::CANStackLogger::CAN_stack_log("[Windows-Serial]: Failed to write new COM" + std::to_string(portNumber) + " state");
				close();
			}
		}
		else
		{
			isobus::CANStackLogger::CAN_stack_log("[Windows-Serial]: Failed to get current COM" + std::to_string(portNumber) + " state");
			close();
		}
	}
	else if (GetLastError() == ERROR_FILE_NOT_FOUND)
	{
		isobus::CANStackLogger::CAN_stack_log("[Windows-Serial]: COM" + std::to_string(portNumber) + " not available");
		close();
	}
}

void WindowsSerialInterface::close()
{
	CloseHandle(handler);
	handler = nullptr;
	connected = false;
}

bool WindowsSerialInterface::write_frame(const isobus::HardwareInterfaceCANFrame &canFrame)
{
	bool retVal = false;

	DWORD bytesWritten;
	if (WriteFile(handler, &canFrame, FRAME_LENGTH, &bytesWritten, NULL))
	{
		retVal = true;
	}
	else
	{
		isobus::CANStackLogger::CAN_stack_log("[Windows-Serial]: Failed to write can frame to COM" + std::to_string(portNumber));
		close();
	}
	return retVal;
}

bool WindowsSerialInterface::read_frame(isobus::HardwareInterfaceCANFrame &canFrame)
{
	bool retVal = false;

	DWORD bytesRead;
	if (ReadFile(handler, &canFrame, FRAME_LENGTH, &bytesRead, NULL))
	{
		if (FRAME_LENGTH == bytesRead)
		{
			retVal = true;
		}
	}
	else
	{
		isobus::CANStackLogger::CAN_stack_log("[Windows-Serial]: Failed to read can frame from COM" + std::to_string(portNumber));
		close();
	}
	return retVal;
}
