#include "isobus/hardware_integration/windows_serial_interface.hpp"
#include "isobus/isobus/can_stack_logger.hpp"
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
  connected(false),
  handle(INVALID_HANDLE_VALUE)
{
}

WindowsSerialInterface::~WindowsSerialInterface()
{
	close();
}

bool WindowsSerialInterface::get_is_valid() const
{
	return connected && (INVALID_HANDLE_VALUE != handle);
}

void WindowsSerialInterface::open()
{
	handle = CreateFile(TEXT(("\\\\.\\COM" + std::to_string(portNumber)).c_str()),
	                    GENERIC_READ | GENERIC_WRITE,
	                    0,
	                    NULL,
	                    OPEN_EXISTING,
	                    FILE_FLAG_OVERLAPPED,
	                    NULL);

	if (INVALID_HANDLE_VALUE != handle)
	{
		DCB state;

		if (GetCommState(handle, &state))
		{
			state.BaudRate = CBR_115200;
			state.ByteSize = 8;
			state.StopBits = ONESTOPBIT;
			state.Parity = NOPARITY;
			if (SetCommState(handle, &state))
			{
				COMMTIMEOUTS timeouts;
				if (GetCommTimeouts(handle, &timeouts))
				{
					timeouts.ReadIntervalTimeout = 0;
					timeouts.ReadTotalTimeoutConstant = 0;
					timeouts.ReadTotalTimeoutMultiplier = 0;
					timeouts.WriteTotalTimeoutConstant = 0;
					timeouts.WriteTotalTimeoutMultiplier = 0;
					if (SetCommTimeouts(handle, &timeouts))
					{
						PurgeComm(handle, PURGE_RXCLEAR | PURGE_TXCLEAR);
						connected = true;
					}
					else
					{
						isobus::CANStackLogger::error("[Windows-Serial]: Error " + std::to_string(GetLastError()) + ", failed to set COM" + std::to_string(portNumber) + " timeouts");
						close();
					}
				}
				else
				{
					isobus::CANStackLogger::error("[Windows-Serial]: Error " + std::to_string(GetLastError()) + ", failed to get COM" + std::to_string(portNumber) + " timeouts");
					close();
				}
			}
			else
			{
				isobus::CANStackLogger::error("[Windows-Serial]: Error " + std::to_string(GetLastError()) + ", failed to write new COM" + std::to_string(portNumber) + " state");
				close();
			}
		}
		else
		{
			isobus::CANStackLogger::error("[Windows-Serial]: Error " + std::to_string(GetLastError()) + ", failed to get current COM" + std::to_string(portNumber) + " state");
			close();
		}
	}
	else
	{
		if (GetLastError() == ERROR_FILE_NOT_FOUND)
		{
			isobus::CANStackLogger::error("[Windows-Serial]: COM" + std::to_string(portNumber) + " not available");
		}
		else
		{
			isobus::CANStackLogger::error("[Windows-Serial]: Error " + std::to_string(GetLastError()) + ", failed to connected to COM" + std::to_string(portNumber));
		}
		close();
	}
}

void WindowsSerialInterface::close()
{
	if (INVALID_HANDLE_VALUE != handle)
	{
		CloseHandle(handle);
		handle = INVALID_HANDLE_VALUE;
	}
	connected = false;
}

bool WindowsSerialInterface::write_frame(const isobus::HardwareInterfaceCANFrame &canFrame)
{
	bool retVal = false;

	OVERLAPPED overlapped = { 0 };
	overlapped.hEvent = CreateEventA(NULL, TRUE, FALSE, NULL);
	if (NULL != overlapped.hEvent)
	{
		BOOL status = WriteFile(handle, &canFrame, FRAME_LENGTH, NULL, &overlapped);
		if (status || (GetLastError() == ERROR_IO_PENDING))
		{
			DWORD result = WaitForSingleObject(overlapped.hEvent, INFINITE);

			if (CloseHandle(overlapped.hEvent))
			{
				retVal = (result == WAIT_OBJECT_0);
			}
			else
			{
				isobus::CANStackLogger::error("[Windows-Serial]: Error " + std::to_string(GetLastError()) + ", failed to close event for write for COM" + std::to_string(portNumber));
				close();
			}
			retVal = true;
		}
		else
		{
			isobus::CANStackLogger::error("[Windows-Serial]: Error " + std::to_string(GetLastError()) + ", failed to write can frame to COM" + std::to_string(portNumber));
			close();
		}
	}
	else
	{
		isobus::CANStackLogger::error("[Windows-Serial]: Error " + std::to_string(GetLastError()) + ", failed to create overlapped event for write for COM" + std::to_string(portNumber));
		close();
	}
	return retVal;
}

bool WindowsSerialInterface::read_frame(isobus::HardwareInterfaceCANFrame &canFrame)
{
	bool retVal = false;

	OVERLAPPED overlapped = { 0 };
	overlapped.hEvent = CreateEventA(NULL, TRUE, FALSE, NULL);
	if (NULL != overlapped.hEvent)
	{
		BOOL status = ReadFile(handle, &canFrame, FRAME_LENGTH, NULL, &overlapped);
		if (status || (GetLastError() == ERROR_IO_PENDING))
		{
			DWORD result = WaitForSingleObject(overlapped.hEvent, INFINITE);

			if (CloseHandle(overlapped.hEvent))
			{
				retVal = (result == WAIT_OBJECT_0);
			}
			else
			{
				isobus::CANStackLogger::error("[Windows-Serial]: Error " + std::to_string(GetLastError()) + ", failed to close event for read for COM" + std::to_string(portNumber));
				close();
			}
		}
		else
		{
			isobus::CANStackLogger::error("[Windows-Serial]: Error " + std::to_string(GetLastError()) + ", failed to read can frame from COM" + std::to_string(portNumber));
			close();
		}
	}
	else
	{
		isobus::CANStackLogger::error("[Windows-Serial]: Error " + std::to_string(GetLastError()) + ", failed to create event for read for COM" + std::to_string(portNumber));
		close();
	}
	return retVal;
}
