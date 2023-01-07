#include "isobus/hardware_integration/linux_serial_interface.hpp"
#include "isobus/isobus/can_warning_logger.hpp"
#include "isobus/utility/to_string.hpp"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

LinuxSerialInterface::LinuxSerialInterface(const std::string portFileName) :
  portFileName(portFileName)
{
}

LinuxSerialInterface::~LinuxSerialInterface()
{
	close();
}

bool LinuxSerialInterface::get_is_valid() const
{
	return (-1 != fileDescriptor);
}

void LinuxSerialInterface::open()
{
	fileDescriptor = ::open(portFileName.c_str(), O_RDWR);

	if (0 <= fileDescriptor)
	{
		struct termios tty;
		if (tcgetattr(fileDescriptor, &tty) == 0)
		{
			tty.c_cflag &= ~PARENB;
			tty.c_cflag &= ~CSTOPB;
			tty.c_cflag &= ~CSIZE;
			tty.c_cflag |= CS8;
			tty.c_cflag &= ~CRTSCTS;
			tty.c_cflag |= CREAD | CLOCAL;

			tty.c_lflag &= ~ICANON;
			tty.c_lflag &= ~ECHO;
			tty.c_lflag &= ~ECHOE;
			tty.c_lflag &= ~ECHONL;
			tty.c_lflag &= ~ISIG;
			tty.c_iflag &= ~(IXON | IXOFF | IXANY);
			tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

			tty.c_oflag &= ~OPOST;
			tty.c_oflag &= ~ONLCR;

			tty.c_cc[VTIME] = 10; // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
			tty.c_cc[VMIN] = 0;

			cfsetispeed(&tty, B115200);
			cfsetospeed(&tty, B115200);

			if (tcsetattr(fileDescriptor, TCSANOW, &tty) != 0)
			{
				isobus::CANStackLogger::CAN_stack_log("[Linux-Serial]: (" + portFileName + ") Error " + isobus::to_string(errno) + " from tcsetattr: " + strerror(errno));
				close();
			}
		}
		else
		{
			isobus::CANStackLogger::CAN_stack_log("[Linux-Serial]: (" + portFileName + ") Error " + isobus::to_string(errno) + " from tcgetattr: " + strerror(errno));
			close();
		}
	}
	else
	{
		isobus::CANStackLogger::CAN_stack_log("[Linux-Serial]: (" + portFileName + ") Error " + isobus::to_string(errno) + " from open: " + strerror(errno));
		close();
	}
}

void LinuxSerialInterface::close()
{
	::close(fileDescriptor);
	fileDescriptor = -1;
}

bool LinuxSerialInterface::write_frame(const isobus::HardwareInterfaceCANFrame &canFrame)
{
	if (write(fileDescriptor, &canFrame, FRAME_LENGTH) > 0)
	{
		return true;
	}
	return false;
}

bool LinuxSerialInterface::read_frame(isobus::HardwareInterfaceCANFrame &canFrame)
{
	std::uint8_t readBuffer[FRAME_LENGTH];

	if (read(fileDescriptor, readBuffer, FRAME_LENGTH) > 0)
	{
		memcpy(&canFrame, readBuffer, FRAME_LENGTH);
		return true;
	}
	return false;
}
