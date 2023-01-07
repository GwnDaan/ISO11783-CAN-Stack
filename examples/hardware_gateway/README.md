# Hardware Gateway Example

This example shows how to use the CAN stack to create a gateway between two CAN networks. Specifically, it shows how to relay messages via a serial connection from a SocketCAN network to a Windows machine, and vice versa. However, it can be easily adapted to use your platform of choice.
This could be usefull if you do your development on a Windows machine, but don't have a Windows CAN adapter (like PCAN-Basic) laying around. If you have a Linux machine around with a SocketCAN connection to your CAN netowrk, you can use this example to connect to let your Linux machine act a gateway. 

## Parts
This example is tested and working on the following setup:

* Windows 10 PC
* Raspberry Pi Zero 2W
* [MCP2515 CAN HAT for Raspberry Pi](https://www.waveshare.com/wiki/RS485_CAN_HAT)  
* Micro USB to USB A cable

These are just some parts I had laying around, but you can use any setup that has a CAN network on one end, and a serial connection on the other.

## Gateway Application

### Setup
By default, the `can0` interface is used. To setup the interface, you can use the following command:
```bash
sudo ip link set can0 up type can bitrate 250000
```
Here we set the bitrate to 250kbps, but you can change this to your desired bitrate.

### Building and running
See the [examples README](../README.md) for instructions on how to build and run the application.
Make sure to enable the `LinuxSerial` and `SocketCAN` drivers as explained in [repo README](../../README.md#Compilation). E.g.:
```bash
cmake -S . -B build -DCAN_DRIVER="LinuxSerial;SocketCAN"
```

With the application running and data flowing through the CAN network, you should see output similar to the following:
```bash
pi@tractorcan:~/isobus-stack $ ./build/examples/hardware_gateway/GatewayExampleTarget
[Channel 0]: Received: 419358758, length: 8
[Channel 0]: Received: 419358758, length: 8
[Channel 0]: Received: 419358758, length: 8
[Channel 0]: Received: 419358758, length: 8
[Channel 0]: Received: 419358758, length: 8
[Channel 0]: Received: 484900646, length: 8
[Channel 0]: Received: 419358758, length: 8
```
Now we are ready to connect the other end of the gateway. For this example, we will use a Windows machine, see the [Windows Application Example](#Windows-Application-Example) for more information.

### Driver Properties
The relaying part of the system is done by the `gateway.cpp` application. This application is a simple CAN stack application that uses the `LinuxSerial` driver to communicate with the serial port. By default, the application will open the serial port at `/dev/ttyGS0`. Furthermore, the application is configured to use the `SocketCAN` driver to communicate with the CAN network. Which is configured to use the `can0` interface. If you want to use different drivers or ports, you can change this in the `gateway.cpp` file.

When changing the drivers, make sure to also change the `if` statement in the `CMakeLists.txt` file. This is to make sure that the `gateway.cpp` file is only compiled when the desired drivers are enabled:
```cmake
if("LinuxSerial" IN_LIST CAN_DRIVER AND "SocketCAN" IN_LIST CAN_DRIVER)
  add_executable(GatewayExampleTarget gateway.cpp)

  ...
  
endif()
```

## Windows Application Example
### Setup
By default the application will open the serial port at `COM10`. If you want to use a different port, you can change this in the `windows_application.cpp` file. To find the correct port, you can use the `Device Manager` in Windows.

### Building and running
See the [examples README](../README.md) for instructions on how to build and run the application.
Make sure to enable the `WindowsSerial` driver as explained in [repo README](../../README.md#Compilation). E.g.:
```bash
cmake -S . -B build -DCAN_DRIVER="WindowsSerial"
```

With the application running and data flowing through from the Gateway Application over serial, you should see output similar to the following:
```bash
PS C:\...\ISO11783-CAN-Stack\build\examples\hardware_gateway\Debug> ."MainAppExampleTarget.exe" 
Received: 419358758, length: 8
Received: 419358758, length: 8
Received: 419358758, length: 8
Received: 419358758, length: 8
Received: 484900646, length: 8
Received: 419358758, length: 8
Received: 419358758, length: 8
Received: 419358758, length: 8
Received: 419358758, length: 8
Received: 419358758, length: 8
Received: 484900646, length: 8
```

### Adapting to other projects
If you want to use this Windows driver in your own project, this can easily be done by just specifying the `WindowsSerial` driver to the `CANHardwareInterface` like you normally would with e.g. the `SocketCAN` driver:
```cpp
WindowsSerialInterface serialDriver(10); // COM10
CANHardwareInterface::set_number_of_can_channels(1);
CANHardwareInterface::assign_can_channel_frame_handler(0, &serialDriver);

if ((!CANHardwareInterface::start()) || (!serialDriver.get_is_valid()))
{
        std::cout << "Failed to connect to the socket. The interface might be down." << std::endl;
}
```
Don't forget to enable the `WindowsSerial` driver as explained before and in [repo README](../../README.md#Compilation).