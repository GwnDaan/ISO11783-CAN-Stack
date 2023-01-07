cmake_minimum_required(VERSION 3.16)
project(hardware_gateway)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(isobus CONFIG) # Normally, you set REQUIRED, however if building
                            # from top level, it already exists
find_package(Threads REQUIRED)

if("WindowsSerial" IN_LIST CAN_DRIVER)
  add_executable(MainAppExampleTarget main.cpp)

  target_link_libraries(
    MainAppExampleTarget PRIVATE isobus::Isobus isobus::HardwareIntegration
                                Threads::Threads isobus::SystemTiming)
endif()

if("LinuxSerial" IN_LIST CAN_DRIVER AND "SocketCAN" IN_LIST CAN_DRIVER)
  add_executable(GatewayExampleTarget gateway.cpp)

  target_link_libraries(
    GatewayExampleTarget PRIVATE isobus::Isobus isobus::HardwareIntegration
                                Threads::Threads isobus::SystemTiming)
endif()