#include <gtest/gtest.h>

#include "isobus/hardware_integration/can_hardware_interface.hpp"
#include "isobus/hardware_integration/virtual_can_plugin.hpp"
#include "isobus/isobus/can_NAME_filter.hpp"
#include "isobus/isobus/can_constants.hpp"
#include "isobus/isobus/can_internal_control_function.hpp"
#include "isobus/isobus/can_network_manager.hpp"
#include "isobus/isobus/can_partnered_control_function.hpp"

#include <chrono>
#include <thread>

using namespace isobus;

TEST(ADDRESS_CLAIM_TESTS, PartneredClaim)
{
	auto firstNetwork = std::make_shared<CANNetworkManager>();
	auto secondNetwork = std::make_shared<CANNetworkManager>();
	CANHardwareInterface::assign_can_channel_frame_handler(firstNetwork, std::make_shared<VirtualCANPlugin>());
	CANHardwareInterface::assign_can_channel_frame_handler(secondNetwork, std::make_shared<VirtualCANPlugin>());
	CANHardwareInterface::start();

	std::this_thread::sleep_for(std::chrono::milliseconds(250));

	NAME firstName(0);
	firstName.set_arbitrary_address_capable(true);
	firstName.set_industry_group(1);
	firstName.set_device_class(0);
	firstName.set_function_code(static_cast<std::uint8_t>(isobus::NAME::Function::CabClimateControl));
	firstName.set_identity_number(1);
	firstName.set_ecu_instance(0);
	firstName.set_function_instance(0);
	firstName.set_device_class_instance(0);
	firstName.set_manufacturer_code(69);
	auto firstInternalECU = InternalControlFunction::create(firstName, 0x1C, firstNetwork);

	isobus::NAME secondName(0);
	secondName.set_arbitrary_address_capable(true);
	secondName.set_industry_group(1);
	secondName.set_device_class(0);
	secondName.set_function_code(static_cast<std::uint8_t>(isobus::NAME::Function::SeatControl));
	secondName.set_identity_number(2);
	secondName.set_ecu_instance(0);
	secondName.set_function_instance(0);
	secondName.set_device_class_instance(0);
	secondName.set_manufacturer_code(69);
	auto secondInternalECU2 = InternalControlFunction::create(secondName, 0x1D, secondNetwork);

	const NAMEFilter filterSecond(NAME::NAMEParameters::FunctionCode, static_cast<std::uint8_t>(NAME::Function::SeatControl));
	auto firstPartneredSecondECU = PartneredControlFunction::create(firstNetwork, { filterSecond });
	const isobus::NAMEFilter filterFirst(NAME::NAMEParameters::FunctionCode, static_cast<std::uint8_t>(NAME::Function::CabClimateControl));
	auto secondPartneredFirstEcu = PartneredControlFunction::create(secondNetwork, { filterFirst });

	std::this_thread::sleep_for(std::chrono::milliseconds(500));
	EXPECT_TRUE(firstInternalECU->get_address_valid());
	EXPECT_TRUE(secondInternalECU2->get_address_valid());
	EXPECT_TRUE(firstPartneredSecondECU->get_address_valid());
	EXPECT_TRUE(secondPartneredFirstEcu->get_address_valid());

	CANHardwareInterface::stop();
	ASSERT_TRUE(firstPartneredSecondECU->destroy());
	ASSERT_TRUE(secondPartneredFirstEcu->destroy());
	ASSERT_TRUE(firstInternalECU->destroy());
	ASSERT_TRUE(secondInternalECU2->destroy());
}
