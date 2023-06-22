//================================================================================================
/// @file can_hardware_abstraction.hpp
///
/// @brief An abstraction between this CAN stack and any hardware layer
/// @author Adrian Del Grosso
///
/// @copyright 2022 Adrian Del Grosso
//================================================================================================

#ifndef CAN_HARDWARE_ABSTRACTION_HPP
#define CAN_HARDWARE_ABSTRACTION_HPP

#include "isobus/isobus/can_message_frame.hpp"
#include "isobus/isobus/can_network_manager.hpp"

#include <cstdint>
#include <memory>

namespace isobus
{
	/// @brief The sending abstraction layer between the hardware and the stack
	/// @param[in] frame The frame to transmit from the hardware
	/// @param[in] associatedNetwork The manager that is associated with this message frame
	/// @returns True if the frame was sent successfully, false otherwise
	bool send_can_message_frame_to_hardware(const std::weak_ptr<const CANNetworkManager> associatedNetwork, const CANMessageFrame &frame);

	/// @brief The receiving abstraction layer between the hardware and the stack
	/// @param[in] frame The frame to receive from the hardware
	/// @param[in] associatedNetwork The manager that is associated with this message frame
	void receive_can_message_frame_from_hardware(const std::weak_ptr<const CANNetworkManager> associatedNetwork, const CANMessageFrame &frame);

	/// @brief Informs the network manager whenever messages are emitted on the bus
	/// @param[in] txFrame The CAN frame that was just emitted
	/// @param[in] associatedNetwork The manager that is associated with this message frame
	void on_transmit_can_message_frame_from_hardware(const std::weak_ptr<const CANNetworkManager> associatedNetwork, const CANMessageFrame &txFrame);

	/// @brief The periodic update abstraction layer between the hardware and the stacks
	/// @param[in] network The network to update
	void periodic_update_from_hardware(std::shared_ptr<const CANNetworkManager> network);

} // namespace isobus

#endif // CAN_HARDWARE_ABSTRACTION_HPP
