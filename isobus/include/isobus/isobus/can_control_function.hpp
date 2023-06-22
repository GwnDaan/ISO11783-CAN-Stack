//================================================================================================
/// @file can_control_function.hpp
///
/// @brief Defines a base class to represent a generic ISOBUS control function.
/// @author Adrian Del Grosso
/// @author Daan Steenbergen
///
/// @copyright 2022 Adrian Del Grosso
//================================================================================================

#ifndef CAN_CONTROL_FUNCTION_HPP
#define CAN_CONTROL_FUNCTION_HPP

#include "isobus/isobus/can_NAME.hpp"

#include <memory>
#include <mutex>

namespace isobus
{
	class CANNetworkManager;

	//================================================================================================
	/// @class ControlFunction
	///
	/// @brief A class that describes an ISO11783 control function, which includes a NAME and address.
	//================================================================================================
	class ControlFunction : public std::enable_shared_from_this<ControlFunction>
	{
	public:
		/// @brief The type of the control function
		enum class Type
		{
			Internal, ///< The control function is part of our stack and can address claim
			External, ///< The control function is some other device on the bus
			Partnered ///< An external control function that you explicitly want to talk to
		};

		virtual ~ControlFunction() = default;

		/// @brief The factory function to construct a control function
		/// @param[in] NAMEValue The NAME of the control function
		/// @param[in] addressValue The current address of the control function
		/// @param[in] network The network that the control function is associated with
		static std::shared_ptr<ControlFunction> create(NAME NAMEValue, std::uint8_t addressValue, std::shared_ptr<CANNetworkManager> network);

		/// @brief Destroys this control function, by removing it from the network manager
		/// @param[in] expectedRefCount The expected number of shared pointers to this control function after removal
		/// @returns true if the control function was successfully removed from everywhere in the stack, otherwise false
		virtual bool destroy(std::uint32_t expectedRefCount = 1);

		/// @brief Returns the current address of the control function
		/// @returns The current address of the control function
		std::uint8_t get_address() const;

		/// @brief Describes if the control function has a valid address (not NULL or global)
		/// @returns true if the address is < 0xFE
		bool get_address_valid() const;

		/// @brief Returns the network that this control function is associated with
		/// @returns A weak pointer to the associated network object
		std::weak_ptr<CANNetworkManager> get_associated_network() const;

		/// @brief Returns the NAME of the control function as described by its address claim message
		/// @returns The control function's NAME
		NAME get_NAME() const;

		/// @brief Returns the `Type` of the control function
		/// @returns The control function type
		Type get_type() const;

		///@brief Returns the 'Type' of the control function as a string
		///@returns The control function type as a string
		std::string get_type_string() const;

	protected:
		/// @brief The protected constructor for the control function, which is called by the (inherited) factory function
		/// @param[in] NAMEValue The NAME of the control function
		/// @param[in] addressValue The current address of the control function
		/// @param[in] network The network that the control function is associated with
		ControlFunction(NAME NAMEValue, std::uint8_t addressValue, std::shared_ptr<CANNetworkManager> network, Type type = Type::External);

		friend class CANNetworkManager;
		static std::mutex controlFunctionProcessingMutex; ///< Protects the control function tables
		const Type controlFunctionType; ///< The Type of the control function
		NAME controlFunctionNAME; ///< The NAME of the control function
		std::uint8_t address; ///< The address of the control function
		const std::weak_ptr<CANNetworkManager> associatedNetwork; ///< The network that this control function is associated with
	};

} // namespace isobus

#endif // CAN_CONTROL_FUNCTION_HPP
