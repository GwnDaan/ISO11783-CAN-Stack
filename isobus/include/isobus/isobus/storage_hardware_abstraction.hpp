//================================================================================================
/// @file storage_hardware_abstraction.hpp
///
/// @brief An abstraction between this CAN stack and any hardware layer
/// @author Daan Steenbergen
///
/// @copyright 2023 Adrian Del Grosso
//================================================================================================

#ifndef STORAGE_HARDWARE_ABSTRACTION
#define STORAGE_HARDWARE_ABSTRACTION

#include <cstdint>
#include <vector>

namespace isobus
{
	/// @brief Write data to storage via a hardware abstraction layer
	/// @param[in] id The unique identifier of the data
	/// @param[in] data The data to write to storage
	/// @returns `true` if the data will be writen to storage, otherwise `false`
	bool add_storage_write_request(const std::uint64_t id, const std::vector<std::uint8_t> &data);

	/// @brief Read data from storage via a hardware abstraction layer
	/// @param[in] id The unique identifier of the data
	/// @returns `true` if the data will be read from storage, otherwise `false`
	bool add_storage_read_request(std::uint64_t id);

} // namespace isobus

#endif // STORAGE_HARDWARE_ABSTRACTION