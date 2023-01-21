//================================================================================================
/// @file storage_hardware_plugi.hpp
///
/// @brief A base class for a storage driver. Can be derived into your platform's required interface.
/// @author Daan Steenbergen
///
/// @copyright 2023 Adrian Del Grosso
//================================================================================================
#ifndef STORAGE_HARDWARE_PLUGIN_HPP
#define STORAGE_HARDWARE_PLUGIN_HPP

#include <cstdint>
#include <vector>

//================================================================================================
/// @class StorageHardwarePlugin
///
/// @brief An abstract base class for a storage driver
//================================================================================================
class StorageHardwarePlugin
{
public:
	/// @brief Writes data to the storage, synchronously
	/// @param[in] index The unique identifier of the data
	/// @param[in] data The data to write
	/// @returns `true` if the data is stored succesfully, otherwise `false`
	virtual bool write_data(const std::uint64_t id, const std::vector<std::uint8_t> &data) = 0;

	/// @brief Reads data from the storage, synchronously
	/// @param[in] index The unique identifier of the data
	/// @param[in, out] data The data that was read
	/// @returns `true` if the data is read succesfully, otherwise `false`
	virtual bool read_data(const std::uint64_t id, std::vector<std::uint8_t> &data) = 0;
};

#endif // CAN_HARDEWARE_PLUGIN_HPP
