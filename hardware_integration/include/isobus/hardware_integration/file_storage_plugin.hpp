//================================================================================================
/// @file file_storage_plugin.hpp
///
/// @brief Provides a plugin to read and write binary data to a file.
/// @author Daan Steenbergen
///
/// @copyright 2023 Adrian Del Grosso
//================================================================================================
#ifndef FILE_STORAGE_PLUGIN_HPP
#define FILE_STORAGE_PLUGIN_HPP

#include "isobus/hardware_integration/storage_hardware_plugin.hpp"

#include <string>

//================================================================================================
/// @class FileStoragePlugin
///
/// @brief A plugin to read and write binary data to a file.
//================================================================================================
class FileStoragePlugin : public StorageHardwarePlugin
{
public:
	/// @brief Constructs a `FileStoragePlugin`
	/// @param[in] dir The directory to read and write to
	/// @param[in] suffix The suffix to append to the file name
	explicit FileStoragePlugin(const std::string &dir, const std::string &suffix = ".dat");

	/// @brief Destroys a `FileStoragePlugin`
	virtual ~FileStoragePlugin();

	/// @brief Writes data to the file
	/// @param[in] id The unique identifier of the data
	/// @param[in] data The data to write
	/// @return `true` if the data is written, `false` if an error occurred
	bool write_data(const std::uint64_t id, const std::vector<std::uint8_t> &data) override;

	/// @brief Reads data from the file
	/// @param[in] id The unique identifier of the data
	/// @param[in, out] data The data that will be read
	/// @return `true` if the data is read, `false` if an error occurred
	bool read_data(const std::uint64_t id, std::vector<std::uint8_t> &data) override;

private:
	std::string dir; ///< The directory to read and write to
	std::string suffix; ///< The suffix to append to the file name
};

#endif // FILE_STORAGE_PLUGIN_HPP
