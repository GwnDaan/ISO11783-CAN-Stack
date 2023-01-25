//================================================================================================
/// @file storage_hardware_interface.hpp
///
/// @brief Provides a layer to queue storage requests for reading and writing.
/// @author Daan Steenbergen
///
/// @copyright 2023 Adrian Del Grosso
//================================================================================================
#ifndef STORAGE_HARDWARE_INTERFACE_HPP
#define STORAGE_HARDWARE_INTERFACE_HPP

#include <cstdint>
#include <deque>
#include <mutex>
#include <vector>

#include "isobus/hardware_integration/storage_hardware_plugin.hpp"

//================================================================================================
/// @class StorageHardwareInterface
///
/// @brief A class to queue storage requests for reading and writing.
//================================================================================================
class StorageHardwareInterface
{
public:
	/// @brief A callback function for reading data from storage
	void typedef (*ReadStorageCallback)(const std::uint64_t id, const std::vector<std::uint8_t> data, void *parentPointer);

	/// @brief A class to store information about Rx callbacks
	class ReadStorageCallbackInfo
	{
	public:
		/// @brief Constructs a `ReadStorageCallbackInfo`
		ReadStorageCallbackInfo(const ReadStorageCallback callback, void *parent);

		/// @brief Calls the callback
		/// @param[in] id The unique identifier of the data
		/// @param[in] data The data that was read
		void call_callback(const std::uint64_t id, const std::vector<std::uint8_t> data);

		/// @brief Allows easy comparison of callback data
		/// @param obj the object to compare against
		bool operator==(const ReadStorageCallbackInfo &obj);

	private:
		ReadStorageCallback callback; ///< The callback
		void *parent; ///< Context variable, the owner of the callback
	};

	/// @brief Adds a storage read callback. The added callback will be called any time a storage read request is processed.
	/// @param[in] callback The callback to add
	/// @param[in] parentPointer Generic context variable, usually a pointer to the owner class for this callback
	/// @returns `true` if the callback was added, `false` if it was already in the list
	static bool add_storage_read_callback(ReadStorageCallback callback, void *parentPointer = nullptr);

	/// @brief Removes a storage read callback
	/// @param[in] callback The callback to remove
	/// @param[in] parentPointer Generic context variable, usually a pointer to the owner class for this callback
	/// @returns `true` if the callback was removed, `false` if no callback matched the two parameters
	static bool remove_storage_read_callback(ReadStorageCallback callback, void *parentPointer = nullptr);

	/// @brief Sets the storage driver to use
	/// @param[in] storageDriver The storage driver to use
	/// @returns `true` if the storage driver was set, otherwise `false`.
	static bool set_storage_handler(StorageHardwarePlugin *storageDriver);

	/// @brief Called externally, adds storage data to the storage write queue
	/// @param[in] id The unique identifier of the data
	/// @param[in] data The data to add to the storage write queue
	/// @returns `true` if the data was accepted, otherwise `false`
	static bool add_storage_write_request(const std::uint64_t id, const std::vector<std::uint8_t> &data);

	/// @brief Called externally, adds a storage read request to the storage read queue
	/// @param[in] id The unique identifier of the data to read
	/// @returns `true` if the request was accepted, otherwise `false`
	static bool add_storage_read_request(const std::uint64_t id);

	/// @brief Updates the storage driver, should be called at a regular interval
	static void update();

private:
	static StorageHardwareInterface STORAGE_HARDWARE_INTERFACE; ///< Static singleton instance of this class

	/// @brief Private constructor, prevents more of these classes from being needlessly created
	StorageHardwareInterface();

	/// @brief Private destructor
	~StorageHardwareInterface();

	/// @brief Handles the last item in the storage write queue
	static void process_write_queue_item();

	/// @brief Handles the last item in the storage read queue
	static void process_read_queue_item();

	static std::mutex storageDataToBeWrittenMutex; ///< Mutex to protect the storage write queue
	static std::deque<std::pair<std::uint64_t, std::vector<std::uint8_t>>> storageDataToBeWritten; ///< Storage write queue

	static std::mutex storageReadRequestsMutex; ///< Mutex to protect the storage read queue
	static std::deque<std::uint64_t> storageReadRequests; ///< Storage read queue
	static std::vector<ReadStorageCallbackInfo> storageReadCallbacks; ///< Storage read callbacks

	static StorageHardwarePlugin *storageHandler; ///< The storage driver to use
};

#endif // STORAGE_HARDWARE_INTERFACE_HPP
