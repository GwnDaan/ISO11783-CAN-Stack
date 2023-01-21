//================================================================================================
/// @file storage_manager.hpp
///
/// @brief A class that manages the storage that the stack wants to read/write to.
/// @author Adrian Del Grosso
///
/// @copyright 2022 Adrian Del Grosso
//================================================================================================

#ifndef STORAGE_MANAGER_HPP
#define STORAGE_MANAGER_HPP

#include <mutex>

namespace isobus
{
	//================================================================================================
	/// @class StorageManager
	///
	/// @brief A class that manages the storage needed for some functionalities the stack.
	//================================================================================================
	class StorageManager
	{
	public:
		enum class StorageEntryType : std::uint64_t
		{
			Unknown = 0, //< Unknown storage entry
			Reserved = 1, //< Reserved storage entry
			VTClientPreferredAssignment = 2, //< The preferred assignments for the VT client Aux N implementation
		};

		/// @brief A callback function for reading data from storage
		void typedef (*ReadStorageCallback)(const StorageEntryType id, const std::vector<std::uint8_t> data, void *parentPointer);

		/// @brief A class to store information about Rx callbacks
		class ReadStorageCallbackInfo
		{
		public:
			/// @brief Constructs a `ReadStorageCallbackInfo`
			ReadStorageCallbackInfo(const ReadStorageCallback callback, void *parent);

			/// @brief Calls the callback
			/// @param[in] id The unique identifier of the data
			/// @param[in] data The data that was read
			void call_callback(const StorageEntryType id, const std::vector<std::uint8_t> data);

			/// @brief Allows easy comparison of callback data
			/// @param obj the object to compare against
			bool operator==(const ReadStorageCallbackInfo &obj);

		private:
			const ReadStorageCallback callback; ///< The callback
			void *parent; ///< Context variable, the owner of the callback
		};

		/// @brief Adds a storage read callback. The added callback will be called any time a storage read request is received.
		/// @param[in] callback The callback to add
		/// @param[in] parentPointer Generic context variable, usually a pointer to the owner class for this callback
		/// @returns `true` if the callback was added, `false` if it was already in the list
		static bool add_storage_read_callback(ReadStorageCallback callback, void *parentPointer = nullptr);

		/// @brief Removes a storage read callback
		/// @param[in] callback The callback to remove
		/// @param[in] parentPointer Generic context variable, usually a pointer to the owner class for this callback
		/// @returns `true` if the callback was removed, `false` if no callback matched the two parameters
		static bool remove_storage_read_callback(ReadStorageCallback callback, void *parentPointer = nullptr);

		/// @brief Processes a response to a storage read request
		/// @param[in] id The unique identifier of the data
		/// @param[in] data The data that was read
		static void process_storage_read(const std::uint64_t id, const std::vector<std::uint8_t> data);

		/// @brief Write data to storage
		/// @param[in] id The unique identifier of the data
		/// @param[in] data The data to write
		/// @returns `true` if the data will be written, `false` otherwise
		static bool request_write_storage(const StorageEntryType id, const std::vector<std::uint8_t> &data);

		/// @brief Read data from storage
		/// @param[in] id The unique identifier of the data
		/// @returns `true` if the data will be read, `false` otherwise
		static bool request_read_storage(const StorageEntryType id);

	private:
		static StorageManager StorageNetwork; ///< Static singleton instance of this class
		static std::mutex storageReadCallbackMutex; ///< Mutex to protect the storage read callbacks
		static std::vector<ReadStorageCallbackInfo> storageReadCallbacks; ///< Storage read callbacks

		/// @brief Constructor for the StorageManager
		StorageManager();
	};

} // namespace isobus

#endif // STORAGE_MANAGER_HPP
