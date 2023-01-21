#include "isobus/isobus/storage_manager.hpp"
#include "isobus/isobus/storage_hardware_abstraction.hpp"

std::vector<isobus::StorageManager::StorageManager::ReadStorageCallbackInfo> isobus::StorageManager::storageReadCallbacks;

isobus::StorageManager::ReadStorageCallbackInfo::ReadStorageCallbackInfo(const ReadStorageCallback callback, void *parent) :
  callback(callback), parent(parent)
{
}

void isobus::StorageManager::ReadStorageCallbackInfo::call_callback(const StorageEntryType id, const std::vector<std::uint8_t> data)
{
	callback(id, data, parent);
}

bool isobus::StorageManager::ReadStorageCallbackInfo::operator==(const ReadStorageCallbackInfo &obj)
{
	return obj.callback == callback && obj.parent == parent;
}

bool isobus::StorageManager::add_storage_read_callback(ReadStorageCallback callback, void *parentPointer)
{
	bool retVal = false;
	ReadStorageCallbackInfo callbackInfo(callback, parentPointer);

	std::lock_guard<std::mutex> lock(storageReadCallbackMutex);
	auto location = std::find(storageReadCallbacks.begin(), storageReadCallbacks.end(), callbackInfo);
	if (location != storageReadCallbacks.end())
	{
		storageReadCallbacks.push_back(callbackInfo);
		retVal = true;
	}
	return retVal;
}

bool isobus::StorageManager::remove_storage_read_callback(ReadStorageCallback callback, void *parentPointer)
{
	bool retVal = false;
	ReadStorageCallbackInfo callback(callback, parentPointer);

	std::lock_guard<std::mutex> lock(storageReadCallbackMutex);
	auto location = std::find(storageReadCallbacks.begin(), storageReadCallbacks.end(), callback);
	if (location != storageReadCallbacks.end())
	{
		storageReadCallbacks.erase(location);
		retVal = true;
	}
	return retVal;
}

void isobus::StorageManager::process_storage_read(const std::uint64_t id, const std::vector<std::uint8_t> data)
{
	std::lock_guard<std::mutex> lock(storageReadCallbackMutex);
	for (auto &callback : storageReadCallbacks)
	{
		/// @todo somehow check if the `id` is a valid StorageEntryType?
		callback.call_callback(static_cast<StorageEntryType>(id), data);
	}
}

bool isobus::StorageManager::request_read_storage(const StorageEntryType id)
{
	return add_storage_read_request(static_cast<std::uint64_t>(id));
}

bool isobus::StorageManager::request_write_storage(const StorageEntryType id, const std::vector<std::uint8_t> &data)
{
	return add_storage_write_request(static_cast<std::uint64_t>(id), data);
}
