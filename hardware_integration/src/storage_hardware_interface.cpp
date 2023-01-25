#include "isobus/hardware_integration/storage_hardware_interface.hpp"
#include "isobus/isobus/can_stack_logger.hpp"
#include "isobus/isobus/storage_hardware_abstraction.hpp"
#include "isobus/utility/to_string.hpp"

#include <algorithm>

std::mutex StorageHardwareInterface::storageDataToBeWrittenMutex;
std::deque<std::pair<std::uint64_t, std::vector<std::uint8_t>>> StorageHardwareInterface::storageDataToBeWritten;

std::mutex StorageHardwareInterface::storageReadRequestsMutex;
std::deque<std::uint64_t> StorageHardwareInterface::storageReadRequests;
std::vector<StorageHardwareInterface::ReadStorageCallbackInfo> StorageHardwareInterface::storageReadCallbacks;

StorageHardwarePlugin *StorageHardwareInterface::storageHandler = nullptr;

StorageHardwareInterface::ReadStorageCallbackInfo::ReadStorageCallbackInfo(const ReadStorageCallback callback, void *parent) :
  callback(callback), parent(parent)
{
}

void StorageHardwareInterface::ReadStorageCallbackInfo::call_callback(const std::uint64_t id, const std::vector<std::uint8_t> data)
{
	callback(id, data, parent);
}

bool StorageHardwareInterface::ReadStorageCallbackInfo::operator==(const ReadStorageCallbackInfo &obj)
{
	return obj.callback == callback && obj.parent == parent;
}

bool StorageHardwareInterface::set_storage_handler(StorageHardwarePlugin *storageDriver)
{
	storageHandler = storageDriver;
	return true;
}

bool StorageHardwareInterface::add_storage_write_request(const std::uint64_t id, const std::vector<std::uint8_t> &data)
{
	std::lock_guard<std::mutex> lock(storageDataToBeWrittenMutex);
	storageDataToBeWritten.push_back(std::make_pair(id, data));
	return true;
}

bool isobus::add_storage_write_request(const std::uint64_t id, const std::vector<std::uint8_t> &data)
{
	return StorageHardwareInterface::add_storage_write_request(id, data);
}

bool StorageHardwareInterface::add_storage_read_request(const std::uint64_t id)
{
	std::lock_guard<std::mutex> lock(storageReadRequestsMutex);
	storageReadRequests.push_back(id);
	return true;
}

bool isobus::add_storage_read_request(std::uint64_t id)
{
	return StorageHardwareInterface::add_storage_read_request(id);
}

void StorageHardwareInterface::update()
{
	process_read_queue_item();
	process_write_queue_item();
}

bool StorageHardwareInterface::add_storage_read_callback(ReadStorageCallback callback, void *parentPointer)
{
	bool retVal = false;
	ReadStorageCallbackInfo callbackInfo(callback, parentPointer);

	std::lock_guard<std::mutex> lock(storageReadRequestsMutex);
	auto location = std::find(storageReadCallbacks.begin(), storageReadCallbacks.end(), callbackInfo);
	if (location == storageReadCallbacks.end())
	{
		storageReadCallbacks.push_back(callbackInfo);
		retVal = true;
	}
	return retVal;
}

bool StorageHardwareInterface::remove_storage_read_callback(ReadStorageCallback callback, void *parentPointer)
{
	bool retVal = false;
	ReadStorageCallbackInfo callbackInfo(callback, parentPointer);

	std::lock_guard<std::mutex> lock(storageReadRequestsMutex);
	auto location = std::find(storageReadCallbacks.begin(), storageReadCallbacks.end(), callbackInfo);
	if (location != storageReadCallbacks.end())
	{
		storageReadCallbacks.erase(location);
		retVal = true;
	}
	return retVal;
}

void StorageHardwareInterface::process_read_queue_item()
{
	std::lock_guard<std::mutex> lock(storageReadRequestsMutex);
	if (storageReadRequests.size() > 0)
	{
		std::uint64_t id = storageReadRequests.front();
		storageReadRequests.pop_front();
		if (storageHandler != nullptr)
		{
			std::vector<std::uint8_t> data;
			if (storageHandler->read_data(id, data))
			{
				for (auto callback : storageReadCallbacks)
				{
					callback.call_callback(id, data);
				}
			}
			else
			{
				isobus::CANStackLogger::error("[Storage]: Failed to read data with id " + isobus::to_string(static_cast<int>(id)));
			}
		}
		else
		{
			isobus::CANStackLogger::error("[Storage]: No storage handler set, cannot read data with id " + isobus::to_string(static_cast<int>(id)));
		}
	}
}

void StorageHardwareInterface::process_write_queue_item()
{
	std::lock_guard<std::mutex> lock(storageDataToBeWrittenMutex);
	if (storageDataToBeWritten.size() > 0)
	{
		auto data = storageDataToBeWritten.front();
		storageDataToBeWritten.pop_front();
		if (storageHandler != nullptr)
		{
			if (!storageHandler->write_data(data.first, data.second))
			{
				isobus::CANStackLogger::error("[Storage]: Failed to write data with id " + isobus::to_string(static_cast<int>(data.first)));
			}
		}
		else
		{
			isobus::CANStackLogger::error("[Storage]: No storage handler set, cannot write data with id " + isobus::to_string(static_cast<int>(data.first)));
		}
	}
}

StorageHardwareInterface::StorageHardwareInterface()
{
}

StorageHardwareInterface::~StorageHardwareInterface()
{
}
