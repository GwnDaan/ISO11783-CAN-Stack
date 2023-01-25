#include "isobus/hardware_integration/file_storage_plugin.hpp"
#include <fstream>
#include <iostream>

FileStoragePlugin::FileStoragePlugin(const std::string &dir, const std::string &suffix) :
  dir(dir), suffix(suffix)
{
}

FileStoragePlugin::~FileStoragePlugin()
{
}

bool FileStoragePlugin::write_data(const std::uint64_t id, const std::vector<std::uint8_t> &data)
{
	bool retVal = false;
	std::ofstream file(dir + std::to_string(id), std::ios::out | std::ios::binary);
	if (file.is_open())
	{
		file.clear();
		file.write(reinterpret_cast<const char *>(data.data()), data.size());
		file.close();
		retVal = true;
	}
	return retVal;
}

bool FileStoragePlugin::read_data(const std::uint64_t id, std::vector<std::uint8_t> &data)
{
	bool retVal = false;
	std::ifstream file(dir + std::to_string(id), std::ios::in | std::ios::binary);
	if (file.is_open())
	{
		file.seekg(0, std::ios::end);
		std::streampos fileSize = file.tellg();
		file.seekg(0, std::ios::beg);
		data.reserve(fileSize);
		data.insert(data.begin(), std::istream_iterator<std::uint8_t>(file), std::istream_iterator<std::uint8_t>());
		file.close();
		retVal = true;
	}
	return retVal;
}
