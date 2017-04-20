// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "MavLinkLog.hpp"
#include "Utils.hpp"
#include <chrono>

using namespace mavlinkcom;
using namespace mavlink_utils;

MavLinkLog::MavLinkLog()
{
	ptr_ = nullptr;
	reading_ = false;
	writing_ = false;
	json_ = false;
}

MavLinkLog::~MavLinkLog()
{
	close();
}

bool MavLinkLog::isOpen()
{
	return reading_ || writing_;
}
void MavLinkLog::openForReading(const std::string& filename)
{
	close();
	file_name_ = filename;
	ptr_ = fopen(filename.c_str(), "rb");
	if (ptr_ == nullptr) {
		throw std::runtime_error(Utils::stringf("Could not open the file %s, error=%d", filename.c_str(), errno));
	}

    fseek(ptr_, 0, SEEK_SET);
	reading_ = true;
	writing_ = false;
}
void MavLinkLog::openForWriting(const std::string& filename, bool json)
{
	close();
	json_ = json;
	file_name_ = filename;
	ptr_ = fopen(filename.c_str(), "wb");
	if (ptr_ == nullptr) {
		throw std::runtime_error(Utils::stringf("Could not open the file %s, error=%d", filename.c_str(), errno));
	}
	if (json) {
		fprintf(ptr_, "{ \"rows\": [\n");
	}
	reading_ = false;
	writing_ = true;
}

void MavLinkLog::close()
{
	FILE* temp = ptr_;
	if (json_ && ptr_ != nullptr) {
        fprintf(ptr_, "    {}\n"); // so that trailing comma on last row isn't a problem.
		fprintf(ptr_, "]}\n");
	}
	ptr_ = nullptr;
	if (temp != nullptr) {
		fclose(temp);
	}
	reading_ = false;
	writing_ = false;
}


uint64_t FlipEndianness(uint64_t v)
{
	uint64_t result = 0;
	uint64_t shift = v;
	const int size = sizeof(uint64_t);
	for (int i = 0; i < size; i++)
	{
		uint64_t low = (shift & 0xff);
		result = (result << 8) + low;
		shift >>= 8;
	}
	return result;
}

uint64_t MavLinkLog::getTimeStamp()
{
    return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
}

void MavLinkLog::write(const mavlinkcom::MavLinkMessage& msg, uint64_t timestamp)
{
	if (ptr_ != nullptr) {
		if (reading_) {
			throw std::runtime_error("Log file was opened for reading");
		}
		if (json_) {
			MavLinkMessageBase* strongTypedMsg = MavLinkMessageBase::lookup(msg);
			if (strongTypedMsg != nullptr) {
                strongTypedMsg->timestamp = timestamp;
				std::string line = strongTypedMsg->toJSon();
				fprintf(ptr_, "    %s\n", line.c_str());
                delete strongTypedMsg;
			}
		}
		else {
            if (timestamp == 0) {
                timestamp = getTimeStamp();
            }
			// for compatibility with QGroundControl we have to save the time field in big endian.
            timestamp = FlipEndianness(timestamp);
			fwrite(&timestamp, sizeof(uint64_t), 1, ptr_);
			fwrite(&msg.magic, 1, 1, ptr_);
			fwrite(&msg.len, 1, 1, ptr_);
			fwrite(&msg.seq, 1, 1, ptr_);
			fwrite(&msg.sysid, 1, 1, ptr_);
			fwrite(&msg.compid, 1, 1, ptr_);
			fwrite(&msg.msgid, 1, 1, ptr_);
			fwrite(&msg.payload64, 1, msg.len, ptr_);
			fwrite(&msg.checksum, sizeof(uint16_t), 1, ptr_);
		}
	}
}

bool MavLinkLog::read(mavlinkcom::MavLinkMessage& msg, uint64_t& timestamp)
{
	if (ptr_ != nullptr) {
		if (writing_) {
			throw std::runtime_error("Log file was opened for writing");
		}
		uint64_t time;

		size_t s = fread(&time, 1, sizeof(uint64_t), ptr_);
		if (s < sizeof(uint64_t)) {
            int hr = errno;            
			return false;
		}

        timestamp = FlipEndianness(time);

		s = fread(&msg.magic, 1, 1, ptr_);
		if (s == 0) {
			return false;
		}
		s = fread(&msg.len, 1, 1, ptr_);
		if (s == 0) {
			return false;
		}
		s = fread(&msg.seq, 1, 1, ptr_);
		if (s == 0) {
			return false;
		}
		s = fread(&msg.sysid, 1, 1, ptr_);
		if (s == 0) {
			return false;
		}
		s = fread(&msg.compid, 1, 1, ptr_);
		if (s == 0) {
			return false;
		}
		s = fread(&msg.msgid, 1, 1, ptr_);
		if (s < 1) {
			return false;
		}
		s = fread(&msg.payload64, 1, msg.len, ptr_);
		if (s < msg.len) {
			return false;
		}
		s = fread(&msg.checksum, 1, sizeof(uint16_t), ptr_);
		if (s < sizeof(uint16_t)) {
			return false;
		}
		return true;
	}
	return false;
}
