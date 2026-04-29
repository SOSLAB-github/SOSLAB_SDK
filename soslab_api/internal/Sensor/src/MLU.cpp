#include "MLU.h"

#include <fstream>
#include <iomanip>
#include <sstream>
#include <chrono>
# include <math.h>

soslab::MLU::MLU()
{

	//MLU Parameter
	numCol = 256;
	numRow = 192;
	numTotalSize = numCol * numRow;

	numSegmentCol = 256;
	numSegmentRow = 6;

	frameDataVec.resize(4);

	for (auto& frameData : frameDataVec)
	{
		frameData.echoNum = 1;
		frameData.lidarId = 0;
		frameData.timestamp.resize(32);
		frameData.ambient.resize(numTotalSize);
		frameData.depth[0].resize(numTotalSize);
		frameData.intensity[0].resize(numTotalSize);
		frameData.points[0].resize(numTotalSize);
		frameData.rows = numRow;
		frameData.cols = numCol;
	}
	std::cout << "MLU created." << std::endl;
}

soslab::MLU::~MLU()
{
	std::cout << "MLU deleted." << std::endl;
}

bool soslab::MLU::supports(Feature f) const
{
	switch (f)
	{
	case Feature::StreamEnable:
		return true;
	default:
		std::cerr << "Selected feature is not supported in MLU.\n";
		return false;
	}
}

bool soslab::MLU::buildCommand(const Request& req, soslab::MessageBase& msg, std::vector<std::vector<uint8_t>>& totalProtocol)
{
	if (!supports(req.feature))
	{
		std::cerr << "Wrong feature to build MLU command.\n";
		return false;
	}

	switch (req.feature)
	{
	case Feature::StreamEnable:		totalProtocol = createBooleanMessage("command", msg); break;
	default:						std::cerr << "Wrong feature to build MLU command.\n"; return false;

	}

	return !totalProtocol.empty();
}

bool soslab::MLU::parseCommand(const Request& req, const std::vector<uint8_t>& pkt, soslab::MessageBase& out, bool& isFinish)
{
	bool retval = false;
	switch (req.feature)
	{
	case Feature::StreamEnable:		retval = validateJsonAck(pkt, out); break;
	default:				std::cerr << "Wrong feature to build MLU command.\n"; return false;
	}

	return retval;
}


soslab::packetStatus soslab::MLU::classifyPacket(const std::vector<uint8_t>& pkt) const
{
	if (pkt.size() < sizeof(header::headerMLU)) return packetStatus::INVALID;

	header::headerMLU h{};
	std::memcpy(&h, pkt.data(), sizeof(header::headerMLU));

	if (memcmp(h.header, "MUUSR", 5) == 0)
		return packetStatus::STREAM;

	return packetStatus::UNKNOWN;
}

std::vector<std::vector<uint8_t>> soslab::MLU::createBooleanMessage(std::string key, const soslab::MessageBase& dtn)
{
	std::vector<std::vector<uint8_t>> totalCmd;
	std::vector<uint8_t> cmd;
	soslab::Message::GeneralMessage<bool> boolMsg = static_cast<const soslab::Message::GeneralMessage<bool>&>(dtn);

	json_t payload;

	payload[key] = (boolMsg.data) ? "run" : "stop";
	std::string jsonStr = payload.dump();

	cmd.assign(jsonStr.begin(), jsonStr.end());
	totalCmd.push_back(cmd);
	return totalCmd;
}

bool soslab::MLU::validateJsonAck(const std::vector<uint8_t>& ack, soslab::MessageBase& out)
{
	bool retval = false;
	if (json_t::accept(ack.begin(), ack.end()))
	{
		json_t ackJson;
		ackJson = json_t::parse(ack.begin(), ack.end());
		if (ackJson.contains("json_ack"))
		{
			retval = true;
			return ackJson["json_ack"].get<bool>();
		}
	}

	return retval;
}

bool soslab::MLU::parseStreamData(const std::vector<uint8_t>& packetData)
{
	return buildStreamData(packetData);
}

bool soslab::MLU::buildStreamData(const std::vector<uint8_t>& packetData)
{
	bool retval = false;
	header::headerMLU headerInfo;

	int hroll = 0;
	int vroll = 0;
	int offset = 0;

	memcpy(reinterpret_cast<char*>(&headerInfo), packetData.data(), sizeof(header::headerMLU));
	offset += sizeof(header::headerMLU);

	int numbering = 0;

	if (memcmp(headerInfo.header, "MUUSR", 5) == 0)
	{
		numbering = headerInfo.header[7] - '0';

		hroll = (headerInfo.hroll_vroll) >> 6;
		vroll = headerInfo.hroll_vroll & 0x3F;
	}
	else
	{
		return false;
	}

	FrameData& frameData = frameDataVec[numbering];
	frameData.lidarId = numbering;
	frameData.timestamp[vroll] = headerInfo.timestamp;

	for (int r = 0; r < numSegmentRow; r++)
	{
		for (int c = 0; c < numSegmentCol; c++)
		{
			uint64_t xyzi = 0;
			memcpy(reinterpret_cast<char*>(&xyzi), packetData.data() + offset, sizeof(uint64_t));
			offset += sizeof(uint64_t);

			int32_t x = xyzi & 0x1FFFF;
			int32_t sign_x = (xyzi >> 17) & 0x1;
			int32_t y = (xyzi >> 18) & 0x1FFFF;
			int32_t sign_y = (xyzi >> 35) & 0x1;
			int32_t z = (xyzi >> 36) & 0x1FFFF;
			int32_t sign_z = (xyzi >> 53) & 0x1;

			uint16_t intensity = (xyzi >> 54) & 0x3FF;

			uint8_t image = 0;
			memcpy(reinterpret_cast<char*>(&image), packetData.data() + offset, sizeof(uint8_t));
			offset += sizeof(uint8_t);
			uint16_t upper_intensity = (image & 0x3);
			intensity = intensity | (upper_intensity << 10);

			uint32_t ambient = image;

			memcpy(reinterpret_cast<char*>(&ambient), packetData.data() + offset, sizeof(uint32_t));
			offset += sizeof(uint32_t);

			int cIdx = c;
			int rInx = r + (vroll * numSegmentRow);
			int idx = (rInx * numSegmentCol) + cIdx;

			frameData.ambient[idx] = (uint32_t)ambient;
			frameData.intensity[0][idx] = (uint32_t)intensity;
			frameData.points[0][idx].x = (sign_x == 1) ? -(float)x : (float)x;
			frameData.points[0][idx].y = (sign_y == 1) ? -(float)y : (float)y;
			frameData.points[0][idx].z = (sign_z == 1) ? -(float)z : (float)z;
			frameData.depth[0][idx] = std::sqrt((float)x * (float)x + (float)y * (float)y + (float)z * (float)z);
		}
	}
	if (vroll == 31)
	{
		if (frameBuffer != nullptr)
		{
			frameData.rows = numRow;
			frameData.cols = numCol;
			frameBuffer->push(std::make_shared<FrameData>(frameData));
		}
	}

	return retval;
}