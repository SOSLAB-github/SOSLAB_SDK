#include "MLA.h"

#include <fstream>
#include <iomanip>
#include <sstream>
#include <chrono>
# include <math.h>

soslab::MLA::MLA()
{
	std::cout << "MLA created." << std::endl;

	//MLA Parameter
	int numCol = 192;
	int numAmbCol = numCol * 3;
	int numRow = 56;
	int numTotalSize = numCol * numRow;
	int numTotalAmbSize = numCol * numAmbCol;
	frameDataVec.resize(4);

	for (auto& frameData : frameDataVec)
	{
		frameData.echoNum = 1;
		frameData.lidarId = 0;
		frameData.frameNum = 0;
		frameData.timestamp.resize(numRow);
		frameData.ambient.resize(numTotalAmbSize);
		frameData.depth[0].resize(numTotalSize);
		frameData.intensity[0].resize(numTotalSize);
		frameData.points[0].resize(numTotalSize);
		frameData.rows = numRow;
		frameData.cols = numCol;
	}
}

soslab::MLA::~MLA()
{
	std::cout << "MLA deleted." << std::endl;
}

bool soslab::MLA::supports(Feature f) const
{
	switch (f)
	{
	case Feature::StreamEnable:
	case Feature::EthernetInfo:
	case Feature::SaveSetting:
		return true;
	default:
		std::cerr << "Selected feature is not supported in MLA.\n";
		return false;
	}
}

bool soslab::MLA::buildCommand(const Request& req, soslab::MessageBase& msg, std::vector<std::vector<uint8_t>>& totalProtocol)
{
	if (!supports(req.feature))
	{
		std::cerr << "Wrong feature to build MLA command.\n";
		return false;
	}

	switch (req.feature)
	{
	case Feature::StreamEnable:		totalProtocol = createBooleanMessage("command", msg);	break;
	case Feature::EthernetInfo:		totalProtocol = req.mode == SetMode::Set ? createNetworkSettingMessage(msg) : createRequestMessage("command", req); break;
	case Feature::SaveSetting:		totalProtocol = createJsonMessage("command", msg);	break;
	default:				std::cerr << "Wrong feature to build MLA command.\n"; return false;

	}

	return !totalProtocol.empty();
}

bool soslab::MLA::parseCommand(const Request& req, const std::vector<uint8_t>& pkt, soslab::MessageBase& out, bool& isFinish)
{
	bool retval = false;
	Feature feature = req.feature;
	switch (feature)
	{
	case Feature::StreamEnable:		retval = validateJsonAck(pkt, out);			break;
	case Feature::EthernetInfo:		retval = req.mode == SetMode::Set ? validateJsonAck(pkt, out) : validateEthernetInfoJson(pkt, out); break;
	case Feature::SaveSetting:		retval = validateJsonAck(pkt, out);			break;
	default:				std::cerr << "Wrong feature to build MLA command.\n"; return false;
	}

	return retval;
}

std::vector<std::vector<uint8_t>> soslab::MLA::createBooleanMessage(std::string key, const soslab::MessageBase& dtn)
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

bool soslab::MLA::validateJsonAck(const std::vector<uint8_t>& ack, soslab::MessageBase& out)
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

std::vector<std::vector<uint8_t>> soslab::MLA::createJsonMessage(std::string key, const soslab::MessageBase& dtn)
{
	std::vector<std::vector<uint8_t>> totalCmd;
	std::vector<uint8_t> cmd;
	soslab::Message::GeneralMessage<std::string> msg = static_cast<const soslab::Message::GeneralMessage<std::string>&>(dtn);

	json_t payload;

	payload[key] = msg.data;
	std::string jsonStr = payload.dump();

	cmd.assign(jsonStr.begin(), jsonStr.end());
	totalCmd.push_back(cmd);
	return totalCmd;
}

std::vector<std::vector<uint8_t>> soslab::MLA::createNetworkSettingMessage(const soslab::MessageBase& dtn)
{
	std::vector<std::vector<uint8_t>> totalCmd;

	const auto& msg = static_cast<const soslab::Message::EthernetInfoMessage&>(dtn);

	json_t setting;
	setting["gateway"] = msg.gatewayAddr;
	setting["ml_ip"] = msg.sensorIp;
	setting["ml_port"] = msg.sensorPort;
	setting["netmask"] = msg.subnetMask;
	setting["pc_ip"] = msg.pcIp;
	setting["pc_port"] = msg.pcPort;

	json_t payload;
	payload["IP_Setting"] = setting;

	std::string jsonStr = payload.dump();
	std::vector<uint8_t> cmd(jsonStr.begin(), jsonStr.end());
	totalCmd.push_back(cmd);
	return totalCmd;
}

std::vector<std::vector<uint8_t>> soslab::MLA::createRequestMessage(std::string key, const soslab::Request& req)
{
	std::vector<std::vector<uint8_t>> totalCmd;
	std::vector<uint8_t> cmd;

	json_t payload;

	switch (req.feature)
	{
	case Feature::EthernetInfo: payload[key] = "read_ip"; break;
	}

	std::string jsonStr = payload.dump();

	cmd.assign(jsonStr.begin(), jsonStr.end());
	totalCmd.push_back(cmd);
	return totalCmd;
}

bool soslab::MLA::validateEthernetInfoJson(const std::vector<uint8_t>& ack, soslab::MessageBase& out)
{
	soslab::Message::EthernetInfoMessage& msg = static_cast<soslab::Message::EthernetInfoMessage&>(out);

	bool retval = true;
	if (json_t::accept(ack.begin(), ack.end()))
	{
		json_t ackJson = json_t::parse(ack.begin(), ack.end());
		if (ackJson.contains("ml_ip") && ackJson["ml_ip"].is_string()) msg.sensorIp = ackJson["ml_ip"].get<std::string>();
		else retval = false;

		if (ackJson.contains("ml_port") && ackJson["ml_port"].is_number_integer()) msg.sensorPort = ackJson["ml_port"].get<int>();
		else retval = false;

		if (ackJson.contains("pc_ip") && ackJson["pc_ip"].is_string()) msg.pcIp = ackJson["pc_ip"].get<std::string>();
		else retval = false;

		if (ackJson.contains("pc_port") && ackJson["pc_port"].is_number_integer()) msg.pcPort = ackJson["pc_port"].get<int>();
		else retval = false;

		if (ackJson.contains("netmask") && ackJson["netmask"].is_string()) msg.subnetMask = ackJson["netmask"].get<std::string>();
		else retval = false;

		if (ackJson.contains("gateway") && ackJson["gateway"].is_string()) msg.gatewayAddr = ackJson["gateway"].get<std::string>();
		else retval = false;

		if (ackJson.contains("mac") && ackJson["mac"].is_string()) msg.macAddr = ackJson["mac"].get<std::string>();
		else retval = false;
	}
	else retval = false;

	return retval;
}

soslab::packetStatus soslab::MLA::classifyPacket(const std::vector<uint8_t>& pkt) const
{
	if (pkt.size() < 8) return packetStatus::INVALID;

	std::string header(pkt.begin(), pkt.begin() + 8);
	auto a = header.substr(2, 2);
	if (memcmp(header.c_str(), "LIDARPK", 7) == 0) return packetStatus::STREAM;
	else if (json_t::accept(pkt.begin(), pkt.end())) return packetStatus::RESPONSE;

	return packetStatus::UNKNOWN;
}

bool soslab::MLA::parseStreamData(const std::vector<uint8_t>& packetData)
{
	return buildStreamData(packetData);
}

bool soslab::MLA::buildStreamData(const std::vector<uint8_t>& packetData)
{
	if (packetData.size() < sizeof(header::headerMLX))
	{
		std::cerr << "[Error] ML Frame data size is too small: " << packetData.size() << std::endl;
		return false;
	}

	header::headerMLX header;
	memcpy(&header, packetData.data(), sizeof(header::headerMLX));

	int numbering = header.header[7] - '0';
	FrameData& frameData = frameDataVec[numbering];
	frameData.lidarId = numbering;

	uint8_t row_number = header.row_number;
	const uint8_t* dataPtr = packetData.data() + sizeof(header);

	frameData.timestamp[row_number] = header.timestamp;

	if(row_number == 0) frameData.status = 0;
	frameData.status |= header.status;
		
	size_t row_start_ambient = row_number * 576;
	size_t row_start_data = row_number * 192;

	memcpy(&frameData.ambient[row_start_ambient], dataPtr, 576 * sizeof(uint32_t));
	dataPtr += 576 * sizeof(uint32_t);

	for (size_t i = 0; i < 192; i++)
	{
		uint32_t depth = 0;
		memcpy(&depth, dataPtr, sizeof(uint32_t));
		frameData.depth[0][row_start_data + i] = depth;
		dataPtr += sizeof(uint32_t);

		uint16_t intensity = 0;
		memcpy(&intensity, dataPtr, sizeof(uint16_t));
		frameData.intensity[0][row_start_data + i] = intensity;
		dataPtr += sizeof(uint16_t);

		// reserved
		dataPtr += sizeof(uint16_t);

		uint64_t rawData = 0;
		memcpy(&rawData, dataPtr, sizeof(uint64_t));
		dataPtr += sizeof(uint64_t);

		Points point;
		int32_t x_raw = rawData & 0x1FFFFF;
		int32_t x = (x_raw & 0x100000) ? (x_raw | 0xFFE00000) : x_raw;

		int32_t y_raw = (rawData >> 21) & 0x1FFFFF;
		int32_t y = (y_raw & 0x100000) ? (y_raw | 0xFFE00000) : y_raw;

		int32_t z_raw = (rawData >> 42) & 0x1FFFFF;
		int32_t z = (z_raw & 0x100000) ? (z_raw | 0xFFE00000) : z_raw;

		point.x = static_cast<float>(x);
		point.y = static_cast<float>(y);
		point.z = static_cast<float>(z);

		frameData.points[0][row_start_data + i] = point;
	}

	if (row_number == 55)
	{
		frameData.rows = 56;
		frameData.cols = 192;
		frameData.frameNum++;
		if (frameBuffer != nullptr)
		{
			frameBuffer->push(std::make_shared<FrameData>(frameData));
		}
	}

	return true;
}
