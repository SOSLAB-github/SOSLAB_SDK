#include "GL5.h"

#include <fstream>
#include <iomanip>
#include <sstream>
#include <chrono>
# include <math.h>

soslab::GL5::GL5()
{
	std::cout << "GL5 created." << std::endl;
	frameData.rows = 1;
	frameData.cols = 1500;
	frameData.depth[0].resize(1500);
	frameData.intensity[0].resize(1500);
	frameData.points[0].resize(1500);
	angle.resize(1500);
}

soslab::GL5::~GL5()
{
	std::cout << "GL5 deleted." << std::endl;
}

bool soslab::GL5::supports(Feature f) const
{
	switch (f)
	{
	case Feature::Console:
	case Feature::OperationMode:
	case Feature::StreamData:
	case Feature::StreamEnable:
	case Feature::AreaLevelData:
	case Feature::AreaDataFinish:
	case Feature::AreaDataCompare:
	case Feature::SerialNum:
	case Feature::EthernetInfo:
	case Feature::FWVersion:
		return true;
	default:
		std::cerr << "Selected feature is not supported in GL5.\n";
		return false;
	}
}

bool soslab::GL5::buildCommand(const Request& req, soslab::MessageBase& msg, std::vector<std::vector<uint8_t>>& totalProtocol)
{
	std::vector<uint8_t> protocol;
	uint8_t pageIndex = 0;
	uint8_t pageLength = 1;
	std::vector<uint8_t> dtnBytes;

	if (!supports(req.feature))
	{
		std::cerr << "Wrong feature to build GL5 command.\n";
		return false;
	}

	header.bi = header::PayloadBI::PC2DEV;
	header.sm = (req.mode == SetMode::Set) ? header::PayloadSM::SM_SET : header::PayloadSM::SM_GET;

	switch (req.feature)
	{
	case Feature::Console:											header.cat = static_cast<uint16_t>(soslab::GL5::Command::Console); break;
	case Feature::OperationMode:									header.cat = static_cast<uint16_t>(soslab::GL5::Command::OperationMode); dtnBytes = BoolMsg2Bytes(msg); break;
	case Feature::StreamEnable:										header.cat = static_cast<uint16_t>(soslab::GL5::Command::StreamEnable); dtnBytes = BoolMsg2Bytes(msg); break;
	case Feature::AreaLevelData:									header.cat = static_cast<uint16_t>(soslab::GL5::Command::AreaLevelData); dtnBytes = req.mode == SetMode::Set ? AreaInfo2Bytes(msg) : UInt8Msg2Bytes(msg); break;
	case Feature::AreaDataFinish:									header.cat = static_cast<uint16_t>(soslab::GL5::Command::AreaDataFinish); dtnBytes = BoolMsg2Bytes(msg); break;
	case Feature::SerialNum:										header.cat = static_cast<uint16_t>(soslab::GL5::Command::SerialNum); dtnBytes = req.mode == SetMode::Set ? StringMsg2Bytes(msg) : BoolMsg2Bytes(msg); break;
	case Feature::EthernetInfo:										header.cat = static_cast<uint16_t>(soslab::GL5::Command::EthernetInfo); dtnBytes = req.mode == SetMode::Set ? EthernetInfo2Bytes(msg) : BoolMsg2Bytes(msg); break;
	case Feature::FWVersion:										header.cat = static_cast<uint16_t>(soslab::GL5::Command::FWVersion); dtnBytes = req.mode == SetMode::Set ? StringMsg2Bytes(msg) : BoolMsg2Bytes(msg); break;
	default:
		std::cerr << " [ERROR] Feature is not supported in GL5 buildCommand.\n";
		return false;
	}

	// Calculate total pages needed based on dtnBytes size
	pageLength = static_cast<uint16_t>(ceil(static_cast<double>(dtnBytes.size()) / static_cast<double>(commandDtnMax)));

	// Build protocol for each page
	for (uint16_t currentPage = 0; currentPage < pageLength; currentPage++)
	{
		protocol.clear();
		uint8_t cs = 0x00;

		// Calculate actual DTn size for this page
		size_t startIdx = currentPage * commandDtnMax;
		size_t endIdx = std::min((currentPage + 1) * commandDtnMax, dtnBytes.size());
		size_t currentPageDtnSize = endIdx - startIdx;

		header.total_length = static_cast<uint16_t>(currentPageDtnSize + 14);

		// build protocol
		protocol.push_back(header.ps0); //PS1
		protocol.push_back(header.ps1); //PS2
		protocol.push_back(header.ps2); //PS3
		protocol.push_back(header.ps3); //PS4

		// tl (total length)
		protocol.push_back(header.total_length & 0xFF); //TL_L
		protocol.push_back((header.total_length >> 8) & 0xFF); //TL_H

		// pi (page index), pl (page length)
		pageIndex = currentPage;
		protocol.push_back(static_cast<uint8_t>(pageIndex)); //PI
		protocol.push_back(static_cast<uint8_t>(pageLength)); //PL

		// sm, bi
		protocol.push_back(static_cast<uint8_t>(header.sm)); //SM
		protocol.push_back(static_cast<uint8_t>(header.bi)); //BI

		// cat0, cat1
		protocol.push_back((header.cat >> 8) & 0xFF); //CAT0
		protocol.push_back(header.cat & 0xFF); //CAT1

		// Add DTn bytes for this page
		for (size_t i = startIdx; i < endIdx; i++)
		{
			protocol.push_back(dtnBytes[i]); //DTn
		}

		protocol.push_back(0xC2); //PE

		// Calculate checksum
		for (uint8_t element : protocol)
		{
			cs ^= element;
		}
		protocol.push_back(cs); //CS

		totalProtocol.push_back(protocol);
	}

	return !totalProtocol.empty();
}

bool soslab::GL5::parseCommand(const Request& req, const std::vector<uint8_t>& pkt, soslab::MessageBase& out, bool& isFinish)
{
	bool retval = true;
	Feature feature = req.feature;

	if (pkt.size() < sizeof(header::headerGL5))
	{
		std::cerr << "[Error] GL Frame data size is too small: " << pkt.size() << std::endl;
		return false;
	}

	// protocol header parsing
	memcpy(&header, pkt.data(), sizeof(header::headerGL5));
	header.cat = (header.cat >> 8) | (header.cat << 8);

	if (header.ps0 != 0xC3 || header.ps1 != 0x51 || header.ps2 != 0xA1 || header.ps3 != 0xF8)
	{
		std::cerr << "[Error] Invalid GL header" << std::endl;
		return false;
	}

	if (header.bi != header::PayloadBI::DEV2PC)
	{
		std::cerr << "[Error] Invalid Board ID" << std::endl;
		return false;
	}

	if (catToFeature(header.cat) != feature)
	{
		std::cerr << "[Error] Invalid feature.\n";
		return false;
	}

	uint16_t totalLength = header.total_length;
	const uint8_t* dataPtr = pkt.data() + sizeof(header::headerGL5);
	const size_t dataSize = totalLength >= 15 ? totalLength - sizeof(header::headerGL5) - 2 : 0;

	if (dataSize == 0)
	{
		std::cerr << "[Error] Invalid data size" << std::endl;
		return false;
	}

	// Verify packet integrity: PE and CS
	if (pkt.size() < sizeof(header::headerGL5) + 2)
	{
		std::cerr << "[Error] Packet too small for PE and CS" << std::endl;
		return false;
	}

	// Check PE (Packet End) at second-to-last byte
	uint8_t pe = pkt[pkt.size() - 2];
	if (pe != 0xC2)
	{
		std::cerr << "[Error] Invalid PE. Expected 0xC2, got 0x" << std::hex
			<< static_cast<int>(pe) << std::dec << std::endl;
		return false;
	}

	// Check CS (Checksum) at last byte - XOR of all bytes except CS itself
	uint8_t calculatedCS = 0x00;
	for (size_t i = 0; i < pkt.size() - 1; i++)
	{
		calculatedCS ^= pkt[i];
	}
	uint8_t receivedCS = pkt[pkt.size() - 1];
	if (calculatedCS != receivedCS)
	{
		std::cerr << "[Error] CS mismatch. Calculated=0x" << std::hex
			<< static_cast<int>(calculatedCS) << ", Received=0x"
			<< static_cast<int>(receivedCS) << std::dec << std::endl;
		return false;
	}

	// Handle multi-page assembly if pageLength > 1
	if (header.pageLength > 1)
	{
		// Multi-page command response
		if (header.pageIdx == 0)
		{
			// First page: reset accumulator
			cmdPayloadAccum_.clear();
			cmdAssembling = true;
			cmdPrevPageIdx = 0;
		}
		else
		{
			// Subsequent page: validate sequence
			if (!cmdAssembling)
			{
				std::cerr << "[Error] Received non-zero command page while not assembling" << std::endl;
				cmdPayloadAccum_.clear();
				cmdPrevPageIdx = -1;
				return false;
			}
			if (header.pageIdx != cmdPrevPageIdx + 1)
			{
				std::cerr << "[Error] Command page sequence error. Expected="
					<< (cmdPrevPageIdx + 1) << ", got=" << static_cast<int>(header.pageIdx) << std::endl;
				cmdPayloadAccum_.clear();
				cmdPrevPageIdx = -1;
				cmdAssembling = false;
				return false;
			}
			cmdPrevPageIdx = static_cast<int>(header.pageIdx);
		}

		// Accumulate DTn (payload)
		cmdPayloadAccum_.insert(cmdPayloadAccum_.end(), dataPtr, dataPtr + dataSize);

		// If not last page, wait for more
		if (header.pageIdx != (header.pageLength - 1))
		{
			isFinish = false;
			return true;
		}

		// Reset assembly state
		cmdPayloadAccum_.clear();
		cmdPrevPageIdx = -1;
		cmdAssembling = false;
		isFinish = true;

	}
	else
	{
		// Single-page command response
		cmdPayloadAccum_.assign(dataPtr, dataPtr + dataSize);

		isFinish = true;
	}

	// Common parsing logic for both multi-page and single-page
	Feature receivedFeature = catToFeature(header.cat);
	retval = parseCommandData(req, cmdPayloadAccum_, out);

	// Clear payload after processing
	cmdPayloadAccum_.clear();

	return retval;
}

soslab::packetStatus soslab::GL5::classifyPacket(const std::vector<uint8_t>& pkt) const
{
	if (pkt.size() < sizeof(header::headerGL5)) return packetStatus::INVALID;

	header::headerGL5 h{};
	std::memcpy(&h, pkt.data(), sizeof(header::headerGL5));
	h.cat = (h.cat >> 8) | (h.cat << 8);

	if (h.ps0 != 0xC3 || h.ps1 != 0x51 || h.ps2 != 0xA1 || h.ps3 != 0xF8)
		return packetStatus::INVALID;

	if (h.cat == 0x0102) return packetStatus::STREAM;

	return packetStatus::RESPONSE;
}

bool soslab::GL5::parseStreamData(const std::vector<uint8_t>& packetData)
{
	bool retval = true;

	if (packetData.size() < sizeof(header::headerGL5))
	{
		std::cerr << "[Error] GL Frame data size is too small: " << packetData.size() << std::endl;
		return false;
	}

	// protocol header parsing
	memcpy(&header, packetData.data(), sizeof(header::headerGL5));
	header.cat = (header.cat >> 8) | (header.cat << 8);

	if (header.ps0 != 0xC3 || header.ps1 != 0x51 || header.ps2 != 0xA1 || header.ps3 != 0xF8)
	{
		std::cerr << "[Error] Invalid GL header" << std::endl;
		return false;
	}

	if (header.bi != header::PayloadBI::DEV2PC)
	{
		std::cerr << "[Error] Invalid Board ID" << std::endl;
		return false;
	}

	if (header.cat != 0x0102)
	{
		std::cerr << "[Error] Invalid Category ID" << std::endl;
		return false;
	}

	if (header.pageLength != kPageLengthFixed)
	{
		std::cerr << "[Error] Invalid pageLength. Expected=" << static_cast<int>(kPageLengthFixed)
			<< ", got=" << static_cast<int>(header.pageLength) << std::endl;
		assemFrame = false;
		payloadAccum_.clear();
		prePageIdx = -1;
		return false;
	}
	if (header.pageIdx >= kPageLengthFixed)
	{
		std::cerr << "[Error] Invalid PageIdx. Expected=" << static_cast<int>(header.pageIdx)
			<< ", got=" << static_cast<int>(header.pageLength) << std::endl;
		assemFrame = false;
		payloadAccum_.clear();
		prePageIdx = -1;
		return false;
	}

	uint16_t totalLength = header.total_length;
	const uint8_t* dataPtr = packetData.data() + sizeof(header::headerGL5);
	const size_t dataSize = totalLength >= 15 ? totalLength - sizeof(header::headerGL5) - 2 : 0;

	if (dataSize == 0)
	{
		std::cerr << "[Error] Invalid data size" << std::endl;
		return false;
	}

	// Verify packet integrity: PE and CS
	if (packetData.size() < sizeof(header::headerGL5) + 2)
	{
		std::cerr << "[Error] Packet too small for PE and CS" << std::endl;
		return false;
	}

	// Check PE (Packet End) at second-to-last byte
	uint8_t pe = packetData[packetData.size() - 2];
	if (pe != 0xC2)
	{
		std::cerr << "[Error] Invalid PE. Expected 0xC2, got 0x" << std::hex
			<< static_cast<int>(pe) << std::dec << std::endl;
		return false;
	}

	// Check CS (Checksum) at last byte - XOR of all bytes except CS itself
	uint8_t calculatedCS = 0x00;
	for (size_t i = 0; i < packetData.size() - 1; i++)
	{
		calculatedCS ^= packetData[i];
	}
	uint8_t receivedCS = packetData[packetData.size() - 1];
	if (calculatedCS != receivedCS)
	{
		std::cerr << "[Error] CS mismatch. Calculated=0x" << std::hex
			<< static_cast<int>(calculatedCS) << ", Received=0x"
			<< static_cast<int>(receivedCS) << std::dec << std::endl;
		return false;
	}

	if (header.pageIdx == 0)
	{
		payloadAccum_.clear();
		assemFrame = true;
		prePageIdx = 0;
	}
	else
	{
		if (!assemFrame)
		{
			std::cerr << "[Error] Received non-zero page while frame not assembling." << std::endl;
			payloadAccum_.clear();
			prePageIdx = -1;
			return false;
		}

		// Check page sequence continuity
		if (header.pageIdx != (prePageIdx + 1))
		{
			std::cerr << "[Error] Page sequence broken. Expected pageIdx=" << (prePageIdx + 1)
				<< ", got=" << static_cast<int>(header.pageIdx) << std::endl;
			assemFrame = false;
			payloadAccum_.clear();
			prePageIdx = -1;
			return false;
		}

		prePageIdx = static_cast<int>(header.pageIdx);
	}

	payloadAccum_.insert(payloadAccum_.end(), dataPtr, dataPtr + dataSize);

	if (header.pageIdx != (kPageLengthFixed - 1))
	{
		return true;
	}

	constexpr size_t kMetaSize = 22;
	if (payloadAccum_.size() < 2 + kMetaSize)
	{
		std::cerr << "[Error] Accumulated payload too small: " << payloadAccum_.size() << std::endl;
		assemFrame = false;
		payloadAccum_.clear();
		prePageIdx = -1;
		return false;
	}

	numAllPoints = static_cast<uint16_t>(payloadAccum_[0] | (payloadAccum_[1] << 8));

	const size_t pointBytes = payloadAccum_.size() - kMetaSize;
	if ((pointBytes % 4) != 0)
	{
		std::cerr << "[Error] Invalid pointBytes alignment. pointBytes=" << pointBytes << std::endl;
		assemFrame = false;
		payloadAccum_.clear();
		prePageIdx = -1;
		return false;
	}

	const size_t totalPoints = pointBytes / 4;

	frameData.depth[0].resize(numAllPoints);
	frameData.intensity[0].resize(numAllPoints);
	frameData.points[0].resize(numAllPoints);
	angle.resize(numAllPoints);

	Points point;
	const size_t ptsToFill = (totalPoints < numAllPoints) ? totalPoints : numAllPoints;

	for (size_t i = 0; i < ptsToFill; ++i)
	{
		const size_t offset = 2 + i * 4;
		uint16_t distance = static_cast<uint16_t>(payloadAccum_[offset + 0] | (payloadAccum_[offset + 1] << 8));
		uint16_t pulsewidth = static_cast<uint16_t>(payloadAccum_[offset + 2] | (payloadAccum_[offset + 3] << 8));

		if (distance > 60000) distance = 0;

		frameData.depth[0][i] = distance;
		frameData.intensity[0][i] = pulsewidth;

		if (numAllPoints > 1)
		{
			angle[i] = static_cast<double>(i) * h_fov_ / static_cast<double>(numAllPoints - 1) * 3.141592 / 180.0
				- (h_fov_ / 2.0 - 90.0) * 3.141592 / 180.0;
		}
		else
		{
			angle[i] = 0.0;
		}

		point.x = frameData.depth[0][i] * cos(angle[i]);
		point.y = frameData.depth[0][i] * sin(angle[i]);
		point.z = 0.0;

		frameData.points[0][i] = point;
	}

	accumCnt = ptsToFill;

	if (frameBuffer != nullptr)
	{
		frameBuffer->push(std::make_shared<FrameData>(frameData));
	}

	assemFrame = false;
	payloadAccum_.clear();
	prePageIdx = -1;

	return true;
}

bool soslab::GL5::parseCommandData(const Request& req, const std::vector<uint8_t>& dtn, soslab::MessageBase& out)
{
	bool retval = true;
	Feature feature = req.feature;

	// parse command data
	switch (feature)
	{
	case Feature::Console:			return parseBoolAck(dtn, out);
	case Feature::OperationMode:	return parseUInt8Ack(dtn, out);
	case Feature::StreamEnable:			return parseBoolAck(dtn, out);
	case Feature::AreaLevelData:			return req.mode == SetMode::Set ? parseBoolAck(dtn, out) : parseAreaInfoAck(dtn, out);
	case Feature::AreaDataFinish:			return parseBoolAck(dtn, out);
	case Feature::SerialNum:			return parseStringAck(dtn, out);
	case Feature::EthernetInfo:			return req.mode == SetMode::Set ? parseBoolAck(dtn, out) : parseEthernetInfoAck(dtn, out);
	case Feature::FWVersion:			return parseStringAck(dtn, out);
	default:
		std::cerr << "Selected feature is not supported in GL5.\n";
		return false;
	}

	return retval;
}

bool soslab::GL5::parseAreaInfoAck(const std::vector<uint8_t>& ack, soslab::MessageBase& out)
{
	soslab::Message::GL5::AreaDataMessage* areaDataMsg = static_cast<soslab::Message::GL5::AreaDataMessage*>(&out);
	if (areaDataMsg == nullptr)
	{
		return false;
	}
	size_t idx = 0;

	areaDataMsg->areaNum = ack[idx++];
	areaDataMsg->area = std::vector<soslab::region_info_t>();

	uint8_t recvRegionSize = ack[idx++];

	for (size_t i = 0; i < recvRegionSize; i++)
	{
		soslab::region_info_t region;
		uint8_t regionNum = ack[idx++];
		region.region_type = ack[idx++];
		uint8_t recvCoordSize = ack[idx++];

		for (size_t j = 0; j < recvCoordSize; j++)
		{
			int32_t coord_x = ack[idx++];
			coord_x |= (ack[idx++] << 8);
			coord_x |= (ack[idx++] << 16);
			coord_x |= (ack[idx++] << 24);

			int32_t coord_y = ack[idx++];
			coord_y |= (ack[idx++] << 8);
			coord_y |= (ack[idx++] << 16);
			coord_y |= (ack[idx++] << 24);

			region.coordsX.push_back(coord_x / 1000.0);
			region.coordsY.push_back(coord_y / 1000.0);
		}

		region.level_type = ack[idx++];
		uint8_t recvLevelSize = ack[idx++];
		for (size_t j = 0; j < recvLevelSize; j++)
		{
			int32_t level = ack[idx++];
			level |= (ack[idx++] << 8);
			level |= (ack[idx++] << 16);
			level |= (ack[idx++] << 24);

			region.levels.push_back(level / 1000.0);
		}

		region.hysteresis = ack[idx++];

		bool region_maximum_value = region.region_type > 1;
		bool level_maximum_value = (recvRegionSize > 1) ? false : region.level_type > 2;
		bool hysteresis_maximum_value = region.hysteresis > 2;

		if (region_maximum_value || level_maximum_value || hysteresis_maximum_value)
		{
			return false;
		}

		areaDataMsg->area.push_back(region);
	}

	return true;
}

bool soslab::GL5::parseEthernetInfoAck(const std::vector<uint8_t>& ack, soslab::MessageBase& out)
{
	soslab::Message::GL5::EthernetInfoMessage* ethernetInfoMsg = static_cast<soslab::Message::GL5::EthernetInfoMessage*>(&out);
	if (ethernetInfoMsg == nullptr)
	{
		return false;
	}

	std::ostringstream ossPcIp;
	ossPcIp << (int)(ack[0]) << "." << (int)(ack[1]) << "." << (int)(ack[2]) << "." << (int)(ack[3]);
	std::string pcIp = ossPcIp.str();

	std::ostringstream ossSensorIp;
	ossSensorIp << (int)(ack[4]) << "." << (int)(ack[5]) << "." << (int)(ack[6]) << "." << (int)(ack[7]);
	std::string sensorIp = ossSensorIp.str();

	std::ostringstream ossSubnetMask;
	ossSubnetMask << (int)(ack[8]) << "." << (int)(ack[9]) << "." << (int)(ack[10]) << "." << (int)(ack[11]);
	std::string subnetMask = ossSubnetMask.str();

	std::ostringstream ossGatewayAddr;
	ossGatewayAddr << (int)(ack[12]) << "." << (int)(ack[13]) << "." << (int)(ack[14]) << "." << (int)(ack[15]);
	std::string gatewayAddr = ossGatewayAddr.str();

	std::ostringstream ossMacAddr;
	ossMacAddr << std::hex << std::uppercase << std::setfill('0') << std::setw(2) << (int)(ack[16]) << "-" << std::setw(2) << (int)(ack[17]) << "-" << std::setw(2) << (int)(ack[18]) << "-" << std::setw(2) << (int)(ack[19]) << "-" << std::setw(2) << (int)(ack[20]) << "-" << std::setw(2) << (int)(ack[21]);
	std::string macAddr = ossMacAddr.str();

	uint16_t pcPort = (uint16_t)(ack[22] & 0xff);
	pcPort |= (uint16_t)(ack[23] & 0xff) << 8;

	uint16_t sensorPort = (uint16_t)(ack[24] & 0xff);
	sensorPort |= (uint16_t)(ack[25] & 0xff) << 8;

	ethernetInfoMsg->sensorIp = sensorIp;
	ethernetInfoMsg->pcIp = pcIp;
	ethernetInfoMsg->subnetMask = subnetMask;
	ethernetInfoMsg->gatewayAddr = gatewayAddr;
	ethernetInfoMsg->macAddr = macAddr;
	ethernetInfoMsg->pcPort = pcPort;
	ethernetInfoMsg->sensorPort = sensorPort;

	return true;
}

bool soslab::GL5::parseFWVersionAck(const std::vector<uint8_t>& ack, soslab::MessageBase& out)
{
	soslab::Message::GL5::FWVersionMessage* fwVersionMsg = static_cast<soslab::Message::GL5::FWVersionMessage*>(&out);
	if (fwVersionMsg == nullptr)
	{
		return false;
	}
	fwVersionMsg->year = ack[0];
	fwVersionMsg->month = ack[1];
	fwVersionMsg->day = ack[2];

	return true;
}

bool soslab::GL5::parseBoolAck(const std::vector<uint8_t>& ack, soslab::MessageBase& out)
{
	if (ack.empty()) return false;

	return ack[0] == 0x01;
}

bool soslab::GL5::parseUInt8Ack(const std::vector<uint8_t>& ack, soslab::MessageBase& out)
{
	soslab::Message::GeneralMessage<uint8_t>* uint8Msg = static_cast<soslab::Message::GeneralMessage<uint8_t>*>(&out);
	if (uint8Msg == nullptr)
	{
		return false;
	}
	uint8Msg->data = ack[0];

	return true;
}

bool soslab::GL5::parseUInt16Ack(const std::vector<uint8_t>& ack, soslab::MessageBase& out)
{
	soslab::Message::GeneralMessage<uint16_t>* uint16Msg = static_cast<soslab::Message::GeneralMessage<uint16_t>*>(&out);
	if (uint16Msg == nullptr)
	{
		return false;
	}
	uint16Msg->data = ack[0] | (static_cast<uint16_t>(ack[1] << 8));

	return true;
}

bool soslab::GL5::parseUInt32Ack(const std::vector<uint8_t>& ack, soslab::MessageBase& out)
{
	soslab::Message::GeneralMessage<uint32_t>* uint32Msg = static_cast<soslab::Message::GeneralMessage<uint32_t>*>(&out);
	if (uint32Msg == nullptr)
	{
		return false;
	}
	uint32Msg->data = ack[0] | (static_cast<uint32_t>(ack[1] << 8)) | (static_cast<uint32_t>(ack[2] << 16)) | (static_cast<uint32_t>(ack[3] << 24));

	return true;
}

bool soslab::GL5::parseStringAck(const std::vector<uint8_t>& ack, soslab::MessageBase& out)
{
	soslab::Message::GeneralMessage<std::string>* stringMsg = static_cast<soslab::Message::GeneralMessage<std::string>*>(&out);
	if (stringMsg == nullptr)
	{
		return false;
	}
	stringMsg->data = std::string(ack.begin(), ack.end());

	return true;
}

bool soslab::GL5::parseDoubleAck(const std::vector<uint8_t>& ack, soslab::MessageBase& out)
{
	soslab::Message::GeneralMessage<double>* doubleMsg = static_cast<soslab::Message::GeneralMessage<double>*>(&out);
	if (doubleMsg == nullptr)
	{
		return false;
	}
	doubleMsg->data = static_cast<double>(ack[0] | (static_cast<uint32_t>(ack[1] << 8)) | (static_cast<uint32_t>(ack[2] << 16)) | (static_cast<uint32_t>(ack[3] << 24)));

	return true;
}

bool soslab::GL5::parseFloatAck(const std::vector<uint8_t>& ack, soslab::MessageBase& out)
{
	soslab::Message::GeneralMessage<float> floatMsg;
	floatMsg.data = static_cast<float>(ack[0] | (static_cast<uint32_t>(ack[1] << 8)) | (static_cast<uint32_t>(ack[2] << 16)) | (static_cast<uint32_t>(ack[3] << 24)));
	out = floatMsg;

	return true;
}

std::vector<uint8_t> soslab::GL5::BoolMsg2Bytes(const soslab::MessageBase& msg)
{
	std::vector<uint8_t> bytes;

	soslab::Message::GeneralMessage<bool> boolMsg = static_cast<const soslab::Message::GeneralMessage<bool>&>(msg);
	bytes = boolMsg.data ? std::vector<uint8_t>({ 0x01 }) : std::vector<uint8_t>({ 0x00 });

	return bytes;
}

std::vector<uint8_t> soslab::GL5::UInt8Msg2Bytes(const soslab::MessageBase& msg)
{
	std::vector<uint8_t> bytes;

	soslab::Message::GeneralMessage<uint8_t> uint8Msg = static_cast<const soslab::Message::GeneralMessage<uint8_t>&>(msg);
	bytes = std::vector<uint8_t>({ uint8Msg.data });

	return bytes;
}

std::vector<uint8_t> soslab::GL5::UInt16Msg2Bytes(const soslab::MessageBase& msg)
{
	std::vector<uint8_t> bytes;

	soslab::Message::GeneralMessage<uint16_t> uint16Msg = static_cast<const soslab::Message::GeneralMessage<uint16_t>&>(msg);
	bytes.push_back(static_cast<uint8_t>(uint16Msg.data & 0xFF));
	bytes.push_back(static_cast<uint8_t>((uint16Msg.data >> 8) & 0xFF));

	return bytes;
}

std::vector<uint8_t> soslab::GL5::UInt32Msg2Bytes(const soslab::MessageBase& msg)
{
	std::vector<uint8_t> bytes;

	soslab::Message::GeneralMessage<uint32_t> uint32Msg = static_cast<const soslab::Message::GeneralMessage<uint32_t>&>(msg);
	bytes.push_back(static_cast<uint8_t>(uint32Msg.data & 0xFF));
	bytes.push_back(static_cast<uint8_t>((uint32Msg.data >> 8) & 0xFF));
	bytes.push_back(static_cast<uint8_t>((uint32Msg.data >> 16) & 0xFF));
	bytes.push_back(static_cast<uint8_t>((uint32Msg.data >> 24) & 0xFF));

	return bytes;
}

std::vector<uint8_t> soslab::GL5::StringMsg2Bytes(const soslab::MessageBase& msg)
{
	std::vector<uint8_t> bytes;

	soslab::Message::GeneralMessage<std::string> stringMsg = static_cast<const soslab::Message::GeneralMessage<std::string>&>(msg);
	bytes.insert(bytes.end(), stringMsg.data.begin(), stringMsg.data.end());

	return bytes;
}

std::vector<uint8_t> soslab::GL5::DoubleMsg2Bytes(const soslab::MessageBase& msg)
{
	std::vector<uint8_t> bytes;

	soslab::Message::GeneralMessage<double> doubleMsg = static_cast<const soslab::Message::GeneralMessage<double>&>(msg);
	uint32_t bits = 0;
	std::memcpy(&bits, &doubleMsg.data, sizeof(bits));
	bytes.push_back(static_cast<uint8_t>(bits & 0xFF));
	bytes.push_back(static_cast<uint8_t>((bits >> 8) & 0xFF));
	bytes.push_back(static_cast<uint8_t>((bits >> 16) & 0xFF));
	bytes.push_back(static_cast<uint8_t>((bits >> 24) & 0xFF));

	return bytes;
}

std::vector<uint8_t> soslab::GL5::FloatMsg2Bytes(const soslab::MessageBase& msg)
{
	std::vector<uint8_t> bytes;

	soslab::Message::GeneralMessage<float> floatMsg = static_cast<const soslab::Message::GeneralMessage<float>&>(msg);
	uint32_t bits = 0;
	std::memcpy(&bits, &floatMsg.data, sizeof(bits));
	bytes.push_back(static_cast<uint8_t>(bits & 0xFF));
	bytes.push_back(static_cast<uint8_t>((bits >> 8) & 0xFF));
	bytes.push_back(static_cast<uint8_t>((bits >> 16) & 0xFF));
	bytes.push_back(static_cast<uint8_t>((bits >> 24) & 0xFF));

	return bytes;
}

std::vector<uint8_t> soslab::GL5::EthernetInfo2Bytes(const soslab::MessageBase& msg)
{
	std::vector<uint8_t> bytes;

	soslab::Message::GL5::EthernetInfoMessage ethernetInfoMsg = static_cast<const soslab::Message::GL5::EthernetInfoMessage&>(msg);

	std::istringstream issPcIp(ethernetInfoMsg.pcIp);
	std::string octet;
	while (std::getline(issPcIp, octet, '.'))
	{
		bytes.push_back(static_cast<uint8_t>(std::stoi(octet)));
	}

	std::istringstream issSensorIp(ethernetInfoMsg.sensorIp);
	while (std::getline(issSensorIp, octet, '.'))
	{
		bytes.push_back(static_cast<uint8_t>(std::stoi(octet)));
	}

	std::istringstream issSubnetMask(ethernetInfoMsg.subnetMask);
	while (std::getline(issSubnetMask, octet, '.'))
	{
		bytes.push_back(static_cast<uint8_t>(std::stoi(octet)));
	}

	std::istringstream issGatewayAddr(ethernetInfoMsg.gatewayAddr);
	while (std::getline(issGatewayAddr, octet, '.'))
	{
		bytes.push_back(static_cast<uint8_t>(std::stoi(octet)));
	}

	std::istringstream issMacAddr(ethernetInfoMsg.macAddr);
	while (std::getline(issMacAddr, octet, '-'))
	{
		bytes.push_back(static_cast<uint8_t>(std::stoi(octet, nullptr, 16)));
	}

	bytes.push_back(static_cast<uint8_t>(ethernetInfoMsg.pcPort & 0xFF));
	bytes.push_back(static_cast<uint8_t>((ethernetInfoMsg.pcPort >> 8) & 0xFF));

	bytes.push_back(static_cast<uint8_t>(ethernetInfoMsg.sensorPort & 0xFF));
	bytes.push_back(static_cast<uint8_t>((ethernetInfoMsg.sensorPort >> 8) & 0xFF));

	return bytes;
}

std::vector<uint8_t> soslab::GL5::AreaInfo2Bytes(const soslab::MessageBase& msg)
{
	std::vector<uint8_t> bytes;

	const soslab::Message::GL5::AreaDataMessage* areaDataMsg = static_cast<const soslab::Message::GL5::AreaDataMessage*>(&msg);
	if (areaDataMsg == nullptr)
	{
		std::cerr << "[ERROR] Failed to cast to AreaDataMessage" << std::endl;
		return bytes;
	}

	bytes.push_back(static_cast<uint8_t>(areaDataMsg->areaNum));
	bytes.push_back(static_cast<uint8_t>(areaDataMsg->area.size()));

	// For each region
	for (size_t regionNum = 0; regionNum < areaDataMsg->area.size(); regionNum++)
	{
		const auto& region = areaDataMsg->area[regionNum];
		std::vector<double> coordsX = region.coordsX;
		std::vector<double> coordsY = region.coordsY;
		bytes.push_back(static_cast<uint8_t>(regionNum));
		bytes.push_back(static_cast<uint8_t>(region.region_type));

		std::vector<double> outlines;
		if (region.region_type == REGION_TYPE::POLYGON)
			calOutLinePolygon(numAllPoints, h_fov_, coordsX, coordsY, outlines);
		else
			calOutLineArc(numAllPoints, h_fov_, coordsX, coordsY, outlines);

		std::vector<double> sortedLevels = region.levels;
		std::sort(sortedLevels.begin(), sortedLevels.end(), [](double a, double b)
			{
				return a < b;
			});

		for (auto& outline : outlines)
		{
			bytes.push_back(static_cast<uint16_t>(outline * 1000.0) & 0xFF);
			bytes.push_back((static_cast<uint16_t>(outline * 1000.0) >> 8) & 0xFF);
		}

		// coordinateSize
		uint8_t coordSize = static_cast<uint8_t>(std::min(coordsX.size(), coordsY.size()));
		bytes.push_back(coordSize);

		for (size_t coordIdx = 0; coordIdx < coordSize; coordIdx++)
		{
			int32_t x = static_cast<int32_t>(coordsX[coordIdx] * 1000.0);
			bytes.push_back(static_cast<uint8_t>(x & 0xFF));
			bytes.push_back(static_cast<uint8_t>((x >> 8) & 0xFF));
			bytes.push_back(static_cast<uint8_t>((x >> 16) & 0xFF));
			bytes.push_back(static_cast<uint8_t>((x >> 24) & 0xFF));

			int32_t y = static_cast<int32_t>(coordsY[coordIdx] * 1000.0);
			bytes.push_back(static_cast<uint8_t>(y & 0xFF));
			bytes.push_back(static_cast<uint8_t>((y >> 8) & 0xFF));
			bytes.push_back(static_cast<uint8_t>((y >> 16) & 0xFF));
			bytes.push_back(static_cast<uint8_t>((y >> 24) & 0xFF));
		}

		bytes.push_back(static_cast<uint8_t>(region.level_type));
		bytes.push_back(static_cast<uint8_t>(sortedLevels.size()));

		for (size_t levelIdx = 0; levelIdx < sortedLevels.size(); levelIdx++)
		{
			int32_t level = static_cast<int32_t>(sortedLevels[levelIdx] * 1000.0);
			bytes.push_back(static_cast<uint8_t>(level & 0xFF));
			bytes.push_back(static_cast<uint8_t>((level >> 8) & 0xFF));
			bytes.push_back(static_cast<uint8_t>((level >> 16) & 0xFF));
			bytes.push_back(static_cast<uint8_t>((level >> 24) & 0xFF));
		}

		bytes.push_back(static_cast<uint8_t>(region.hysteresis));
	}

	return bytes;
}

void soslab::GL5::calOutLinePolygon(int frameDataSize, double h_fov, std::vector<double>& coordX, std::vector<double>& coordY, std::vector<double>& outlines)
{
	outlines.clear();

	std::vector<double> distances;
	std::vector<double> angles;
	convertCoordinate(coordX, coordY, distances, angles);

	for (size_t i = 0; i < distances.size(); i++)
	{
		coordX[i] = distances[i] * cos(angles[i]);
		coordY[i] = distances[i] * sin(angles[i]);
	}

	if (distances.size() > 2)
	{
		double initAngle = -(h_fov / 2.0 - 90.0) * 3.141592 / 180.0;
		double dAngle = h_fov / (double)frameDataSize * 3.141592 / 180.0;
		for (size_t i = 0; i < frameDataSize; i++)
		{
			double dist = 0.0;
			double ang = i * dAngle + initAngle;
			ang = ang > 3.141592 ? ang - 2 * 3.141592 : ang;

			for (size_t num = 1; num < coordX.size() - 1; num++)
			{
				double p0X = coordX[num];
				double p0Y = coordY[num];
				double p1X = coordX[num + 1];
				double p1Y = coordY[num + 1];

				std::vector<double> xy = calcCrossingPointWithPolygon(p0X, p0Y, p1X, p1Y, ang, 0.0);
				double distVal = std::sqrt(xy[0] * xy[0] + xy[1] * xy[1]);
				if (distVal > 0.0)
				{
					dist = distVal;
					break;
				}
			}
			outlines.push_back(dist);
		}
	}
}

void soslab::GL5::calOutLineArc(int frameDataSize, double h_fov, std::vector<double>& coordX, std::vector<double>& coordY, std::vector<double>& outlines)
{
	outlines.clear();

	std::vector<double> distances;
	std::vector<double> angles;

	for (size_t i = 0; i < coordX.size(); i++)
	{
		double distance = std::sqrt(coordX[i] * coordX[i] + coordY[i] * coordY[i]);

		double angle = std::atan2(coordY[i], coordX[i]) - 3.141592 / 2.0;
		angle = angle < -3.141592 ? angle + 2 * 3.141592 : angle;

		distances.push_back(distance);
		angles.push_back(angle);
	}

	if (distances.size() > 2)
	{
		double initAngle = -h_fov / 2.0 * 3.141592 / 180.0;
		double dAngle = h_fov / (double)frameDataSize * 3.141592 / 180.0;
		for (size_t i = 0; i < frameDataSize; i++)
		{
			double dist = 0.0;
			double ang = i * dAngle + initAngle;

			if (angles[1] < angles[2])
			{
				if (ang > angles[1] && ang < angles[2])
				{
					dist = distances[1];
				}
			}
			else
			{
				if (ang > angles[1] || ang < angles[2])
				{
					dist = distances[1];
				}
			}
			outlines.push_back(dist);
		}
	}
}

void soslab::GL5::convertCoordinate(const std::vector<double>& coordX, const std::vector<double>& coordY, std::vector<double>& distances, std::vector<double>& angles)
{
	if (!coordX.empty() && !coordY.empty())
	{
		for (size_t i = 0; i < coordX.size(); i++)
		{
			double distance = std::sqrt(coordX[i] * coordX[i] + coordY[i] * coordY[i]);
			if (distance != 0)
			{
				double angle = std::atan2(coordY[i], coordX[i]) - 3.141592 / 2.0;
				angle = angle < -3.141592 ? angle + 2 * 3.141592 : angle;
				distances.push_back(distance);
				angles.push_back(angle);
			}
		}

		std::vector<std::tuple<double, double>> tupleDistAngle;
		for (size_t i = 0; i < distances.size(); i++)
			tupleDistAngle.emplace_back(distances[i], angles[i]);

		std::sort(tupleDistAngle.begin(), tupleDistAngle.end(),
			[](const auto& a, const auto& b)
			{
				return std::get<1>(a) < std::get<1>(b);
			});

		for (size_t i = 0; i < tupleDistAngle.size(); i++)
		{
			distances[i] = std::get<0>(tupleDistAngle[i]);
			angles[i] = std::get<1>(tupleDistAngle[i]) + 3.141592 / 2.0;
			angles[i] = angles[i] > 3.141592 ? angles[i] - 2 * 3.141592 : angles[i];
		}

		if (!distances.empty() && distances.front() != 0.0)
		{
			distances.insert(distances.begin(), 0.0);
			angles.insert(angles.begin(), 0.0);
		}
	}
}

std::vector<double> soslab::GL5::calcCrossingPointWithPolygon(double x0, double y0, double x1, double y1, double ang, double yi)
{
	std::vector<double> retXY = { 0.0, 0.0 };

	double l1Slope = std::tan(ang);
	double l1yi = yi;

	double x = 0.0;
	double y = 0.0;

	if (x0 == x1)
	{
		x = x0;
		y = l1Slope * x + l1yi;
	}
	else
	{
		double l0Slope = (y1 - y0) / (x1 - x0);
		double l0yi = -l0Slope * x1 + y1;

		double xNum = (l0yi - l1yi);
		double xDen = (l1Slope - l0Slope);
		if (xDen != 0.0)
			x = xNum / xDen;
		else
			x = 0.0;

		if (l1Slope != 0.0)
		{
			double yNum = l0yi - (l0Slope / l1Slope) * l1yi;
			double yDen = (1.0 - (l0Slope / l1Slope));
			y = yNum / yDen;
		}
		else
			y = yi;

		if (!(x == 0.0 && y == 0.0))
		{
			if (onLine(x0, y0, x1, y1, x, y))
			{
				if (std::abs(ang - std::atan2(y, x)) < 0.00003)
				{
					retXY = { x, y };
				}
			}
		}
	}

	return retXY;
}

bool soslab::GL5::onLine(double x0, double y0, double x1, double y1, double x, double y)
{
	double x_max = std::max(x0, x1);
	double x_min = std::min(x0, x1);
	double y_max = std::max(y0, y1);
	double y_min = std::min(y0, y1);

	return x <= x_max && x >= x_min && y <= y_max && y >= y_min;
}

soslab::Feature soslab::GL5::catToFeature(uint16_t cat) const
{
	switch (cat)
	{
	case static_cast<uint16_t>(Command::Console): return Feature::Console;
	case static_cast<uint16_t>(Command::OperationMode): return Feature::OperationMode;
	case static_cast<uint16_t>(Command::StreamEnable): return Feature::StreamEnable;
	case static_cast<uint16_t>(Command::AreaLevelData): return Feature::AreaLevelData;
	case static_cast<uint16_t>(Command::AreaDataFinish): return Feature::AreaDataFinish;
	case static_cast<uint16_t>(Command::SerialNum): return Feature::SerialNum;
	case static_cast<uint16_t>(Command::EthernetInfo): return Feature::EthernetInfo;
	case static_cast<uint16_t>(Command::FWVersion): return Feature::FWVersion;
	default:
		std::cerr << "[ERROR] Unknown cat value: 0x" << std::hex << cat << std::dec << std::endl;
		return Feature::Unknown;
	}
}