#include "MLX.h"
#include "ProtocolConstants.h"

#include <fstream>
#include <iomanip>
#include <sstream>
#include <chrono>
# include <math.h>

soslab::MLX::MLX()
{
	std::cout << "MLX created." << std::endl;
}

soslab::MLX::~MLX()
{
	std::cout << "MLX deleted." << std::endl;
}

bool soslab::MLX::supports(Feature f) const
{
	switch (f)
	{
	case Feature::StreamEnable:
	case Feature::AreaAlarm:
	case Feature::SetAreaLUT:
	case Feature::SaveAreaLUT:
	case Feature::GetAreaLUT:
	case Feature::SelectArea:
		return true;
	default:
		std::cerr << "Selected feature is not supported in MLX.\n";
		return false;
	}
}

bool soslab::MLX::buildCommand(const Request& req, soslab::MessageBase& msg, std::vector<std::vector<uint8_t>>& totalProtocol)
{
	if (!supports(req.feature))
	{
		std::cerr << "Wrong feature to build MLX command.\n";
		return false;
	}

	switch (req.feature)
	{
	case Feature::StreamEnable:		totalProtocol = createBooleanMessage("command", msg);	break;
		//Area
	case Feature::SetAreaLUT:		totalProtocol = createSetAreaInfo(msg);					break;
	case Feature::GetAreaLUT:		totalProtocol = createGetAreaInfo(msg);					break;
	case Feature::SaveAreaLUT:		totalProtocol = createSaveAreaInfo();					break;
	case Feature::SelectArea:		totalProtocol = createAreaSelection(msg);				break;
	default:						std::cerr << "Wrong feature to build MLX command.\n"; return false;

	}

	return !totalProtocol.empty();
}

bool soslab::MLX::parseCommand(const Request& req, const std::vector<uint8_t>& pkt, soslab::MessageBase& out, bool& isFinish)
{
	bool retval = false;
	Feature feature = req.feature;
	switch (feature)
	{
		// cat
	case Feature::StreamEnable:		retval = validateJsonAck(pkt, out);			break;
		//Area
	case Feature::SetAreaLUT:		retval = validateSetAreaAck(pkt, out);		break;
	case Feature::GetAreaLUT:		retval = validateGetAreaAck(pkt, out);		break;
	case Feature::SaveAreaLUT:		retval = validateSaveAreaInfo(pkt, out);	break;
	case Feature::SelectArea:		retval = validateAreaSelection(pkt, out);	break;
	case Feature::AreaAlarm:		retval = validateAreaAlarm(pkt, out);		break;
	default:						std::cerr << "Wrong feature to build MLX command.\n"; return false;
	}

	return retval;
}

std::vector<std::vector<uint8_t>> soslab::MLX::createBooleanMessage(std::string key, const soslab::MessageBase& dtn)
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

std::vector<std::vector<uint8_t>> soslab::MLX::createSetAreaInfo(const soslab::MessageBase& dtn)
{
	std::vector<std::vector<uint8_t>> totalCmd;
	std::vector<uint8_t> cmd;

	soslab::Message::MLX::AreaMessage msg = static_cast<const soslab::Message::MLX::AreaMessage&>(dtn);

	uint8_t areaIndex = msg.areaIndex;
	soslab::area::Area areaData = msg.area;
	std::vector<uint32_t> minLut = msg.minLut;
	std::vector<uint32_t> maxLut = msg.maxLut;

	soslab::util::Endianness endian = soslab::util::Endianness::Little;

	std::vector<uint8_t> metaPacket(20, 0);
	{
		soslab::protocol::mlx::MLXAreaHeader h{};
		h.mainCmd = 'L';
		h.subCmd = 'S';

		std::memcpy(h.header, "LUTIDX", 6);
		h.header[6] = char('0' + (areaIndex / 10) % 10);
		h.header[7] = char('0' + (areaIndex % 10));

		if (areaData.polygons.size() > 255) throw std::runtime_error("polygon count > 255");
		h.polygonIndex = static_cast<uint8_t>(areaData.polygons.size());

		if (!areaData.polygons.empty())
		{
			if (areaData.polygons[0].pts.size() > 255) throw std::runtime_error("vertex count > 255");
			h.vertexIndex = static_cast<uint8_t>(areaData.polygons[0].pts.size());
		}
		else
		{
			h.vertexIndex = 0;
		}

		h.minimumCount = static_cast<uint16_t>(areaData.detectMinimumCount);


		soslab::util::write_u16(metaPacket, 0, 0, endian);

		metaPacket[2] = uint8_t(h.mainCmd);
		metaPacket[3] = uint8_t(h.subCmd);

		std::memcpy(metaPacket.data() + 4, h.header, 8);

		soslab::util::write_u16(metaPacket, 12, 0, endian);
		soslab::util::write_u16(metaPacket, 14, 0, endian);

		soslab::util::write_u16(metaPacket, 16, h.minimumCount, endian);

		metaPacket[18] = h.polygonIndex;
		metaPacket[19] = h.vertexIndex;
	}

	std::vector<uint8_t> payloadPacket(soslab::protocol::mlx::kLutBytes, 0);;
	{

		const size_t expected = static_cast<size_t>(soslab::protocol::mlx::kRows) * static_cast<size_t>(soslab::protocol::mlx::kCols);
		if (minLut.size() != expected || maxLut.size() != expected)
			throw std::invalid_argument("minLut/maxLut must be 56*192 elements (row-major)");

		size_t vOff = 0;
		for (size_t pi = 0; pi < soslab::protocol::mlx::kMaxPolygons; ++pi)
		{
			const uint8_t polyCount = static_cast<uint8_t>(std::min<size_t>(areaData.polygons.size(), soslab::protocol::mlx::kMaxPolygons));
			const bool polyValid = (pi < polyCount);
			const auto& poly = polyValid ? areaData.polygons[pi] : soslab::area::Polygon{};
			const size_t vCount = polyValid ? std::min<size_t>(poly.pts.size(), soslab::protocol::mlx::kMaxVertices) : 0;

			for (size_t vi = 0; vi < soslab::protocol::mlx::kMaxVertices; ++vi)
			{
				if (!polyValid || vi >= vCount)
				{
					std::fill(payloadPacket.begin() + vOff, payloadPacket.begin() + vOff + soslab::protocol::mlx::kVertexBytes, 0xFF);
				}
				else
				{
					const auto& p = poly.pts[vi];
					const int32_t x = static_cast<int32_t>(p.x);
					const int32_t y = static_cast<int32_t>(p.y);
					const int16_t z = static_cast<int16_t>(p.z);

					soslab::util::write_i24(payloadPacket, vOff + 0, x, endian);
					soslab::util::write_i24(payloadPacket, vOff + 3, y, endian);
					soslab::util::write_i16(payloadPacket, vOff + 6, z, endian);

				}
				vOff += soslab::protocol::mlx::kVertexBytes;
			}
		}

		for (size_t i = 0; i < expected; ++i)
		{
			soslab::util::write_u32(payloadPacket, vOff + 0, minLut[i], endian);
			soslab::util::write_u32(payloadPacket, vOff + 4, maxLut[i], endian);
			vOff += 8;
		}
	}

	uint16_t totalPage = static_cast<uint16_t>((payloadPacket.size() + soslab::protocol::mlx::kLutPageBytes - 1) / soslab::protocol::mlx::kLutPageBytes);

	const size_t frameBytes = metaPacket.size() + soslab::protocol::mlx::kLutPageBytes;


	for (uint16_t currentPage = 0; currentPage < totalPage; currentPage++)
	{
		cmd = std::vector<uint8_t>(soslab::protocol::mlx::kLutPageBytes, 0);
		{
			cmd = metaPacket;

			soslab::util::write_u16(cmd, 0, static_cast<uint16_t>(frameBytes - 2), endian);
			soslab::util::write_u16(cmd, 12, totalPage, endian);
			soslab::util::write_u16(cmd, 14, currentPage, endian);
		}

		{
			const size_t off = static_cast<size_t>(currentPage) * soslab::protocol::mlx::kLutPageBytes;
			const size_t remain = (off < payloadPacket.size()) ? (payloadPacket.size() - off) : 0;
			const size_t chunkLen = std::min(remain, soslab::protocol::mlx::kLutPageBytes);

			if (chunkLen > 0)
				cmd.insert(cmd.end(), payloadPacket.begin() + off, payloadPacket.begin() + off + chunkLen);

			if (chunkLen < soslab::protocol::mlx::kLutPageBytes)
				cmd.insert(cmd.end(), soslab::protocol::mlx::kLutPageBytes - chunkLen, 0x00); // padding

		}

		totalCmd.push_back(cmd);
	}

	return totalCmd;
}

std::vector<std::vector<uint8_t>> soslab::MLX::createGetAreaInfo(const soslab::MessageBase& dtn)
{
	std::vector<std::vector<uint8_t>> totalCmd;
	std::vector<uint8_t> cmd;

	soslab::Message::GeneralMessage<uint8_t> msg = static_cast<const soslab::Message::GeneralMessage<uint8_t>&>(dtn);
	std::vector<uint8_t> sendPacket(soslab::protocol::mlx::kLutPageBytes, 0);

	uint8_t areaIndex = msg.data;
	soslab::util::Endianness endian = soslab::util::Endianness::Little;

	const size_t kFrameBytes = 12;
	soslab::protocol::mlx::MLXAreaHeader h{};
	h.dataLength = kFrameBytes - 4;
	h.mainCmd = 'I';
	h.subCmd = 'G';

	std::memcpy(h.header, "LUTIDX", 6);
	h.header[6] = char('0' + (areaIndex / 10) % 10);
	h.header[7] = char('0' + (areaIndex % 10));

	cmd = std::vector<uint8_t>(kFrameBytes, 0);

	soslab::util::write_u16(cmd, 0, h.dataLength, endian);

	cmd[2] = uint8_t(h.mainCmd);
	cmd[3] = uint8_t(h.subCmd);

	std::memcpy(cmd.data() + 4, h.header, 8);

	totalCmd.push_back(cmd);
	return totalCmd;
}

std::vector<std::vector<uint8_t>> soslab::MLX::createSaveAreaInfo()
{
	std::vector<std::vector<uint8_t>> totalCmd;
	std::vector<uint8_t> cmd;

	const size_t kFrameBytes = 12;
	soslab::protocol::mlx::MLXAreaHeader h{};
	h.dataLength = kFrameBytes - 4;
	h.mainCmd = 'A';
	h.subCmd = 'W';

	std::memcpy(h.header, "AREASAVE", 8);

	cmd = std::vector<uint8_t>(kFrameBytes, 0);

	soslab::util::write_u16(cmd, 0, h.dataLength, soslab::util::Endianness::Little);

	cmd[2] = uint8_t(h.mainCmd);
	cmd[3] = uint8_t(h.subCmd);

	std::memcpy(cmd.data() + 4, h.header, 8);

	totalCmd.push_back(cmd);
	return totalCmd;
}

std::vector<std::vector<uint8_t>> soslab::MLX::createAreaSelection(const soslab::MessageBase& dtn)
{
	std::vector<std::vector<uint8_t>> totalCmd;

	soslab::Message::GeneralMessage<std::vector<uint8_t>> msg = static_cast<const soslab::Message::GeneralMessage<std::vector<uint8_t>>&>(dtn);
	if (msg.data.size() != 5)
	{
		return totalCmd;
	}

	uint8_t numOfArea = msg.data[0];
	uint8_t areaIdx0 = msg.data[1];
	uint8_t areaIdx1 = msg.data[2];
	uint8_t areaIdx2 = msg.data[3];
	uint8_t areaIdx3 = msg.data[4];

	constexpr size_t kFrameBytes = 17;
	soslab::protocol::mlx::MLXAreaHeader h{};

	h.dataLength = kFrameBytes - 4;
	h.mainCmd = 'A';
	h.subCmd = 'S';

	std::memcpy(h.header, "AREASELC", 8);

	std::vector<uint8_t> cmd(kFrameBytes, 0);

	soslab::util::write_u16(cmd, 0, h.dataLength, soslab::util::Endianness::Little);

	cmd[2] = uint8_t(h.mainCmd);
	cmd[3] = uint8_t(h.subCmd);

	std::memcpy(cmd.data() + 4, h.header, 8);

	cmd[12] = numOfArea;

	cmd[13] = areaIdx0;
	cmd[14] = areaIdx1;
	cmd[15] = areaIdx2;
	cmd[16] = areaIdx3;

	totalCmd.push_back(cmd);
	return totalCmd;
}

bool soslab::MLX::validateJsonAck(const std::vector<uint8_t>& ack, soslab::MessageBase& out)
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

bool soslab::MLX::validateSetAreaAck(const std::vector<uint8_t>& ack, soslab::MessageBase& out, soslab::util::Endianness endian)
{
	soslab::Message::GeneralMessage<std::pair<uint16_t, uint16_t>>* msg = static_cast<soslab::Message::GeneralMessage<std::pair<uint16_t, uint16_t>>*>(&out);
	if (msg == nullptr)
	{
		return false;
	}

	uint16_t ackAreaIndex = msg->data.first;
	uint16_t ackExpectedPage = msg->data.second;

	std::string why;
	if (ack.size() < 14)
	{
		why = "ack too small";
		return false;
	}

	if (ack[2] != static_cast<uint8_t>('L') || ack[3] != static_cast<uint8_t>('A'))
	{
		why = "cmd mismatch";
		return false;
	}

	char expectedHdr[6] = { 'L','U','T','I','D','X' };
	for (size_t i = 0; i < 6; ++i)
	{
		if (ack[4 + i] != static_cast<uint8_t>(expectedHdr[i]))
		{
			why = "header mismatch";
			return false;
		}
	}

	uint16_t ackAreaNum = 0;
	if (endian == soslab::util::Endianness::Little)
		ackAreaNum = static_cast<uint16_t>((static_cast<uint16_t>(ack[11]) << 8) | static_cast<uint16_t>(ack[10]));
	else
		ackAreaNum = static_cast<uint16_t>((static_cast<uint16_t>(ack[10]) << 8) | static_cast<uint16_t>(ack[11]));
	if (ackAreaNum != static_cast<uint16_t>(ackAreaIndex))
	{
		why = "area number mismatch";
		return false;
	}

	uint16_t ackPage = 0;
	if (endian == soslab::util::Endianness::Little)
		ackPage = static_cast<uint16_t>((static_cast<uint16_t>(ack[13]) << 8) | static_cast<uint16_t>(ack[12]));
	else
		ackPage = static_cast<uint16_t>((static_cast<uint16_t>(ack[12]) << 8) | static_cast<uint16_t>(ack[13]));

	if (ackPage != ackExpectedPage)
	{
		why = "page mismatch";
		return false;
	}

	std::cerr << why << " :: " << ackPage << " :: " << ackExpectedPage << "\n";

	msg->data.second++;

	return true;
}

bool soslab::MLX::validateGetAreaAck(const std::vector<uint8_t>& ack, soslab::MessageBase& out)
{
	soslab::Message::MLX::AreaMessage& msg = static_cast<soslab::Message::MLX::AreaMessage&>(out);
	if (ack.size() < 20)
	{
		std::cerr << "ack too small\n";
		return false;
	}

	if (ack[2] != static_cast<uint8_t>('I') || ack[3] != static_cast<uint8_t>('A'))
	{
		std::cerr << "ack too small\n";
		return false;
	}

	const char hdrPrefix[6] = { 'L','U','T','I','D','X' };
	for (size_t i = 0; i < 6; ++i)
	{
		if (ack[4 + i] != static_cast<uint8_t>(hdrPrefix[i]))
		{
			std::cerr << "ack too small\n";
			return false;
		}
	}

	uint16_t minimumCount = soslab::util::read_u16(ack, 16);

	uint8_t polygonCount = ack[18];
	uint8_t vertexCount = ack[19];
	if (polygonCount > 5)
		polygonCount = 5;
	if (vertexCount > 8)
		vertexCount = 8;

	msg.area.polygons.clear();
	msg.area.detectMinimumCount = minimumCount;
	msg.area.polygons.resize(polygonCount);

	const size_t kVertexBytes = 8;
	const size_t kMaxPolygons = 5;
	const size_t kMaxVertices = 8;
	const size_t baseOff = 20;
	const size_t required = baseOff + (kMaxPolygons * kMaxVertices * kVertexBytes);
	if (ack.size() < required)
	{
		std::cerr << "ack too small (vertex payload)\n";
		return false;
	}

	for (size_t p = 0; p < polygonCount; ++p)
	{
		auto& poly = msg.area.polygons[p];
		poly.pts.clear();
		poly.pts.resize(vertexCount);

		bool empty = false;
		for (size_t v = 0; v < vertexCount; ++v)
		{
			const size_t off = baseOff + ((p * kMaxVertices + v) * kVertexBytes);
			const int32_t x = soslab::util::read_i24(ack, off + 0);
			const int32_t y = soslab::util::read_i24(ack, off + 3);
			const int16_t z = soslab::util::read_i16(ack, off + 6);
			poly.pts[v].x = static_cast<float>(x);
			poly.pts[v].y = static_cast<float>(y);
			poly.pts[v].z = static_cast<float>(z);
		}
	}

	return true;
}

bool soslab::MLX::validateSaveAreaInfo(const std::vector<uint8_t>& ack, soslab::MessageBase& out)
{
	if (ack.size() < 12)
	{
		std::cerr << "ack too small\n";
		return false;
	}

	if (ack[2] != static_cast<uint8_t>('A') || ack[3] != static_cast<uint8_t>('K'))
	{
		std::cerr << "cmd mismatch\n";
		return false;
	}

	const char hdr[8] = { 'A','R','E','A','S','A','V','E' };
	for (size_t i = 0; i < 8; ++i)
	{
		if (ack[4 + i] != static_cast<uint8_t>(hdr[i]))
		{
			std::cerr << "header mismatch\n";
			return false;
		}
	}

	return true;
}

bool soslab::MLX::validateAreaAlarm(const std::vector<uint8_t>& packet, soslab::MessageBase& out)
{
	soslab::Message::GeneralMessage<soslab::AreaAlarmData>& msg = static_cast<soslab::Message::GeneralMessage<soslab::AreaAlarmData>&>(out);

	if (packet.size() < 28)
	{
		std::cerr << "packet too small" << "\n";
		return false;
	}


	if (packet[2] != static_cast<uint8_t>('A') || packet[3] != static_cast<uint8_t>('F'))
	{
		std::cerr << "cmd mismatch" << "\n";
		return false;
	}


	const char hdr[8] = { 'A','R','E','A','F','L','A','G' };
	for (size_t i = 0; i < 8; ++i)
	{
		if (packet[4 + i] != static_cast<uint8_t>(hdr[i]))
		{
			std::cerr << "header mismatch" << "\n";
			return false;
		}
	}

	msg.data.areaIdx[0] = packet[12];
	msg.data.areaFlag[0] = packet[13];
	msg.data.areaIdx[1] = packet[14];
	msg.data.areaFlag[1] = packet[15];
	msg.data.areaIdx[2] = packet[16];
	msg.data.areaFlag[2] = packet[17];
	msg.data.areaIdx[3] = packet[18];
	msg.data.areaFlag[3] = packet[19];

	uint64_t e = 0;
	for (size_t i = 0; i < 8; ++i) e |= (static_cast<uint64_t>(packet[20 + i]) << (8 * i));

	msg.data.errorBit = e;

	return true;
}

bool soslab::MLX::validateAreaSelection(const std::vector<uint8_t>& packet, soslab::MessageBase& out)
{
	if (packet.size() < 12)
	{
		std::cerr << "ack too small\n";
		return false;
	}

	if (packet[2] != static_cast<uint8_t>('A') || packet[3] != static_cast<uint8_t>('A'))
	{
		std::cerr << "cmd mismatch\n";
		return false;
	}

	const char hdr[8] = { 'A','R','E','A','S','E','L','C' };
	for (size_t i = 0; i < 8; ++i)
	{
		if (packet[4 + i] != static_cast<uint8_t>(hdr[i]))
		{
			std::cerr << "header mismatch\n";
			return false;
		}
	}

	return true;
}

soslab::packetStatus soslab::MLX::classifyPacket(const std::vector<uint8_t>& pkt) const
{
	if (pkt.size() < 8) return packetStatus::INVALID;

	std::string header(pkt.begin(), pkt.begin() + 8);
	auto a = header.substr(2, 2);
	if (memcmp(header.c_str(), "LIDARPK", 7) == 0) return packetStatus::STREAM;
	else if (header.substr(2, 2) == "LA") return packetStatus::RESPONSE;
	else if (header.substr(2, 2) == "AK") return packetStatus::RESPONSE;
	else if (header.substr(2, 2) == "AA") return packetStatus::RESPONSE;
	else if (header.substr(2, 2) == "IA") return packetStatus::RESPONSE;
	else if (header.substr(2, 2) == "AF") return packetStatus::ALARM;
	else if (json_t::accept(pkt.begin(), pkt.end())) return packetStatus::RESPONSE;

	return packetStatus::UNKNOWN;
}

bool soslab::MLX::parseStreamData(const std::vector<uint8_t>& packetData)
{
	return buildStreamData(packetData);
}

bool soslab::MLX::buildStreamData(const std::vector<uint8_t>& packetData)
{
	if (packetData.size() < sizeof(header::headerMLX))
	{
		std::cerr << "[Error] ML Frame data size is too small: " << packetData.size() << std::endl;
		return false;
	}

	header::headerMLX header;
	memcpy(&header, packetData.data(), sizeof(header::headerMLX));

	uint8_t row_number = header.row_number;
	const uint8_t* dataPtr = packetData.data() + sizeof(header);

	frameData.timestamp[row_number] = header.timestamp;

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

		if (frameBuffer != nullptr)
		{
			frameBuffer->push(std::make_shared<FrameData>(frameData));
		}
	}

	return true;
}
