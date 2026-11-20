#include "FileIO.h"
#include "ProtocolHeader.h"
#include <algorithm>
#include <cctype>
#include <cstring>

soslab::FileIO::FileIO(const std::string& path, FileMode mode)
	: filePath(path), mode(mode), rawBuffer(nullptr), detectedLidarType(soslab::lidarType::UNKNOWN),
	isWriting(false), writeThread(nullptr), msgBuffer(nullptr)
{
	std::cerr << filePath << "\n";

	std::cout << "FileIO created." << std::endl;
}

soslab::FileIO::~FileIO()
{
	close();
}

bool soslab::FileIO::open()
{
	bool retval = true;
	std::ios::openmode openMode = std::ios::binary;

	std::size_t found = filePath.find_last_of(".");
	std::string extension = (found == std::string::npos) ? "" : filePath.substr(found + 1);
	std::transform(extension.begin(), extension.end(), extension.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

	std::cerr << filePath << "\n";

	switch (mode)
	{
	case FileMode::Read:

		if (extension == "pcap")
		{
			extenstionType = 1;
		}
		else if (extension == "pcapng")
		{
			extenstionType = 2;
		}
		else if (extension == "bin")
		{
			extenstionType = 0;
		}
		else
		{
			std::cerr << "[Error] file format is not supported. \n";
			retval = false;
		}

		fs.open(filePath.c_str(), std::ios::in | openMode);

		if (!fs.is_open() || !preprocessFile())
		{
			retval = false;
		}

		break;

	case FileMode::Write:
		fs.open(filePath.c_str(), std::ios::out | std::ios::trunc | openMode);

		if (fs.is_open())
		{
			isWriting = true;

			msgBuffer = std::make_shared<soslab::RingBuffer<std::vector<uint8_t>>>(8);
			writeThread = std::make_shared<std::thread>(&soslab::FileIO::messageWriter, this);
		}
		else
		{
			retval = false;
		}
		break;
	default:
		retval = false;
	}

	return retval;
}

void soslab::FileIO::close()
{
	if (mode == FileMode::Write)
	{
		isWriting = false;
		if (writeThread != nullptr)
		{
			writeThread->join();
			writeThread.reset();
		}
		if (msgBuffer != nullptr)
		{
			msgBuffer.reset();
		}
	}

	if (fs.is_open()) fs.close();
}

void soslab::FileIO::registerRawBuffer(std::shared_ptr<soslab::RingBuffer<std::vector<uint8_t>>> buffer)
{
	rawBuffer = buffer;
}

void soslab::FileIO::removeRawBuffer()
{
	if (rawBuffer != nullptr)
	{
		rawBuffer.reset();
	}
}

bool soslab::FileIO::preprocessFile()
{
	if (mode != FileMode::Read || !fs.is_open())
	{
		std::cerr << "[Error] File is not open in read mode." << std::endl;
		return false;
	}

	if (extenstionType == 1)
	{
		fs.seekg(sizeof(GLOBAL_HDR), std::ios::beg);
	}
	else
	{
		fs.seekg(0, std::ios::beg);
	}
	frameIdx.clear();


	uint8_t prevFrameId = 0;
	bool isFirstFrame = true;
	size_t filePositionDiff = 0;
	size_t filePositionOld = 0;

	std::vector<uint8_t> readData;

	int64_t currentPos;
	int64_t segmentStartPos = -1;

	while (fs.good())
	{
		switch (extenstionType)
		{
		case 0:
		{
			//bin
			currentPos = fs.tellg();
			logHeader header;
			if (!readHeader(header))
			{
				continue;
			}

			readData.resize(header.size);
			fs.read(reinterpret_cast<char*>(readData.data()), header.size);

			break;
		}
		case 1:
		{
			const int64_t packetPos = fs.tellg();

			if (segmentStartPos < 0)
			{
				segmentStartPos = packetPos;
			}

			if (!getPayloadfromPCAP(readData, segmentStartPos))
			{
				//Need More Reassemble
				continue;
			}

			currentPos = segmentStartPos;
			segmentStartPos = -1;
			break;
		}
		case 2:
		{
			const int64_t blockPos = fs.tellg();

			if (segmentStartPos < 0)
			{
				segmentStartPos = blockPos;
			}

			if (!getPayloadfromPCAPNG(readData, segmentStartPos))
			{
				//Need More Reassemble
				continue;
			}

			currentPos = segmentStartPos;
			segmentStartPos = -1;
			break;
		}
		}


		if (memcmp(readData.data(), "LIDARPKT", 8) == 0)
		{
			if (detectedLidarType == soslab::lidarType::UNKNOWN)
			{
				detectedLidarType = soslab::lidarType::MLX;
			}
			header::headerMLX packet;
			memcpy(&packet, readData.data(), sizeof(header::headerMLX));

			if (packet.row_number == 0)
			{
				frameIdx.push_back(currentPos);
			}
		}
		else if (memcmp(readData.data(), "LIDARPK", 7) == 0)
		{
			if (detectedLidarType == soslab::lidarType::UNKNOWN)
			{
				detectedLidarType = soslab::lidarType::MLA;
			}
			header::headerMLX packet;
			memcpy(&packet, readData.data(), sizeof(header::headerMLX));

			if (packet.row_number == 0)
			{
				frameIdx.push_back(currentPos);
			}
		}
		else if (memcmp(readData.data(), "MUUSR", 5) == 0)
		{
			if (detectedLidarType == soslab::lidarType::UNKNOWN)
			{
				detectedLidarType = soslab::lidarType::MLU;
			}
			header::headerMLUv20 headerInfo;
			memcpy(reinterpret_cast<char*>(&headerInfo), readData.data(), sizeof(header::headerMLUv20));

			int hroll = 0;
			int vroll = 0;

			if (headerInfo.header[5] == '1')
			{
				header::headerMLUv10 headerInfov10;
				memcpy(reinterpret_cast<char*>(&headerInfov10), readData.data(), sizeof(header::headerMLUv10));

				hroll = (headerInfov10.hroll_vroll) >> 6;
				vroll = headerInfov10.hroll_vroll & 0x3F;
			}
			else if (headerInfo.header[5] == '2')
			{
				hroll = headerInfo.hroll;
				vroll = headerInfo.vroll;
			}

			if (vroll == 0)
			{
				frameIdx.push_back(currentPos);
			}
		}
		else if (isGLHeader(readData))
		{
			if (detectedLidarType == soslab::lidarType::UNKNOWN)
			{
				detectedLidarType = soslab::lidarType::GL5;
			}

			header::headerGL5 packet;
			memcpy(&packet, readData.data(), sizeof(header::headerGL5));

			if (packet.pageIdx == 0 && packet.cat == 0x0201)
			{
				frameIdx.push_back(currentPos);
			}
		}
	}

	fs.clear();
	fs.seekg(0, std::ios::beg);

	return true;
}

bool soslab::FileIO::readFrame(size_t frameIndex)
{
	if (frameIndex + 1 >= frameIdx.size())
	{
		std::cerr << "[Error] Invalid frame index: " << frameIndex << std::endl;
		return false;
	}

	fs.seekg(frameIdx[frameIndex], std::ios::beg);

	int64_t endPos = frameIdx[frameIndex + 1];

	std::vector<uint8_t> readData;
	int64_t currentPos;

	while (fs.tellg() < endPos)
	{
		switch (extenstionType)
		{
		case 0:
		{
			//bin
			currentPos = fs.tellg();
			logHeader header;
			if (!readHeader(header))
			{
				continue;
			}

			readData.resize(header.size);
			fs.read(reinterpret_cast<char*>(readData.data()), header.size);

			break;
		}
		case 1:
		{
			if (!getPayloadfromPCAP(readData, currentPos))
			{
				//Need More Reassemble
				continue;
			}
			break;
		}
		case 2:
		{
			if (!getPayloadfromPCAPNG(readData, currentPos))
			{
				//Need More Reassemble
				continue;
			}
			break;
		}
		}

		if (rawBuffer != nullptr)
		{
			rawBuffer->push(readData);
		}
	}

	return true;
}

bool soslab::FileIO::write(const std::vector<uint8_t>& msg)
{
	bool retval = true;
	if (mode == FileMode::Write)
	{
		if (fs.is_open() && msgBuffer != nullptr)
		{
			int pushTry = 0;
			while (!msgBuffer->push(msg))
			{
				std::this_thread::yield();
				if (pushTry++ == 10)
				{
					std::cerr << __FUNCTION__ << " :: Push Fail\n";
					break;
				}
			}
		}
		else
		{
			std::cerr << "FileIO : File is not open\n";
			retval = false;
		}
	}
	else
	{
		std::cerr << "FileIO : Current Mode is not Write\n";
		retval = false;
	}
	return retval;
}


void soslab::FileIO::messageWriter()
{
	std::vector<uint8_t> msg;
	while (isWriting)
	{
		if (msgBuffer->pop(msg))
		{
			if (fs.is_open())
			{
				logHeader header;
				std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
				uint64_t now_tick = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count();

				header = { now_tick, msg.size() };

				fs.write(reinterpret_cast<const char*>(&header), sizeof(header));
				fs.write(reinterpret_cast<const char*>(msg.data()), msg.size());
			}
		}
	}
}

bool soslab::FileIO::getPayloadfromPCAP(std::vector<uint8_t>& payload, int64_t& recordStartPos)
{
	const int64_t localPos = static_cast<int64_t>(fs.tellg());

	PCAP_HDR pcapHeader;

	if (!fs.read(reinterpret_cast<char*>(&pcapHeader), sizeof(pcap_pkthdr)))
	{
		return false;
	}

	if (pcapHeader.caplen == 0)
	{
		return false;
	}

	std::vector<uint8_t> temp(pcapHeader.caplen);
	if (!fs.read(reinterpret_cast<char*>(temp.data()), pcapHeader.caplen))
	{
		return false;
	}

	return getPayloadfromPacketData(temp, payload, localPos, recordStartPos);
}

bool soslab::FileIO::getPayloadfromPCAPNG(std::vector<uint8_t>& payload, int64_t& recordStartPos)
{
	const int64_t localPos = static_cast<int64_t>(fs.tellg());
	if (localPos < 0)
	{
		return false;
	}

	constexpr uint32_t enhancedPacketBlock = 0x00000006;
	constexpr uint32_t simplePacketBlock = 0x00000003;

	PCAPNG_BLOCK_HDR blockHeader;
	if (!fs.read(reinterpret_cast<char*>(&blockHeader), sizeof(PCAPNG_BLOCK_HDR)))
	{
		return false;
	}

	if (blockHeader.blockTotalLength < sizeof(PCAPNG_BLOCK_HDR) + sizeof(uint32_t))
	{
		fs.setstate(std::ios::failbit);
		return false;
	}

	const int64_t nextBlockPos = localPos + static_cast<int64_t>(blockHeader.blockTotalLength);

	if (blockHeader.blockType == enhancedPacketBlock)
	{
		if (blockHeader.blockTotalLength < sizeof(PCAPNG_BLOCK_HDR) + sizeof(PCAPNG_EPB_HDR) + sizeof(uint32_t))
		{
			fs.seekg(nextBlockPos, std::ios::beg);
			return false;
		}

		PCAPNG_EPB_HDR epbHeader;
		if (!fs.read(reinterpret_cast<char*>(&epbHeader), sizeof(PCAPNG_EPB_HDR)))
		{
			return false;
		}

		const uint32_t minBlockLength = static_cast<uint32_t>(sizeof(PCAPNG_BLOCK_HDR) + sizeof(PCAPNG_EPB_HDR) + sizeof(uint32_t));
		if (epbHeader.capturedLen == 0 || epbHeader.capturedLen > blockHeader.blockTotalLength - minBlockLength)
		{
			fs.seekg(nextBlockPos, std::ios::beg);
			return false;
		}

		std::vector<uint8_t> temp(epbHeader.capturedLen);
		if (!fs.read(reinterpret_cast<char*>(temp.data()), epbHeader.capturedLen))
		{
			return false;
		}

		fs.seekg(nextBlockPos, std::ios::beg);
		return getPayloadfromPacketData(temp, payload, localPos, recordStartPos);
	}

	if (blockHeader.blockType == simplePacketBlock)
	{
		if (blockHeader.blockTotalLength < sizeof(PCAPNG_BLOCK_HDR) + sizeof(uint32_t) + sizeof(uint32_t))
		{
			fs.seekg(nextBlockPos, std::ios::beg);
			return false;
		}

		uint32_t packetLen = 0;
		if (!fs.read(reinterpret_cast<char*>(&packetLen), sizeof(packetLen)))
		{
			return false;
		}

		const uint32_t blockPayloadSize = blockHeader.blockTotalLength - static_cast<uint32_t>(sizeof(PCAPNG_BLOCK_HDR) + sizeof(uint32_t) + sizeof(uint32_t));
		const uint32_t capturedLen = std::min(packetLen, blockPayloadSize);
		if (capturedLen == 0)
		{
			fs.seekg(nextBlockPos, std::ios::beg);
			return false;
		}

		std::vector<uint8_t> temp(capturedLen);
		if (!fs.read(reinterpret_cast<char*>(temp.data()), capturedLen))
		{
			return false;
		}

		fs.seekg(nextBlockPos, std::ios::beg);
		return getPayloadfromPacketData(temp, payload, localPos, recordStartPos);
	}

	fs.seekg(nextBlockPos, std::ios::beg);
	return false;
}

bool soslab::FileIO::getPayloadfromPacketData(const std::vector<uint8_t>& packetData, std::vector<uint8_t>& payload, int64_t packetStartPos, int64_t& recordStartPos)
{
	bool retval = false;
	bool fragment_flag = false;

	const int ether_header_size = sizeof(ETHER_HDR);
	const int ip_header_size = sizeof(IPV4_HDR);
	const int udp_header_size = sizeof(UDP_HDR);

	if (packetData.size() < static_cast<size_t>(ether_header_size + ip_header_size + udp_header_size))
	{
		return false;
	}

	std::vector<uint8_t> ipHeader;
	std::vector<uint8_t> udpHeader;

	ipHeader = std::vector<uint8_t>(packetData.begin() + ether_header_size, packetData.begin() + ether_header_size + ip_header_size);
	udpHeader = std::vector<uint8_t>(packetData.begin() + ether_header_size + ip_header_size, packetData.begin() + ether_header_size + ip_header_size + udp_header_size);
	std::vector<uint8_t> udp_data = std::vector<uint8_t>(packetData.begin() + ether_header_size + ip_header_size + udp_header_size, packetData.end());

	IPV4_HDR ipHeaderInfo;
	UDP_HDR udpHeaderInfo;
	memcpy(reinterpret_cast<char*>(&ipHeaderInfo), ipHeader.data(), sizeof(IPV4_HDR));
	memcpy(reinterpret_cast<char*>(&udpHeaderInfo), udpHeader.data(), sizeof(UDP_HDR));

	if (ipHeaderInfo.ip_protocol == 6)
	{
		//TCP
	}
	else if (ipHeaderInfo.ip_protocol == 17)
	{
		//UDP
		int offset_fragment = (ipHeaderInfo.ip_frag_offset) * 2048 + (ipHeaderInfo.ip_frag_offset1) * 8;
		int flag_fragment = ipHeaderInfo.ip_more_fragment;
		int dont_fragment = ipHeaderInfo.ip_dont_fragment;
		int start_pos = 0;

		int ip1, ip2, ip3, ip4;

		ip1 = int(ipHeaderInfo.ip_srcaddr & 0xFF);
		ip2 = int((ipHeaderInfo.ip_srcaddr >> 8) & 0xFF);
		ip3 = int((ipHeaderInfo.ip_srcaddr >> 16) & 0xFF);
		ip4 = int((ipHeaderInfo.ip_srcaddr >> 24) & 0xFF);

		if (flag_fragment == 1 && offset_fragment == 0)
		{
			recordStartPos = packetStartPos;

			fragment_flag = false;
			payload.clear();
		}
		else if (offset_fragment == 0 && flag_fragment == 0 && dont_fragment == 0)
		{
			recordStartPos = packetStartPos;
			payload.clear();
		}

		if (dont_fragment == 0 && offset_fragment != 0)
		{
			for (int i = 0; i < udpHeader.size(); i++)
			{
				payload.push_back(udpHeader[i]);
			}
		}

		for (int i = 0; i < udp_data.size(); i++)
		{
			payload.push_back(udp_data[i]);
		}

		if (dont_fragment == 1)
		{
			recordStartPos = packetStartPos;
			payload.assign(udp_data.begin(), udp_data.end());
			return true;
		}
		else if (flag_fragment == 0 && offset_fragment > 0)
		{
			fragment_flag = true;
		}
		else if (offset_fragment == 0 && flag_fragment == 0 && dont_fragment == 0)
		{
			return true;
		}

		if (fragment_flag)
		{
			retval = true;
		}
	}

	return retval;
}

bool soslab::FileIO::isSLHeader(const std::vector<uint8_t>& data)
{
	if (data.size() < sizeof(header::headerSL))
	{
		return false;
	}

	return false;
}

bool soslab::FileIO::isGLHeader(const std::vector<uint8_t>& data)
{
	if (data.size() < sizeof(header::headerGL5))
	{
		return false;
	}

	return (data[0] == 0xC3 && data[1] == 0x51 &&
		data[2] == 0xA1 && data[3] == 0xF8);
}

bool soslab::FileIO::isMLHeader(const std::vector<uint8_t>& data)
{
	if (data.size() < 8)
	{
		return false;
	}

	return (memcmp(data.data(), "LIDARPKT", 8) == 0);
}

bool soslab::FileIO::readHeader(logHeader& header)
{
	if (!fs.read(reinterpret_cast<char*>(&header), sizeof(logHeader)))
	{
		return false;
	}
	return true;
}