/************************************************************************************************
Copyright (C) 2025 SOSLAB Co., Ltd.
All rights reserved.
************************************************************************************************/

#ifndef SOSLAB_FILEIO_H
#define SOSLAB_FILEIO_H

#include "soslabTypedef.h"
#include "ringBuffer.h"
#include <string>
#include <vector>
#include <fstream>
#include <thread>
#include <chrono>
#include <array>
#include <map>

/* Pcap Global Header */
typedef struct pcap_file_header
{
	uint32_t magic;
	uint16_t version_major;
	uint16_t version_minor;
	int thiszone;	/* gmt to local correction */
	uint32_t sigfigs;	/* accuracy of timestamps */
	uint32_t snaplen;	/* max length saved portion of each pkt */
	uint32_t linktype;	/* data link type (LINKTYPE_*) */
}GLOBAL_HDR;

/* Pcap Generic Header */
typedef struct pcap_pkthdr
{
	uint32_t tv_sec;	/* time stamp second */
	uint32_t tv_usec;	/* time stamp microseconds */
	uint32_t caplen;	/* length of portion present */
	uint32_t len;	/* length this packet (off wire) */
}PCAP_HDR;

/* Ethernet header */
typedef struct ethernet_header
{
	unsigned char dest[6];
	unsigned char source[6];
	unsigned short type;
} ETHER_HDR;

/* IPv4 header */
typedef struct ip_hdr
{
	unsigned char ip_header_len : 4; // 4-bit header length (in 32-bit words) normally=5 (Means 20 Bytes may be 24 also)
	unsigned char ip_version : 4; // 4-bit IPv4 version	
	unsigned char ip_tos; // IP type of service	
	unsigned short ip_total_length; // Total length	
	unsigned short ip_id; // Unique identifier	
	unsigned char ip_frag_offset : 5; // Fragment offset field	
	unsigned char ip_more_fragment : 1;
	unsigned char ip_dont_fragment : 1;
	unsigned char ip_reserved_zero : 1;
	unsigned char ip_frag_offset1; //fragment offset	
	unsigned char ip_ttl; // Time to live	
	unsigned char ip_protocol; // Protocol(TCP,UDP etc)	
	unsigned short ip_checksum; // IP checksum	
	unsigned int ip_srcaddr; // Source address	
	unsigned int ip_destaddr; // Source address
} IPV4_HDR;

/* UDP header*/
typedef struct udp_header
{
	unsigned short sport;          // Source port
	unsigned short dport;          // Destination port
	unsigned short len;            // Datagram length
	unsigned short crc;            // Checksum
}UDP_HDR;

namespace soslab
{

	enum class FileMode
	{
		Read,
		Write
	};
#pragma pack(push, 1) 
	struct logHeader
	{
		uint64_t timestamp; //usec
		uint64_t size;
	};
#pragma pack(pop)

	class SOSLAB_EXPORTS FileIO
	{
	public:
		FileIO(const std::string& path, FileMode mode);
		~FileIO();

		bool open();
		void close();
		bool isOpen() const { return fs.is_open(); }

		size_t getFrameCount() const { return frameIdx.size() - 1; }
		lidarType getDetectedLidarType() const { return detectedLidarType; }

		bool readFrame(size_t frameIndex);
		bool write(const std::vector<uint8_t>& msg);

		void registerRawBuffer(std::shared_ptr<soslab::RingBuffer<std::vector<uint8_t>>> buffer);
		void removeRawBuffer();

	private:
		int extenstionType = 0;
		std::string filePath;
		FileMode mode;
		std::fstream fs;

		lidarType detectedLidarType;
		std::vector<int64_t> frameIdx;

		bool preprocessFile();

		std::shared_ptr<soslab::RingBuffer<std::vector<uint8_t>>> rawBuffer;

		bool getPayloadfromPCAP(std::vector<uint8_t>& payload, int64_t& recordStartPos);

		bool readHeader(logHeader& header);

		bool isGLHeader(const std::vector<uint8_t>& data);
		bool isMLHeader(const std::vector<uint8_t>& data);
		bool isSLHeader(const std::vector<uint8_t>& data);

		bool isWriting;
		std::shared_ptr<std::thread > writeThread;
		std::shared_ptr<soslab::RingBuffer<std::vector<uint8_t>>> msgBuffer;
		void messageWriter();

	};
}

#endif