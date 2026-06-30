/************************************************************************************************
Copyright (C) 2026 SOSLAB Co., Ltd.
All rights reserved.

************************************************************************************************/

#ifndef SOSLAB_PROTOCOLHEADER_H
#define SOSLAB_PROTOCOLHEADER_H

#include <stdint.h>

namespace soslab
{
	namespace header
	{
#pragma pack(push, 1)
		// GL Lidar Packet
		enum class PayloadSM : uint8_t
		{
			SM_SET = 0,
			SM_GET,
			SM_STREAM,
			SM_ERROR = 255
		};

		enum class PayloadBI : uint8_t
		{
			PC2DEV = 0x21,
			DEV2PC = 0x12
		};

		struct headerGL5
		{
			uint8_t ps0 = 0xC3;
			uint8_t ps1 = 0x51;
			uint8_t ps2 = 0xA1;
			uint8_t ps3 = 0xF8;
			uint16_t total_length = 0x000F;
			uint8_t pageIdx;
			uint8_t pageLength;
			PayloadSM sm;
			PayloadBI bi;
			uint16_t cat = 0x020A;
		};

		// ML Lidar Packet Header
		struct headerMLX
		{
			char header[8];
			uint64_t timestamp;
			uint64_t status;
			union
			{
				struct
				{
					uint8_t multi_echo : 1;
					uint8_t rsvd : 3;
					uint8_t depth_completion : 1;
					uint8_t ambient_disable : 1;
					uint8_t depth_disable : 1;
					uint8_t intensity_disable : 1;
				}type;
				uint8_t packet_type;
			};
			uint8_t frame_id;
			uint8_t row_number;
			uint8_t rsvd[5];
		};

		//MLU Lidar Packet Header
		struct headerMLUv10
		{
			char header[8];
			uint16_t frame_id;
			uint64_t timestamp;
			uint8_t type;
			uint8_t hroll_vroll;
		};

		//MLU Lidar Packet Header
		struct headerMLUv20
		{
			char header[8];
			uint16_t frame_id;
			uint64_t timestamp;
			uint64_t status;
			uint8_t type;
			uint8_t hroll;
			uint8_t vroll;
		};

		// SL Lidar Packet Header
		struct headerSL
		{
			uint8_t ProtocolVersionMajor;
			uint8_t ProtocolVersionMinor;
			uint8_t ProductID;
			uint8_t InstanceID;
		};
#pragma pack(pop)
	}
}
#endif