/************************************************************************************************
Copyright (C) 2025 SOSLAB Co., Ltd.
All rights reserved.

TODO: LINSENSE INFORMATION
************************************************************************************************/

#ifndef SOSLAB_SENSOR_H
#define SOSLAB_SENSOR_H


#include "soslabTypedef.h"
#include "LidarFeature.h"
#include "GeneralMessage.h"
#include "ringBuffer.h"
#include "json.hpp"
#include "Endian.h"
#include <vector>
#include <functional>

namespace soslab
{

	enum class packetStatus
	{
		STREAM,
		RESPONSE,
		INVALID,
		STATUS,
		ALARM,
		UNKNOWN
	};

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
		struct headerMLU
		{
			char header[8];
			uint16_t frame_id;
			uint64_t timestamp;
			uint8_t type;
			uint8_t hroll_vroll;
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

	class LidarRuntime;

	class Sensor
	{
	public:
		typedef nlohmann::json json_t;

		Sensor() = default;
		virtual ~Sensor() = default;

		virtual lidarType type() const = 0;
		virtual bool supports(Feature f) const = 0;

		static std::shared_ptr<Sensor> createInstance(lidarType lidarTypeValue);

		packetStatus isValidPacket(const std::vector<uint8_t>& pkt) const;

		void consumePacket(const std::vector<uint8_t>& pkt);

		virtual bool buildCommand(const Request& req, soslab::MessageBase& msg, std::vector<std::vector<uint8_t>>& totalProtocol) = 0;
		virtual bool parseCommand(const Request& req, const std::vector<uint8_t>& pkt, soslab::MessageBase& out, bool& isFinish) = 0;

	protected:
		virtual bool parseStreamData(const std::vector<uint8_t>& rawData) = 0;
		virtual packetStatus classifyPacket(const std::vector<uint8_t>& pkt) const = 0;

		std::shared_ptr<soslab::RingBuffer<std::vector<uint8_t>>> rawBuffer;
		std::shared_ptr<soslab::RingBuffer<std::shared_ptr<soslab::FrameData>>> frameBuffer;
		std::shared_ptr<soslab::RingBuffer<std::vector<uint8_t>>> commandRxBuffer;

	private:
		friend class soslab::LidarRuntime;

		void registerRawBuffer(std::shared_ptr<soslab::RingBuffer<std::vector<uint8_t>>> buffer);
		void removeRawBuffer();
		void registerFrameBuffer(std::shared_ptr<soslab::RingBuffer<std::shared_ptr<soslab::FrameData>>> buffer);
		void removeFrameBuffer();
		void registerCommandBuffer(std::shared_ptr<soslab::RingBuffer<std::vector<uint8_t>>> buffer);
		void removeCommandBuffer();


	};
}

#endif

