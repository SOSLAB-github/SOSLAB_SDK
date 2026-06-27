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
#include "ProtocolHeader.h"
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

