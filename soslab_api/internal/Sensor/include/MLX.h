#ifndef SOSLAB_MLX_H
#define SOSLAB_MLX_H

#include "Sensor.h"

namespace soslab
{
	class MLX : public Sensor
	{
	public:
		MLX();
		~MLX();

		virtual lidarType type() const override { return lidarType::MLX; }
		virtual bool supports(Feature f) const override;

		virtual bool buildCommand(const Request& req, soslab::MessageBase& msg, std::vector<std::vector<uint8_t>>& totalProtocol) override;
		virtual bool parseCommand(const Request& req, const std::vector<uint8_t>& pkt, soslab::MessageBase& out, bool& isFinish) override;

	protected:
		virtual bool parseStreamData(const std::vector<uint8_t>& packetData) override;
		virtual packetStatus classifyPacket(const std::vector<uint8_t>& pkt) const override;
		bool buildStreamData(const std::vector<uint8_t>& packetData);

		//Create Protocol
		std::vector<std::vector<uint8_t>> createBooleanMessage(std::string key, const soslab::MessageBase& dtn);
		std::vector<std::vector<uint8_t>> createSetAreaInfo(const soslab::MessageBase& dtn);
		std::vector<std::vector<uint8_t>> createGetAreaInfo(const soslab::MessageBase& dtn);
		std::vector<std::vector<uint8_t>> createSaveAreaInfo();
		std::vector<std::vector<uint8_t>> createAreaSelection(const soslab::MessageBase& dtn);

		//Parser Protocol
		bool validateJsonAck(const std::vector<uint8_t>& ack, soslab::MessageBase& out);
		bool validateSetAreaAck(const std::vector<uint8_t>& ack, soslab::MessageBase& out, soslab::util::Endianness endian = soslab::util::Endianness::Little);
		bool validateGetAreaAck(const std::vector<uint8_t>& ack, soslab::MessageBase& out);
		bool validateSaveAreaInfo(const std::vector<uint8_t>& ack, soslab::MessageBase& out);
		bool validateAreaAlarm(const std::vector<uint8_t>& packet, soslab::MessageBase& out);
		bool validateAreaSelection(const std::vector<uint8_t>& packet, soslab::MessageBase& out);

	private:
		FrameData frameData;
	};

}

#endif

