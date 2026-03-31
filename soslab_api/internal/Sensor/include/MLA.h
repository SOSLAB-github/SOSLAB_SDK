#ifndef SOSLAB_MLA_H
#define SOSLAB_MLA_H

#include "Sensor.h"

namespace soslab
{
	class MLA : public Sensor
	{
	public:
		MLA();
		~MLA();

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


		//Parser Protocol
		bool validateJsonAck(const std::vector<uint8_t>& ack, soslab::MessageBase& out);

	private:
		std::vector<FrameData> frameDataVec;
	};

}

#endif

