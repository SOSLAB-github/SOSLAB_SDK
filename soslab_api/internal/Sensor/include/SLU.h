#ifndef SOSLAB_SLU_H
#define SOSLAB_SLU_H

#include "Sensor.h"

namespace soslab
{
	class SLU : public Sensor
	{
	public:
		SLU();
		~SLU();

		virtual lidarType type() const override { return lidarType::SLU; }
		virtual bool supports(Feature f) const override;

		virtual bool buildCommand(const Request& req, soslab::MessageBase& msg, std::vector<std::vector<uint8_t>>& totalProtocol) override;
		virtual bool parseCommand(const Request& req, const std::vector<uint8_t>& pkt, soslab::MessageBase& out, bool& isFinish) override;

	protected:
		virtual bool parseStreamData(const std::vector<uint8_t>& packetData) override;
		virtual packetStatus classifyPacket(const std::vector<uint8_t>& pkt) const override;

		bool buildStreamData(const std::vector<uint8_t>& packetData);

	private:
		FrameData frameData;
	};

}

#endif

