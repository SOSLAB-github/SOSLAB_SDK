#ifndef SOSLAB_MLU_H
#define SOSLAB_MLU_H

#include "Sensor.h"

namespace soslab
{
	class MLU : public Sensor
	{
	public:
		MLU();
		~MLU();

		virtual lidarType type() const override { return lidarType::MLU; }
		virtual bool supports(Feature f) const override;

		virtual bool buildCommand(const Request& req, soslab::MessageBase& msg, std::vector<std::vector<uint8_t>>& totalProtocol) override;
		virtual bool parseCommand(const Request& req, const std::vector<uint8_t>& pkt, soslab::MessageBase& out, bool& isFinish) override;

	protected:
		virtual packetStatus classifyPacket(const std::vector<uint8_t>& pkt) const override;
		virtual bool parseStreamData(const std::vector<uint8_t>& packetData) override;
		bool buildStreamData(const std::vector<uint8_t>& packetData);

		//Create Protocol
		std::vector<std::vector<uint8_t>> createBooleanMessage(std::string key, const soslab::MessageBase& dtn);

		//Parser Protocol
		bool validateJsonAck(const std::vector<uint8_t>& ack, soslab::MessageBase& out);

	private:
		std::vector<FrameData> frameDataVec;

		//MLU Parameter
		int numCol = 256;
		int numRow = 192;
		int numTotalSize = numCol * numRow;

		int numSegmentCol = 256;
		int numSegmentRow = 6;
	};

}

#endif

