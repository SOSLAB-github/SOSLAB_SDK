#ifndef SOSLAB_GL3_H
#define SOSLAB_GL3_H

#include "Sensor.h"
#include "LidarFeature.h"

namespace soslab
{
	class GL3 : public Sensor
	{
	public:
		GL3();
		~GL3();

		virtual lidarType type() const override { return lidarType::GL3; }
		virtual bool supports(Feature f) const override;

		virtual bool buildCommand(const Request& req, soslab::MessageBase& msg, std::vector<std::vector<uint8_t>>& totalProtocol) override;
		virtual bool parseCommand(const Request& req, const std::vector<uint8_t>& pkt, soslab::MessageBase& out, bool& isFinish) override;

	protected:
		virtual packetStatus classifyPacket(const std::vector<uint8_t>& pkt) const override;
		virtual bool parseStreamData(const std::vector<uint8_t>& packetData) override;


	private:

		enum class Command : uint16_t
		{
			Console = 0x0001,
			OperationMode = 0x0101,
			StreamData = 0x0102,
			StreamEnable = 0x0103,
			AreaLevelData = 0x0203,
			AreaDataFinish = 0x0204,
			SerialNum = 0x020A,
			EthernetInfo = 0x020B,
			FWVersion = 0x020C,
		};

		// Internal parseCommandData for feature-based parsing (called from parseCommand)
		bool parseCommandData(const Request& req, const std::vector<uint8_t>& dtn, soslab::MessageBase& out);

		// command packet parser
		bool parseAreaInfoAck(const std::vector<uint8_t>& ack, soslab::MessageBase& out);
		bool parseEthernetInfoAck(const std::vector<uint8_t>& ack, soslab::MessageBase& out);
		bool parseFWVersionAck(const std::vector<uint8_t>& ack, soslab::MessageBase& out);

		bool parseBoolAck(const std::vector<uint8_t>& ack, soslab::MessageBase& out);
		bool parseUInt8Ack(const std::vector<uint8_t>& ack, soslab::MessageBase& out);
		bool parseUInt16Ack(const std::vector<uint8_t>& ack, soslab::MessageBase& out);
		bool parseUInt32Ack(const std::vector<uint8_t>& ack, soslab::MessageBase& out);
		bool parseStringAck(const std::vector<uint8_t>& ack, soslab::MessageBase& out);
		bool parseDoubleAck(const std::vector<uint8_t>& ack, soslab::MessageBase& out);
		bool parseFloatAck(const std::vector<uint8_t>& ack, soslab::MessageBase& out);

		std::vector<uint8_t> BoolMsg2Bytes(const soslab::MessageBase& msg);
		std::vector<uint8_t> UInt8Msg2Bytes(const soslab::MessageBase& msg);
		std::vector<uint8_t> UInt16Msg2Bytes(const soslab::MessageBase& msg);
		std::vector<uint8_t> UInt32Msg2Bytes(const soslab::MessageBase& msg);
		std::vector<uint8_t> StringMsg2Bytes(const soslab::MessageBase& msg);
		std::vector<uint8_t> DoubleMsg2Bytes(const soslab::MessageBase& msg);
		std::vector<uint8_t> FloatMsg2Bytes(const soslab::MessageBase& msg);
		std::vector<uint8_t> EthernetInfo2Bytes(const soslab::MessageBase& msg);
		std::vector<uint8_t> AreaInfo2Bytes(const soslab::MessageBase& msg);

		void calOutLinePolygon(int frameDataSize, double h_fov, std::vector<double>& coordX, std::vector<double>& coordY, std::vector<double>& outlines);
		void calOutLineArc(int frameDataSize, double h_fov, std::vector<double>& coordX, std::vector<double>& coordY, std::vector<double>& outlines);
		void convertCoordinate(const std::vector<double>& coordX, const std::vector<double>& coordY, std::vector<double>& distances, std::vector<double>& angles);
		std::vector<double> calcCrossingPointWithPolygon(double x0, double y0, double x1, double y1, double ang, double yi);
		bool onLine(double x0, double y0, double x1, double y1, double x, double y);

		Feature catToFeature(uint16_t cat) const;

	private:
		header::headerGL5 header = header::headerGL5();

		size_t commandDtnMax = 1024;
		FrameData frameData;
		std::vector<double> angle;
		size_t accumCnt = 0;
		int prePageIdx = -1;
		bool assemFrame = false;
		size_t numAllPoints = 1000;
		static constexpr uint8_t kPageLengthFixed = 4;
		std::vector<uint8_t> payloadAccum_;
		double h_fov_ = 180.0;

		std::vector<uint8_t> cmdPayloadAccum_;
		int cmdPrevPageIdx = -1;
		bool cmdAssembling = false;
	};

}

#endif

