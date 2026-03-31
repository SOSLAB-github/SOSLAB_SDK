#include "Sensor.h"
#include <fstream>
#include <iomanip>
#include <sstream>
#include <chrono>
# include <math.h>
#include "GL3.h"
#include "GL5.h"
#include "MLX.h"
#include "MLA.h"
#include "MLU.h"
#include "SLU.h"

std::shared_ptr<soslab::Sensor> soslab::Sensor::createInstance(soslab::lidarType type)
{
	switch (type)
	{
	case soslab::lidarType::GL3:
		return std::make_shared<soslab::GL3>();
	case soslab::lidarType::GL5:
		return std::make_shared<soslab::GL5>();
	case soslab::lidarType::MLX:
		return std::make_shared<soslab::MLX>();
	case soslab::lidarType::MLA:
		return std::make_shared<soslab::MLA>();
	case soslab::lidarType::MLU:
		return std::make_shared<soslab::MLU>();
	case soslab::lidarType::SLU:
		return std::make_shared<soslab::SLU>();
	default:
		return nullptr;
	}
}

soslab::packetStatus soslab::Sensor::isValidPacket(const std::vector<uint8_t>& pkt) const
{
	if (pkt.empty())
	{
		std::cerr << "Empty packet received\n";
		return packetStatus::INVALID;
	}
	return classifyPacket(pkt);
}

void soslab::Sensor::consumePacket(const std::vector<uint8_t>& pkt)
{
	if (pkt.empty()) return;

	packetStatus st = Sensor::isValidPacket(pkt);
	switch (st)
	{
	case packetStatus::INVALID:
	case packetStatus::UNKNOWN:
	case packetStatus::RESPONSE:
		std::cerr << "Invalid or unknown packet received. Stream data is expected.\n";
		return;
	default:
		(void)parseStreamData(pkt);
		return;
	}
}

void soslab::Sensor::registerRawBuffer(std::shared_ptr<soslab::RingBuffer<std::vector<uint8_t>>> buffer)
{
	rawBuffer = buffer;
}

void soslab::Sensor::removeRawBuffer()
{
	if (rawBuffer != nullptr)
	{
		rawBuffer.reset();
	}
}

void soslab::Sensor::registerFrameBuffer(std::shared_ptr<soslab::RingBuffer<std::shared_ptr<soslab::FrameData>>> buffer)
{
	frameBuffer = buffer;
}

void soslab::Sensor::removeFrameBuffer()
{
	if (frameBuffer != nullptr)
	{
		frameBuffer.reset();
	}
}

void soslab::Sensor::registerCommandBuffer(std::shared_ptr<soslab::RingBuffer<std::vector<uint8_t>>> buffer)
{
	commandRxBuffer = buffer;
}

void soslab::Sensor::removeCommandBuffer()
{
	if (commandRxBuffer != nullptr)
	{
		commandRxBuffer.reset();
	}
}


