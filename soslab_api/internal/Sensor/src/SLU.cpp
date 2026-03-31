#include "SLU.h"

#include <fstream>
#include <iomanip>
#include <sstream>
#include <chrono>
# include <math.h>

soslab::SLU::SLU()
{
	std::cout << "SLU created." << std::endl;
}

soslab::SLU::~SLU()
{
	std::cout << "SLU deleted." << std::endl;
}

bool soslab::SLU::supports(Feature f) const
{
	switch (f)
	{
	case Feature::StreamEnable:
		return true;
	default:
		std::cerr << "Selected feature is not supported in SLU.\n";
		return false;
	}
}

bool soslab::SLU::buildCommand(const Request& req, soslab::MessageBase& msg, std::vector<std::vector<uint8_t>>& totalProtocol)
{
	return !totalProtocol.empty();
}

bool soslab::SLU::parseCommand(const Request& req, const std::vector<uint8_t>& pkt, soslab::MessageBase& out, bool& isFinish)
{
	return false;
}

soslab::packetStatus soslab::SLU::classifyPacket(const std::vector<uint8_t>& pkt) const
{
	return packetStatus::UNKNOWN;
}

bool soslab::SLU::parseStreamData(const std::vector<uint8_t>& packetData)
{
	return buildStreamData(packetData);
}

bool soslab::SLU::buildStreamData(const std::vector<uint8_t>& packetData)
{
	return false;
}
