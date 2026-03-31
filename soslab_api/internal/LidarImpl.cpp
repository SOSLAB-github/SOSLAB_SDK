#include "LidarImpl.h"
#include <thread>
#include <chrono>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <sstream>

#include "LidarFeature.h"

namespace soslab
{
	namespace utill
	{
		inline bool json_write_to_file(const char* name, const nlohmann::json& json_arg)
		{
			std::fstream text_write;
			text_write.open(name, std::fstream::out | std::fstream::trunc);

			if (!text_write.is_open())
			{
				return false;
			}

			bool array_check = false;

			try
			{
				std::string json_dump = json_arg.dump();
				std::vector<char> write_buffer;
				write_buffer.reserve(json_dump.size() + 1000);

				for (auto it = json_dump.begin(); it != json_dump.end(); ++it)
				{
					if ('[' == *it)
					{
						array_check = true;
					}
					else if (']' == *it)
					{
						array_check = false;
					}

					if ((('{' == *it) || (',' == *it)) && !array_check)
					{
						write_buffer.push_back(*it);
						write_buffer.push_back('\n');
						write_buffer.push_back(' ');
						write_buffer.push_back(' ');
						write_buffer.push_back(' ');
					}
					else if ('}' == *it && !array_check)
					{
						write_buffer.push_back('\n');
						write_buffer.push_back(*it);
					}
					else if ((('{' == *it) || (',' == *it) || ('}' == *it)) && array_check)
					{
						write_buffer.push_back(*it);
					}
					else if (!('{' == *it) && !(',' == *it) && !('}' == *it))
					{
						write_buffer.push_back(*it);
					}
				}
				write_buffer.shrink_to_fit();

				if (text_write.is_open())
				{
					text_write.write(write_buffer.data(), write_buffer.size());
				}

				text_write.close();

				json_dump.clear();
				json_dump.shrink_to_fit();
				write_buffer.clear();
				write_buffer.shrink_to_fit();
			}
			catch (const nlohmann::json::parse_error&)
			{
				std::cerr << "json parse error" << std::endl;
				return false;
			}
			catch (const nlohmann::json::invalid_iterator&)
			{
				std::cerr << "json invalid_iterator error" << std::endl;
				return false;
			}
			catch (const nlohmann::json::type_error&)
			{
				std::cerr << "json type error" << std::endl;
				return false;
			}
			catch (const nlohmann::json::out_of_range&)
			{
				std::cerr << "json out_of_range error" << std::endl;
				return false;
			}
			catch (const nlohmann::json::other_error&)
			{
				std::cerr << "json other error" << std::endl;
				return false;
			}

			return true;
		}

		inline bool json_read_from_file(const char* name, nlohmann::json& json_arg)
		{
			std::fstream text_read;
			text_read.open(name, std::fstream::in | std::fstream::binary);

			if (!text_read.is_open())
			{
				return false;
			}

			text_read.seekg(0, std::fstream::end);
			std::streamsize size = text_read.tellg();
			text_read.seekg(0, std::fstream::beg);

			std::vector<char> read_buffer(size);
			text_read.read(read_buffer.data(), size);

			text_read.close();

			try
			{
				json_arg = nlohmann::json::parse(read_buffer);
			}
			catch (const nlohmann::json::parse_error&)
			{
				std::cerr << "json parse error" << std::endl;
				return false;
			}
			catch (const nlohmann::json::invalid_iterator&)
			{
				std::cerr << "json invalid_iterator error" << std::endl;
				return false;
			}
			catch (const nlohmann::json::type_error&)
			{
				std::cerr << "json type error" << std::endl;
				return false;
			}
			catch (const nlohmann::json::out_of_range&)
			{
				std::cerr << "json out_of_range error" << std::endl;
				return false;
			}
			catch (const nlohmann::json::other_error&)
			{
				std::cerr << "json other error" << std::endl;
				return false;
			}

			read_buffer.clear();
			read_buffer.shrink_to_fit();

			return true;
		}

	}
}

soslab::LidarImpl::LidarImpl() :
	dataCallback(nullptr),
	areaCallback(nullptr),
	sensorInterface(nullptr),
	runtime_(nullptr)
{
	//set default
	userParameter.conectionTypeValue = soslab::connectionType::ETHERNET;
	userParameter.lidarTypeValue = soslab::lidarType::MLX;
	userParameter.lidarIP = "192.168.1.10";
	userParameter.pcIP = "0.0.0.0";
	userParameter.lidarPort = 2000;
	userParameter.pcPort = 0;

	sizeRawBuffer = 256;
	sizeFrameBuffer = 8;
	sizeCommandBuffer = 3;
}

soslab::LidarImpl::LidarImpl(const soslab::lidarParameters& params) :
	LidarImpl()
{
	setParameters(params);
}

soslab::LidarImpl::~LidarImpl()
{
	disconnectLidar();
}

void soslab::LidarImpl::setParameters(const soslab::lidarParameters& params)
{
	userParameter = params;
}

void soslab::LidarImpl::setParameters(const std::string& paramPath)
{
	userParameter = parserParamterFile(paramPath);
}

bool soslab::LidarImpl::sendRequest(const Request& req, MessageBase& inData, int ackTimeOutMs)
{
	MessageBase dummyOut;
	return sendRequest(req, inData, dummyOut, ackTimeOutMs);
}

bool soslab::LidarImpl::sendRequest(const Request& req, MessageBase& inData, MessageBase& outData, int ackTimeOutMs)
{
	if (!runtime_ || !sensorInterface) return false;

	std::vector<std::vector<uint8_t>> cmd;
	bool retval = false;

	if (!sensorInterface->buildCommand(req, inData, cmd))
	{
		std::cerr << "sendRequest: buildCommand failed\n";
		return false;
	}

	runtime_->clearCommand();
	std::vector<uint8_t> ack;
	bool isFinish = true;

	// Build and send all command pages
	for (int i = 0; i < cmd.size(); i++)
	{
		if (userParameter.lidarTypeValue == soslab::lidarType::GL5 || userParameter.lidarTypeValue == soslab::lidarType::GL3)
		{
			retval = runtime_->sendUdp(cmd[i]);
			if (!retval)
			{
				std::cerr << "sendRequest: UDP send failed\n";
				return false;
			}

			do
			{
				if (!runtime_->popCommand(ack, ackTimeOutMs))
				{
					std::cerr << "sendRequest: popCommand timeout (UDP)\n";
					return false;
				}
				if (!sensorInterface->parseCommand(req, ack, outData, isFinish))
				{
					std::cerr << "sendRequest: parseCommand failed\n";
					return false;
				}
			} while (!isFinish);
		}
		else
		{
			retval = runtime_->sendTcp(cmd[i]);
			if (!retval)
			{
				std::cerr << "sendRequest: TCP send failed\n";
				return false;
			}

			do
			{
				if (!runtime_->popCommand(ack, ackTimeOutMs))
				{
					std::cerr << "sendRequest: popCommand timeout (TCP)\n";
					return false;
				}
				if (!sensorInterface->parseCommand(req, ack, outData, isFinish))
				{
					std::cerr << "sendRequest: parseCommand failed\n";
					return false;
				}
			} while (!isFinish);
		}
	}

	if (!retval) std::cerr << "sendRequest failed.\n";

	return retval;
}

bool soslab::LidarImpl::connectLidar()
{
	initializer();

	sensorInterface = Sensor::createInstance(userParameter.lidarTypeValue);
	if (!sensorInterface)
	{
		std::cerr << "Failed to create sensor interface.\n";
		return false;
	}

	LidarRuntime::QueueSizes qs;
	qs.rawPacketQ = sizeRawBuffer;
	qs.tcpPacketQ = sizeFrameBuffer;
	qs.frameQ = sizeFrameBuffer;
	qs.commandQ = sizeCommandBuffer;

	runtime_ = std::make_shared<LidarRuntime>(sensorInterface, userParameter, qs);

	runtime_->setDataCallback(dataCallback);
	runtime_->setAreaCallback(areaCallback);

	if (!runtime_->start())
	{
		std::cerr << "Runtime start failed.\n";
		runtime_.reset();
		sensorInterface.reset();
		return false;
	}
	return true;
}

bool soslab::LidarImpl::disconnectLidar()
{
	if (runtime_)
	{
		runtime_->stop();
		runtime_.reset();
	}
	sensorInterface.reset();
	return true;
}

bool soslab::LidarImpl::isStreaming()
{
	return runtime_ ? runtime_->isConnected() : false;
}

bool soslab::LidarImpl::streamStart()
{
	if (!runtime_ || !sensorInterface) return false;

	Request req;
	req.feature = Feature::StreamEnable;
	req.mode = SetMode::Set;

	soslab::Message::GeneralMessage<bool> msg;
	msg.data = true;

	return sendRequest(req, msg, 5000);
}

bool soslab::LidarImpl::streamStop()
{
	if (!runtime_ || !sensorInterface) return false;

	Request req;
	req.feature = Feature::StreamEnable;
	req.mode = SetMode::Set;
	soslab::Message::GeneralMessage<bool> msg;
	msg.data = false;

	return sendRequest(req, msg, 5000);
}

bool soslab::LidarImpl::recordStart(const std::string filepath)
{
	return runtime_->recordStart(filepath);
}

void soslab::LidarImpl::recordStop()
{
	runtime_->recordStop();
}

bool soslab::LidarImpl::playStart(const std::string filePath)
{
	initializer();
	sensorInterface.reset();
	LidarRuntime::QueueSizes qs;
	qs.rawPacketQ = sizeRawBuffer;
	qs.tcpPacketQ = sizeFrameBuffer;
	qs.frameQ = sizeFrameBuffer;
	qs.commandQ = sizeCommandBuffer;

	runtime_ = std::make_shared<LidarRuntime>(nullptr, userParameter, qs);
	runtime_->setDataCallback(dataCallback);
	runtime_->setAreaCallback(areaCallback);

	const bool retval = runtime_->playStart(filePath);
	if (retval)
	{
		sensorInterface = runtime_->sensor();
		userParameter.lidarTypeValue = runtime_->lidarTypeValue();
	}
	return retval;
}

void soslab::LidarImpl::playStop()
{
	if (runtime_) runtime_->playStop();
}

uint64_t soslab::LidarImpl::maximumLoggingFrame()
{
	return runtime_ ? runtime_->maximumLoggingFrame() : 0;
}

void soslab::LidarImpl::registerGetDataCallBack(LidarDataCallback cb)
{
	dataCallback = std::move(cb);
	if (runtime_) runtime_->setDataCallback(dataCallback);
}

void soslab::LidarImpl::unregisterGetDataCallBack()
{
	dataCallback = nullptr;
	if (runtime_) runtime_->clearDataCallback();
}

void soslab::LidarImpl::registerGetAreaCallBack(AreaDataCallback cb)
{
	areaCallback = std::move(cb);
	if (runtime_) runtime_->setAreaCallback(areaCallback);
}
void soslab::LidarImpl::unregisterGetAreaCallBack()
{
	areaCallback = nullptr;
	if (runtime_) runtime_->clearAreaCallback();
}

soslab::lidarType soslab::LidarImpl::getLidarType() const
{
	return lidarType::GL5;
}

bool soslab::LidarImpl::getScene(std::shared_ptr<soslab::FrameData>& scene) const
{
	if (!runtime_) return false;
	return runtime_->popFrame(scene);
}

bool soslab::LidarImpl::getScene(std::shared_ptr<soslab::FrameData>& scene, int idx)
{
	if (!runtime_) return false;
	if (!readScene(idx)) return false;
	return runtime_->popFrame(scene);
}

bool soslab::LidarImpl::readScene(int idx)
{
	if (!runtime_) return false;
	return runtime_->readScene(idx);
}

/* ************** */
/* GL5 USER interfaces */
/* ************** */

bool soslab::LidarImpl::getSerialNum(std::string& serialNum)
{
	if (!runtime_ || !sensorInterface)
	{
		std::cerr << "Please Check Initializer.\n";
		return false;
	}
	if (!sensorInterface->supports(Feature::SerialNum)) return false;

	Request req;
	req.feature = Feature::SerialNum;
	req.mode = SetMode::Get;

	soslab::Message::GeneralMessage<bool> msg;
	msg.data = true;

	soslab::Message::GeneralMessage<std::string> outMsg;
	bool retval = sendRequest(req, msg, outMsg, 5000);

	if (retval)
	{
		soslab::Message::GeneralMessage<std::string>* serialNumMsg = static_cast<soslab::Message::GeneralMessage<std::string>*>(&outMsg);
		if (serialNumMsg == nullptr)
		{
			return false;
		}
		serialNum = serialNumMsg->data;
	}

	return retval;
}

bool soslab::LidarImpl::getFWVersion(std::string& fwVersion)
{
	if (!runtime_ || !sensorInterface)
	{
		std::cerr << "Please Check Initializer.\n";
		return false;
	}
	if (!sensorInterface->supports(Feature::FWVersion)) return false;

	Request req;
	req.feature = Feature::FWVersion;
	req.mode = SetMode::Get;

	soslab::Message::GeneralMessage<bool> msg;
	msg.data = true;

	soslab::Message::GeneralMessage<std::string> outMsg;

	bool retval = sendRequest(req, msg, outMsg, 5000);

	if (retval)
	{
		soslab::Message::GeneralMessage<std::string>* fwVersionMsg = static_cast<soslab::Message::GeneralMessage<std::string>*>(&outMsg);
		if (fwVersionMsg == nullptr)
		{
			return false;
		}
		std::ostringstream oss_formatted_date;
		oss_formatted_date << std::setfill('0') << std::setw(2) << (int)fwVersionMsg->data[0] << "-" << std::setw(2) << (int)fwVersionMsg->data[1] << "-" << std::setw(2) << (int)fwVersionMsg->data[2];
		fwVersion = oss_formatted_date.str();
	}

	return retval;
}

bool soslab::LidarImpl::getEthernetInfo(std::string& sensorIp, int& sensorPort, std::string& pcIp, int& pcPort, std::string& subnetMask, std::string& gatewayAddr, std::string& macAddr)
{
	if (!runtime_ || !sensorInterface)
	{
		std::cerr << "Please Check Initializer.\n";
		return false;
	}
	if (!sensorInterface->supports(Feature::EthernetInfo)) return false;

	Request req;
	req.feature = Feature::EthernetInfo;
	req.mode = SetMode::Get;

	soslab::Message::GeneralMessage<bool> msg;
	msg.data = true;

	soslab::Message::GL5::EthernetInfoMessage outMsg;

	bool retval = sendRequest(req, msg, outMsg, 5000);
	soslab::Message::GL5::EthernetInfoMessage* ethernetInfoMsg = static_cast<soslab::Message::GL5::EthernetInfoMessage*>(&outMsg);
	if (ethernetInfoMsg == nullptr)
	{
		return false;
	}

	sensorIp = ethernetInfoMsg->sensorIp;
	pcIp = ethernetInfoMsg->pcIp;
	subnetMask = ethernetInfoMsg->subnetMask;
	gatewayAddr = ethernetInfoMsg->gatewayAddr;
	macAddr = ethernetInfoMsg->macAddr;
	pcPort = ethernetInfoMsg->pcPort;
	sensorPort = ethernetInfoMsg->sensorPort;

	return retval;
}

bool soslab::LidarImpl::setEthernetInfo(const std::string& sensorIp, int sensorPort, const std::string& pcIp, int pcPort, const std::string& subnetMask, const std::string& gatewayAddr)
{
	if (!runtime_ || !sensorInterface)
	{
		std::cerr << "Please Check Initializer.\n";
		return false;
	}
	if (!sensorInterface->supports(Feature::EthernetInfo)) return false;

	std::string currentSensorIp;
	int currentSensorPort;
	std::string currentPcIp;
	int currentPcPort;
	std::string currentSubnetMask;
	std::string currentGatewayAddr;
	std::string currentMacAddr;

	getEthernetInfo(currentSensorIp, currentSensorPort, currentPcIp, currentPcPort, currentSubnetMask, currentGatewayAddr, currentMacAddr);

	Request req;
	req.feature = Feature::EthernetInfo;
	req.mode = SetMode::Set;

	soslab::Message::GL5::EthernetInfoMessage msg;
	msg.sensorIp = sensorIp;
	msg.pcIp = pcIp;
	msg.subnetMask = subnetMask;
	msg.gatewayAddr = gatewayAddr;
	msg.macAddr = currentMacAddr;
	msg.pcPort = pcPort;
	msg.sensorPort = sensorPort;

	bool retval = sendRequest(req, msg, 5000);

	return retval;
}

/* ************** */
/* GL5 AREA interfaces */
/* ************** */

// AREA
bool soslab::LidarImpl::getAreaFromSensor(const uint8_t areaIdx, std::vector<soslab::region_info_t>& area)
{
	if (!runtime_ || !sensorInterface)
	{
		std::cerr << "Please Check Initializer.\n";
		return false;
	}
	if (!sensorInterface->supports(Feature::AreaLevelData)) return false;

	Request req;
	req.feature = Feature::AreaLevelData;
	req.mode = SetMode::Get;

	soslab::Message::GeneralMessage<uint32_t> msg;
	msg.data = areaIdx;

	soslab::Message::GL5::AreaDataMessage outMsg;
	outMsg.areaNum = areaIdx;
	outMsg.area = area;

	bool result = sendRequest(req, msg, outMsg, 5000);

	if (result)
	{
		area = outMsg.area;
	}

	return result;
}

bool soslab::LidarImpl::setAreaToSensor(const uint8_t areaIdx, std::vector<soslab::region_info_t>& area)
{
	if (!runtime_ || !sensorInterface)
	{
		std::cerr << "Please Check Initializer.\n";
		return false;
	}
	if (!sensorInterface->supports(Feature::AreaLevelData)) return false;

	Request req;
	req.feature = Feature::AreaLevelData;
	req.mode = SetMode::Set;

	soslab::Message::GL5::AreaDataMessage msg;
	msg.areaNum = areaIdx;
	msg.area = area;

	return sendRequest(req, msg, 5000);
}

bool soslab::LidarImpl::getAllAreaFromSensor(std::vector<std::vector<soslab::region_info_t>>& areas)
{
	if (!runtime_ || !sensorInterface)
	{
		std::cerr << "Please Check Initializer.\n";
		return false;
	}
	if (!sensorInterface->supports(Feature::AreaLevelData)) return false;

	uint8_t areaNumTotal = 64;

	areas.clear();

	for (uint8_t i = 1; i < areaNumTotal; i++)
	{
		std::vector<soslab::region_info_t> area;
		if (!getAreaFromSensor(i, area))
		{
			return false;
		}
		areas.push_back(area);
	}
	return true;
}

bool soslab::LidarImpl::setAllAreaToSensor(std::vector<std::vector<soslab::region_info_t>>& areas)
{
	if (!runtime_ || !sensorInterface)
	{
		std::cerr << "Please Check Initializer.\n";
		return false;
	}
	if (!sensorInterface->supports(Feature::AreaLevelData)) return false;

	if (areas.empty())
	{
		std::cerr << "No area data to set.\n";
		return false;
	}

	uint8_t areaNumTotal = static_cast<uint8_t>(areas.size());

	for (uint8_t i = 0; i < areaNumTotal; i++)
	{
		if (!setAreaToSensor(i + 1, areas[i]))
		{
			std::cerr << "Failed to set area " << static_cast<int>(i + 1) << "\n";
			return false;
		}
		else
		{
			if (sensorInterface->type() == lidarType::GL5)
			{
				if (!setAreaFinish())
				{
					std::cerr << "Failed to set area finish\n";
					return false;
				}
			}
		}
	}

	return true;
}

bool soslab::LidarImpl::getAllAreaFromFile(std::vector<std::vector<soslab::region_info_t>>& areas, const std::string& filePath)
{
	std::string inputPath = filePath.empty() ? "area_setting.json" : filePath;

	try
	{
		nlohmann::json jsonRoot;

		if (!utill::json_read_from_file(inputPath.c_str(), jsonRoot))
		{
			std::cerr << "[FAILED] Failed to read file: " << inputPath << std::endl;
			return false;
		}

		if (!jsonRoot.is_array())
		{
			std::cerr << "[FAILED] Invalid JSON format: expected array of areas" << std::endl;
			return false;
		}

		areas.clear();

		for (const auto& jsonArea : jsonRoot)
		{
			if (!jsonArea.is_array())
			{
				std::cerr << "[WARNING] Skipping non-array area" << std::endl;
				continue;
			}

			std::vector<soslab::region_info_t> area;

			for (const auto& jsonRegion : jsonArea)
			{
				soslab::region_info_t region;

				if (jsonRegion.contains("regionType"))
					region.region_type = static_cast<uint8_t>(jsonRegion["regionType"].get<int>());

				if (jsonRegion.contains("coords") && jsonRegion["coords"].is_array())
				{
					region.coordsX.clear();
					region.coordsY.clear();
					for (const auto& coord : jsonRegion["coords"])
					{
						if (coord.contains("x") && coord.contains("y"))
						{
							region.coordsX.push_back(coord["x"].get<double>());
							region.coordsY.push_back(coord["y"].get<double>());
						}
					}
				}

				if (jsonRegion.contains("levelType"))
					region.level_type = static_cast<uint8_t>(jsonRegion["levelType"].get<int>());

				if (jsonRegion.contains("levels"))
					region.levels = jsonRegion["levels"].get<std::vector<double>>();

				if (jsonRegion.contains("hysteresis"))
					region.hysteresis = static_cast<uint8_t>(jsonRegion["hysteresis"].get<int>());

				area.push_back(region);
			}

			areas.push_back(area);
		}

		return true;
	}
	catch (const std::exception& e)
	{
		std::cerr << "[FAILED] Exception during JSON load: " << e.what() << std::endl;
		return false;
	}
}

bool soslab::LidarImpl::setAllAreaToFile(std::vector<std::vector<soslab::region_info_t>>& areas, const std::string& filePath)
{
	std::string outputPath = filePath.empty() ? "area_setting.json" : filePath;

	if (areas.empty())
	{
		return false;
	}

	try
	{
		nlohmann::json jsonRoot = nlohmann::json::array();

		for (const auto& area : areas)
		{
			nlohmann::json jsonArea = nlohmann::json::array();

			for (const auto& region : area)
			{
				nlohmann::json jsonRegion;

				nlohmann::json coords = nlohmann::json::array();
				size_t coordCount = std::min(region.coordsX.size(), region.coordsY.size());
				for (size_t i = 0; i < coordCount; ++i)
				{
					nlohmann::json coord;
					coord["x"] = region.coordsX[i];
					coord["y"] = region.coordsY[i];
					coords.push_back(coord);
				}

				jsonRegion["coords"] = coords;
				jsonRegion["hysteresis"] = static_cast<int>(region.hysteresis);
				jsonRegion["levelType"] = static_cast<int>(region.level_type);
				jsonRegion["levels"] = region.levels;
				jsonRegion["regionType"] = static_cast<int>(region.region_type);

				jsonArea.push_back(jsonRegion);
			}

			jsonRoot.push_back(jsonArea);
		}

		if (utill::json_write_to_file(outputPath.c_str(), jsonRoot))
		{
			return true;
		}
		else
		{
			std::cerr << "[FAILED] Failed to write to file: " << outputPath << std::endl;
			return false;
		}
	}
	catch (const std::exception& e)
	{
		std::cerr << "[FAILED] Exception during JSON save: " << e.what() << std::endl;
		return false;
	}
}

bool soslab::LidarImpl::setAreaFinish()
{
	if (!runtime_ || !sensorInterface)
	{
		std::cerr << "Please Check Initializer.\n";
		return false;
	}
	if (!sensorInterface->supports(Feature::AreaDataFinish)) return false;

	Request req;
	req.feature = Feature::AreaDataFinish;
	req.mode = SetMode::Set;

	soslab::Message::GeneralMessage<bool> msg;
	msg.data = true;

	return sendRequest(req, msg, 5000);
}

bool soslab::LidarImpl::compareAreaWithSensor(bool& compareResult, const uint8_t areaIdx, const std::vector<soslab::region_info_t>& area)
{
	if (!runtime_ || !sensorInterface)
	{
		std::cerr << "Please Check Initializer.\n";
		return false;
	}
	if (!sensorInterface->supports(Feature::AreaLevelData)) return false;

	std::vector<soslab::region_info_t> areaSensor;
	if (!getAreaFromSensor(areaIdx, areaSensor))
	{
		return false;
	}

	if (area.size() != areaSensor.size())
	{
		compareResult = false;
		return false;
	}

	for (size_t i = 0; i < area.size(); i++)
	{
		soslab::region_info_t region = area[i];
		soslab::region_info_t regionSensor = areaSensor[i];
		if (region.region_type != regionSensor.region_type)
		{
			compareResult = false;
			return false;
		}
		if (!std::equal(region.coordsX.begin(), region.coordsX.end(), regionSensor.coordsX.begin(), [](double a, double b)
			{
				return std::abs(a - b) <= (1e-3 * 2);
			}))
		{
			compareResult = false;
			return false;
		}
		if (!std::equal(region.coordsY.begin(), region.coordsY.end(), regionSensor.coordsY.begin(), [](double a, double b)
			{
				return std::abs(a - b) <= (1e-3 * 2);
			}))
		{
			compareResult = false;
			return false;
		}
		if (region.level_type != regionSensor.level_type)
		{
			compareResult = false;
			return false;
		}
		if (!std::equal(region.levels.begin(), region.levels.end(), regionSensor.levels.begin(), [](double a, double b)
			{
				return std::abs(a - b) <= (1e-3 * 2);
			}))
		{
			compareResult = false;
			return false;
		}
		if (region.hysteresis != regionSensor.hysteresis)
		{
			compareResult = false;
			return false;
		}
		if (region.sizeFilterValue != regionSensor.sizeFilterValue)
		{
			compareResult = false;
			return false;
		}
	}

	compareResult = true;
	return true;
}

bool soslab::LidarImpl::compareAllAreaWithSensor(std::vector<bool>& compareResults, const std::vector<std::vector<soslab::region_info_t>>& areas)
{
	if (!runtime_ || !sensorInterface)
	{
		std::cerr << "Please Check Initializer.\n";
		return false;
	}
	if (!sensorInterface->supports(Feature::AreaDataCompare)) return false;

	uint8_t areaNumTotal = static_cast<uint8_t>(areas.size());
	compareResults.clear();
	compareResults.resize(areaNumTotal, false);

	bool success = true;
	for (uint8_t i = 0; i < areaNumTotal; i++)
	{
		bool compareResult = false;
		if (!compareAreaWithSensor(compareResult, i + 1, areas[i]))
		{
			std::cerr << "[FAILED] Area " << i + 1 << " is not compared with sensor" << std::endl;
			success = false;
		}
		compareResults[i] = compareResult;
	}

	return success;
}

/* ************** */
/* MLX interfaces */
/* ************** */

bool soslab::LidarImpl::getAlarm(soslab::AreaAlarmData& alarm)
{
	if (!runtime_ || !sensorInterface)
	{
		std::cerr << "Please Check Initializer.\n";
	}
	if (!sensorInterface->supports(Feature::AreaAlarm)) return false;

	return runtime_->popAlarm(alarm, 1000);
}

bool soslab::LidarImpl::setAreaLUT(uint8_t areaIndex, const soslab::area::Area& area, const std::vector<uint32_t>& minLut, const std::vector<uint32_t>& maxLut, soslab::util::Endianness endian)
{
	if (!runtime_ || !sensorInterface) return false;

	Request req;
	req.feature = Feature::SetAreaLUT;
	req.mode = SetMode::Set;

	soslab::Message::MLX::AreaMessage msg;
	msg.areaIndex = areaIndex;
	msg.area = area;
	msg.minLut = minLut;
	msg.maxLut = maxLut;

	soslab::Message::GeneralMessage<std::pair<uint16_t, uint16_t>> checkMsg;
	checkMsg.data.first = static_cast<uint16_t>(areaIndex);
	checkMsg.data.second = 0;

	return sendRequest(req, msg, checkMsg, 20000);
}

bool soslab::LidarImpl::getAreaInfofromSensor(soslab::area::Area& area, uint8_t areaIndex, soslab::util::Endianness endian)
{
	if (!runtime_ || !sensorInterface) return false;

	Request req;
	req.feature = Feature::GetAreaLUT;
	req.mode = SetMode::Set;

	soslab::Message::GeneralMessage<uint8_t> msg;
	msg.data = areaIndex;

	soslab::Message::MLX::AreaMessage outMsg;

	bool retval = sendRequest(req, msg, outMsg, 20000);

	area = outMsg.area;

	return retval;
}

bool soslab::LidarImpl::saveAreaLUTToFlash(soslab::util::Endianness endian)
{
	if (!runtime_ || !sensorInterface) return false;

	Request req;
	req.feature = Feature::SaveAreaLUT;
	req.mode = SetMode::Set;

	soslab::Message::GeneralMessage<uint8_t> msg;

	return sendRequest(req, msg, 25000);
}

bool soslab::LidarImpl::setAreaSelection(uint8_t compareNum, const uint8_t areaIdx0, const uint8_t areaIdx1, const uint8_t areaIdx2, const uint8_t areaIdx3, soslab::util::Endianness endian)
{
	if (!runtime_ || !sensorInterface) return false;

	Request req;
	req.feature = Feature::SelectArea;
	req.mode = SetMode::Set;

	soslab::Message::GeneralMessage<std::vector<uint8_t>> msg;
	msg.data.resize(5);
	msg.data[0] = compareNum;
	msg.data[1] = areaIdx0;
	msg.data[2] = areaIdx1;
	msg.data[3] = areaIdx2;
	msg.data[4] = areaIdx3;

	return sendRequest(req, msg, 25000);
}

/* ************** */
/* SLU interfaces */
/* ************** */

void soslab::LidarImpl::initializer()
{

}

soslab::lidarParameters soslab::LidarImpl::parserParamterFile(const std::string& paramPath)
{
	soslab::lidarParameters outputParameters;

	return outputParameters;
}