/************************************************************************************************
Copyright (C) 2025 SOSLAB Co., Ltd.
All rights reserved.

TODO: LINSENSE INFORMATION
************************************************************************************************/

#ifndef SOSLAB_LIDARIMPL_H
#define SOSLAB_LIDARIMPL_H

#include "Sensor.h"
#include "LidarRuntime.h"
#include "ringBuffer.h"
#include "json.hpp"
#include "GeneralMessage.h"

#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace soslab
{
	class LidarImpl
	{
	public:
		using LidarDataCallback = std::function<void(std::shared_ptr<const soslab::FrameData>)>;
		using AreaDataCallback = std::function<void(const soslab::AreaAlarmData&)>;
		typedef nlohmann::json json_t;

		LidarImpl();
		LidarImpl(const soslab::lidarParameters& params);
		~LidarImpl();

		void setParameters(const soslab::lidarParameters& params);
		void setParameters(const std::string& paramPath);

		bool sendRequest(const Request& req, MessageBase& inData, int ackTimeoutMs);
		bool sendRequest(const Request& req, MessageBase& inData, MessageBase& outData, int ackTimeoutMs);

		bool connectLidar();
		bool disconnectLidar();

		bool isStreaming();

		bool streamStart();
		bool streamStop();

		bool recordStart(const std::string filepath);
		void recordStop();

		bool playStart(const std::string filepath);
		void playStop();
		uint64_t maximumLoggingFrame();

		void registerGetDataCallBack(LidarDataCallback cb);
		void unregisterGetDataCallBack();

		void registerGetAreaCallBack(AreaDataCallback cb);
		void unregisterGetAreaCallBack();

		lidarType getLidarType() const;

		bool getScene(std::shared_ptr<soslab::FrameData>& scene) const;
		bool getScene(std::shared_ptr<soslab::FrameData>& scene, int idx);
		bool readScene(int idx);

		/* ************** */
		/* GL5 interfaces */
		/* ************** */
		bool getSerialNum(std::string& serialNum);
		bool getFWVersion(std::string& fwVersion);
		bool getEthernetInfo(std::string& sensorIp, int& sensorPort, std::string& pcIp, int& pcPort, std::string& subnetMask, std::string& gatewayAddr, std::string& macAddr);
		bool setEthernetInfo(const std::string& sensorIp, int sensorPort, const std::string& pcIp, int pcPort, const std::string& subnetMask, const std::string& gatewayAddr);
		// AREA
		bool getAreaFromSensor(const uint8_t areaIdx, std::vector<soslab::region_info_t>& area);
		bool setAreaToSensor(const uint8_t areaIdx, std::vector<soslab::region_info_t>& area);
		bool getAllAreaFromSensor(std::vector<std::vector<soslab::region_info_t>>& areas);
		bool setAllAreaToSensor(std::vector<std::vector<soslab::region_info_t>>& areas);
		bool getAllAreaFromFile(std::vector<std::vector<soslab::region_info_t>>& areas, const std::string& filePath);
		bool setAllAreaToFile(std::vector<std::vector<soslab::region_info_t>>& areas, const std::string& filePath);
		bool setAreaFinish();
		bool compareAreaWithSensor(bool& compareResult, const uint8_t areaIdx, const std::vector<soslab::region_info_t>& area);
		bool compareAllAreaWithSensor(std::vector<bool>& compareResults, const std::vector<std::vector<soslab::region_info_t>>& areas);

		/* ************** */
		/* MLX interfaces */
		/* ************** */
		bool getAlarm(soslab::AreaAlarmData& alarm);
		bool setAreaLUT(uint8_t areaIndex, const soslab::area::Area& area, const std::vector<uint32_t>& minLut, const std::vector<uint32_t>& maxLut, soslab::util::Endianness endian = soslab::util::Endianness::Little);
		bool saveAreaLUTToFlash(soslab::util::Endianness endian = soslab::util::Endianness::Little);
		bool setAreaSelection(uint8_t compareNum, const uint8_t areaIdx0, const uint8_t areaIdx1, const uint8_t areaIdx2, const uint8_t areaIdx3, soslab::util::Endianness endian = soslab::util::Endianness::Little);
		bool getAreaInfofromSensor(soslab::area::Area& area, uint8_t areaIndex, soslab::util::Endianness endian = soslab::util::Endianness::Little);

	private:
		void initializer();
		lidarParameters parserParamterFile(const std::string& paramPath);

		std::shared_ptr<Sensor>  sensorInterface;
		std::shared_ptr<LidarRuntime> runtime_;

		lidarParameters userParameter;

		size_t sizeRawBuffer = 256;
		size_t sizeFrameBuffer = 8;
		size_t sizeCommandBuffer = 3;


		//Callback
		LidarDataCallback dataCallback;
		AreaDataCallback areaCallback;
	};
}

#endif