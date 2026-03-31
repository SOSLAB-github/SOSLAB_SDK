// BSD 3-Clause License
// 
// Copyright (c) 2026, SOSLAB Co., Ltd.
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of SOSLAB nor the names of its contributors may be used
//    to endorse or promote products derived from this software without
//    specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
// 
// This file is part of the SOSLAB Lidar SDK, which uses third-party
// components such as Asio (Boost Software License 1.0) and
// nlohmann/json (MIT License). See the top-level LICENSE file for details.

#ifndef SOSLAB_LIDAR_H
#define SOSLAB_LIDAR_H

#include "soslabTypedef.h"
#include "LidarFeature.h"
#include "json.hpp"

namespace soslab
{
	class LidarImpl;

	class SOSLAB_EXPORTS Lidar
	{
	public:
		using LidarDataCallback = std::function<void(std::shared_ptr<const soslab::FrameData>)>;
		using AreaDataCallback = std::function<void(const soslab::AreaAlarmData&)>;
		typedef nlohmann::json json_t;

		Lidar();
		Lidar(const soslab::lidarParameters& params);
		~Lidar();

		std::string getSDKVersion() const;

		void setParameters(const soslab::lidarParameters& params);
		void setParameters(const std::string& paramPath);

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

		lidarType getLidarType() const { return userParameter.lidarTypeValue; }

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

		bool getAreaFromSensor(const uint8_t areaIdx, std::vector<soslab::region_info_t>& area);
		bool setAreaToSensor(const uint8_t areaIdx, std::vector<soslab::region_info_t>& area);
		bool getAllAreaFromSensor(std::vector<std::vector<soslab::region_info_t>>& area);
		bool setAllAreaToSensor(std::vector<std::vector<soslab::region_info_t>>& area);
		bool getAllAreaFromFile(std::vector<std::vector<soslab::region_info_t>>& area, const std::string& filePath);
		bool setAllAreaToFile(std::vector<std::vector<soslab::region_info_t>>& area, const std::string& filePath);
		bool compareAllAreaWithSensor(std::vector<bool>& compareResults, const std::vector<std::vector<soslab::region_info_t>>& areas);

		/* ************** */
		/* MLX interfaces */
		/* ************** */
		bool getAlarm(soslab::AreaAlarmData& alarm);
		bool setAreaLUT(uint8_t areaIndex, const soslab::area::Area& area, const std::vector<uint32_t>& minLut, const std::vector<uint32_t>& maxLut);
		bool saveAreaLUTToFlash();
		bool setAreaSelection(uint8_t compareNum, const uint8_t areaIdx0, const uint8_t areaIdx1, const uint8_t areaIdx2, const uint8_t areaIdx3);
		bool getAreaInfofromSensor(soslab::area::Area& area, uint8_t areaIndex);

	private:
		std::unique_ptr<LidarImpl> impl_;

		void initializer();

		lidarParameters userParameter;

		LidarDataCallback dataCallback;
		AreaDataCallback areaCallback;
	};
}

#endif