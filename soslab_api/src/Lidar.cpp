#include "Lidar.h"
#include "LidarImpl.h"
#include "Endian.h"
#include "version.h"
#include <thread>
#include <chrono>
#include <algorithm>


soslab::Lidar::Lidar()
	: impl_(std::make_unique<LidarImpl>())
{
}

soslab::Lidar::Lidar(const soslab::lidarParameters& params)
	: impl_(std::make_unique<LidarImpl>(params))
{
}

soslab::Lidar::~Lidar() = default;

std::string soslab::Lidar::getSDKVersion() const
{
	return SOSLAB_SDK_VERSION;
}

void soslab::Lidar::setParameters(const soslab::lidarParameters& params)
{
	impl_->setParameters(params);
}

void soslab::Lidar::setParameters(const std::string& paramPath)
{
	impl_->setParameters(paramPath);
}

bool soslab::Lidar::connectLidar()
{
	return impl_->connectLidar();
}

bool soslab::Lidar::disconnectLidar()
{
	return impl_->disconnectLidar();
}

bool soslab::Lidar::isStreaming()
{
	return impl_->isStreaming();
}

bool soslab::Lidar::streamStart()
{
	return impl_->streamStart();
}

bool soslab::Lidar::streamStop()
{
	return impl_->streamStop();
}

bool soslab::Lidar::recordStart(const std::string filepath)
{
	return impl_->recordStart(filepath);
}

void soslab::Lidar::recordStop()
{
	return impl_->recordStop();
}

bool soslab::Lidar::playStart(const std::string filePath)
{
	return impl_->playStart(filePath);
}

void soslab::Lidar::playStop()
{
	impl_->playStop();
}

uint64_t soslab::Lidar::maximumLoggingFrame()
{
	return impl_->maximumLoggingFrame();
}

void soslab::Lidar::registerGetDataCallBack(LidarDataCallback cb)
{
	impl_->registerGetDataCallBack(cb);
}

void soslab::Lidar::unregisterGetDataCallBack()
{
	impl_->unregisterGetDataCallBack();
}

void soslab::Lidar::registerGetAreaCallBack(AreaDataCallback cb)
{
	impl_->registerGetAreaCallBack(cb);
}
void soslab::Lidar::unregisterGetAreaCallBack()
{
	impl_->unregisterGetAreaCallBack();
}

bool soslab::Lidar::getScene(std::shared_ptr<soslab::FrameData>& scene) const
{
	return impl_->getScene(scene);
}

bool soslab::Lidar::getScene(std::shared_ptr<soslab::FrameData>& scene, int idx)
{
	return impl_->getScene(scene, idx);
}

bool soslab::Lidar::readScene(int idx)
{
	return impl_->readScene(idx);
}


/* ************** */
/* GL5 USER interfaces */
/* ************** */

bool soslab::Lidar::getSerialNum(std::string& serialNum)
{
	return impl_->getSerialNum(serialNum);
}

bool soslab::Lidar::getFWVersion(std::string& fwVersion)
{
	return impl_->getFWVersion(fwVersion);
}

bool soslab::Lidar::getEthernetInfo(std::string& sensorIp, int& sensorPort, std::string& pcIp, int& pcPort, std::string& subnetMask, std::string& gatewayAddr, std::string& macAddr)
{
	return impl_->getEthernetInfo(sensorIp, sensorPort, pcIp, pcPort, subnetMask, gatewayAddr, macAddr);
}

bool soslab::Lidar::setEthernetInfo(const std::string& sensorIp, int sensorPort, const std::string& pcIp, int pcPort, const std::string& subnetMask, const std::string& gatewayAddr)
{
	return impl_->setEthernetInfo(sensorIp, sensorPort, pcIp, pcPort, subnetMask, gatewayAddr);
}

/* ************** */
/* GL5 Area interfaces */
/* ************** */

bool soslab::Lidar::getAreaFromSensor(const uint8_t areaIdx, std::vector<soslab::region_info_t>& area)
{
	return impl_->getAreaFromSensor(areaIdx, area);
}

bool soslab::Lidar::setAreaToSensor(const uint8_t areaIdx, std::vector<soslab::region_info_t>& area)
{
	return impl_->setAreaToSensor(areaIdx, area);
}

bool soslab::Lidar::getAllAreaFromSensor(std::vector<std::vector<soslab::region_info_t>>& area)
{
	return impl_->getAllAreaFromSensor(area);
}

bool soslab::Lidar::setAllAreaToSensor(std::vector<std::vector<soslab::region_info_t>>& area)
{
	return impl_->setAllAreaToSensor(area);
}

bool soslab::Lidar::getAllAreaFromFile(std::vector<std::vector<soslab::region_info_t>>& area, const std::string& filePath)
{
	return impl_->getAllAreaFromFile(area, filePath);
}

bool soslab::Lidar::setAllAreaToFile(std::vector<std::vector<soslab::region_info_t>>& area, const std::string& filePath)
{
	return impl_->setAllAreaToFile(area, filePath);
}

bool soslab::Lidar::compareAllAreaWithSensor(std::vector<bool>& compareResults, const std::vector<std::vector<soslab::region_info_t>>& areas)
{
	return impl_->compareAllAreaWithSensor(compareResults, areas);
}

/* ************** */
/* MLX interfaces */
/* ************** */

bool soslab::Lidar::getAlarm(soslab::AreaAlarmData& alarm)
{
	return impl_->getAlarm(alarm);
}

bool soslab::Lidar::setAreaLUT(uint8_t areaIndex, const soslab::area::Area& area, const std::vector<uint32_t>& minLut, const std::vector<uint32_t>& maxLut)
{
	return impl_->setAreaLUT(areaIndex, area, minLut, maxLut, soslab::util::Endianness::Little);
}

bool soslab::Lidar::saveAreaLUTToFlash()
{
	return impl_->saveAreaLUTToFlash(soslab::util::Endianness::Little);
}

bool soslab::Lidar::setAreaSelection(uint8_t compareNum, const uint8_t areaIdx0, const uint8_t areaIdx1, const uint8_t areaIdx2, const uint8_t areaIdx3)
{
	return impl_->setAreaSelection(compareNum, areaIdx0, areaIdx1, areaIdx2, areaIdx3, soslab::util::Endianness::Little);
}

bool soslab::Lidar::getAreaInfofromSensor(soslab::area::Area& area, uint8_t areaIndex)
{
	return impl_->getAreaInfofromSensor(area, areaIndex, soslab::util::Endianness::Little);
}

/* ************** */
/* SLU interfaces */
/* ************** */

/* ************** */
/*    PRIVATE     */
/* ************** */
void soslab::Lidar::initializer()
{

}
