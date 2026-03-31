#include "LidarRuntime.h"
#include "Sensor.h"

#include <chrono>
#include <iostream>


static void joinAndReset(std::shared_ptr<std::thread>& th)
{
	if (th && th->joinable()) th->join();
	th.reset();
}

soslab::LidarRuntime::LidarRuntime(std::shared_ptr<Sensor> sensor,
	const soslab::lidarParameters& params,
	const QueueSizes& sizes)
	: sensor_(std::move(sensor)), params_(params), sizes_(sizes)
{
}

soslab::LidarRuntime::~LidarRuntime()
{
	stopWorkers_();
	disconnect_();
}

bool soslab::LidarRuntime::start()
{
	if (!connect_()) return false;
	if (!startWorkers_())
	{
		disconnect_();
		return false;
	}
	return true;
}

void soslab::LidarRuntime::stop()
{
	(void)disconnect_();
}

bool soslab::LidarRuntime::connect_()
{
	createQueues_();

	if (!isPlayMode_)
	{
		if (!connectNetlinks_()) return false;
	}

	attachQueues_();
	return true;
}

bool soslab::LidarRuntime::disconnect_()
{
	stopWorkers_();
	detachQueues_();

	if (!isPlayMode_)
	{
		disconnectNetlinks_();
	}

	if (fileIO_)
	{
		if (fileIO_->isOpen()) fileIO_->close();
		fileIO_.reset();
	}

	return true;
}

bool soslab::LidarRuntime::startWorkers_()
{
	if (running_.load()) return true;
	running_.store(true);

	if (tcp_ && tcpPacketQ_ && ((params_.lidarTypeValue != soslab::lidarType::GL5) && (params_.lidarTypeValue != soslab::lidarType::GL3)))
	{
		tcpTh_ = std::make_shared<std::thread>(&LidarRuntime::tcpReceiverWorker_, this);
	}
	if (udpPacketQ_)
	{
		udpTh_ = std::make_shared<std::thread>(&LidarRuntime::udpReceiverWorker_, this);
	}

	frameCbTh_ = std::make_shared<std::thread>(&LidarRuntime::frameCallbackWorker_, this);

	alarmTh_ = std::make_shared<std::thread>(&LidarRuntime::areaAlarmParserWorker_, this);

	return true;
}

void soslab::LidarRuntime::stopWorkers_()
{
	running_.store(false);
	joinAndReset(tcpTh_);
	joinAndReset(udpTh_);
	joinAndReset(frameCbTh_);
	joinAndReset(alarmTh_);
}

bool soslab::LidarRuntime::isConnected() const
{
	if (isPlayMode_)
	{
		return fileIO_ && fileIO_->isOpen();
	}

	if ((params_.lidarTypeValue == soslab::lidarType::GL5) || (params_.lidarTypeValue == soslab::lidarType::GL3))
	{
		return udp_ ? udp_->isConnected() : false;
	}
	return tcp_ ? tcp_->isConnected() : false;
}

bool soslab::LidarRuntime::sendTcp(const std::vector<uint8_t>& bytes)
{
	if (!tcp_) return false;
	return tcp_->sendMessage(bytes);
}

bool soslab::LidarRuntime::sendUdp(const std::vector<uint8_t>& bytes)
{
	if (!udp_) return false;
	return udp_->sendMessage(bytes);
}

bool soslab::LidarRuntime::popJson(json_t& out, float timeout_ms)
{
	if (!commandQ_) return false;

	std::vector<uint8_t> raw;
	if (!commandQ_->pop(raw, timeout_ms)) return false;

	try
	{
		out = json_t::parse(raw.begin(), raw.end());
		return true;
	}
	catch (const std::exception& e)
	{
		std::cerr << e.what() << '\n';
		return false;
	}
}

bool soslab::LidarRuntime::transactJson(const std::vector<uint8_t>& cmdBytes, json_t& out, float timeout_ms)
{
	if (!sendTcp(cmdBytes)) return false;
	return popJson(out, timeout_ms);
}

bool soslab::LidarRuntime::popCommand(std::vector<uint8_t>& out, float timeout_ms)
{
	if (!commandQ_) return false;
	return commandQ_->pop(out, timeout_ms);
}

void soslab::LidarRuntime::clearCommand()
{
	if (commandQ_) commandQ_->clear();
}

bool soslab::LidarRuntime::popFrame(std::shared_ptr<soslab::FrameData>& out)
{
	if (!frameQ_) return false;
	return frameQ_->pop(out, 0.0f);
}

bool soslab::LidarRuntime::popAlarm(soslab::AreaAlarmData& out, float timeout_ms)
{
	if (!areaAlarmQ_) return false;
	return areaAlarmQ_->pop(out, timeout_ms);
}

bool soslab::LidarRuntime::playStart(const std::string& filepath)
{
	stopWorkers_();
	detachQueues_();
	disconnectNetlinks_();

	fileIO_ = std::make_shared<soslab::FileIO>(filepath, FileMode::Read);
	if (!fileIO_->open())
	{
		std::cerr << "FileIO open failed: " << filepath << "\n";
		fileIO_.reset();
		return false;
	}

	isPlayMode_ = true;

	const soslab::lidarType detected = fileIO_->getDetectedLidarType();
	params_.lidarTypeValue = detected;

	sensor_.reset();
	sensor_ = Sensor::createInstance(detected);
	if (!sensor_)
	{
		std::cerr << "Sensor::createInstance failed for detected type\n";
		fileIO_->close();
		fileIO_.reset();
		isPlayMode_ = false;
		return false;
	}

	createQueues_();
	attachQueues_();

	fileIO_->registerRawBuffer(udpPacketQ_);

	startWorkers_();
	return true;
}

void soslab::LidarRuntime::playStop()
{
	stopWorkers_();
	if (fileIO_)
	{
		fileIO_->removeRawBuffer();
		fileIO_->close();
		fileIO_.reset();
	}
	isPlayMode_ = false;
}

uint64_t soslab::LidarRuntime::maximumLoggingFrame() const
{
	if (!fileIO_) return 0;
	return static_cast<uint64_t>(fileIO_->getFrameCount());
}

bool soslab::LidarRuntime::readScene(int idx)
{
	if (!fileIO_) return false;
	return fileIO_->readFrame(static_cast<size_t>(idx));
}

bool soslab::LidarRuntime::recordStart(const std::string& filepath)
{
	bool retval = false;
	if (fileIO_ != nullptr)
	{
		fileIO_->close();
	}

	fileIO_ = std::make_shared<soslab::FileIO>(filepath, FileMode::Write);
	if (!fileIO_->open())
	{
		std::cerr << "FileIO open failed: " << filepath << "\n";
		fileIO_.reset();
		isRecordMode_.store(false);
	}
	else
	{
		retval = true;
		isRecordMode_.store(true);
	}

	return retval;
}

void soslab::LidarRuntime::recordStop()
{
	isRecordMode_.store(false);
}

void soslab::LidarRuntime::createQueues_()
{
	if (!tcpPacketQ_) tcpPacketQ_ = std::make_shared<RingBuffer<std::vector<uint8_t>>>(sizes_.tcpPacketQ);
	if (!udpPacketQ_) udpPacketQ_ = std::make_shared<RingBuffer<std::vector<uint8_t>>>(sizes_.rawPacketQ);
	if (!frameQ_)     frameQ_ = std::make_shared<RingBuffer<std::shared_ptr<FrameData>>>(sizes_.frameQ);

	if (!commandQ_)       commandQ_ = std::make_shared<RingBuffer<std::vector<uint8_t>>>(sizes_.commandQ);
	if (!areaAlarmPacketQ_) areaAlarmPacketQ_ = std::make_shared<RingBuffer<std::vector<uint8_t>>>(sizes_.commandQ);
	if (!areaAlarmQ_)       areaAlarmQ_ = std::make_shared<RingBuffer<AreaAlarmData>>(sizes_.frameQ);
}

void soslab::LidarRuntime::attachQueues_()
{
	// Netlink -> Runtime queues
	if (tcp_ && tcpPacketQ_ && ((params_.lidarTypeValue != soslab::lidarType::GL5) && (params_.lidarTypeValue != soslab::lidarType::GL3)))
	{
		tcp_->registerRawBuffer(tcpPacketQ_);
	}
	if (udp_ && udpPacketQ_)
	{
		udp_->registerRawBuffer(udpPacketQ_);
	}

	// Runtime -> Sensor (friend access)
	if (sensor_)
	{
		sensor_->registerFrameBuffer(frameQ_);
		sensor_->registerCommandBuffer(commandQ_);
	}
}

void soslab::LidarRuntime::detachQueues_()
{
	if (tcp_) tcp_->removeRawBuffer();
	if (udp_) udp_->removeRawBuffer();

	if (sensor_)
	{
		sensor_->removeFrameBuffer();
	}
}

bool soslab::LidarRuntime::connectNetlinks_()
{
	if (params_.conectionTypeValue != soslab::connectionType::ETHERNET)
	{
		std::cerr << "Only ETHERNET connection is supported in current implementation.\n";
		return false;
	}

	if (((params_.lidarTypeValue != soslab::lidarType::GL5) && (params_.lidarTypeValue != soslab::lidarType::GL3)))
	{
		tcp_ = Netlink::getInstance(soslab::NetlinkType::TCP);
		soslab::TCPNetlink::IpSettings tcpSettings(
			params_.lidarIP, params_.lidarPort,
			params_.pcIP, params_.pcPort,
			1000
		);

		if (!tcp_ || !tcp_->connect(tcpSettings))
		{
			std::cerr << "TCP connect failed.\n";
			return false;
		}

		params_.pcPort = tcpSettings.pcPort;
	}

	udp_ = Netlink::getInstance(soslab::NetlinkType::UDP);
	soslab::UDPNetlink::IpSettings udpSettings(
		params_.lidarIP, params_.lidarPort,
		params_.pcIP, params_.pcPort,
		1000
	);

	if (!udp_ || !udp_->connect(udpSettings))
	{
		std::cerr << "UDP connect failed.\n";
		return false;
	}

	return true;
}

void soslab::LidarRuntime::disconnectNetlinks_()
{
	if (tcp_)
	{
		if (tcp_->isConnected()) tcp_->disconnect();
		tcp_.reset();
	}
	if (udp_)
	{
		if (udp_->isConnected()) udp_->disconnect();
		udp_.reset();
	}
}

void soslab::LidarRuntime::tcpReceiverWorker_()
{
	std::vector<uint8_t> pkt;
	while (running_.load())
	{
		if (!tcpPacketQ_) continue;
		if (!tcpPacketQ_->pop(pkt, 50.0f)) continue;
		routeTcpPayload_(pkt);
	}
}

void soslab::LidarRuntime::udpReceiverWorker_()
{
	std::vector<uint8_t> pkt;
	while (running_.load())
	{
		if (!udpPacketQ_) continue;
		if (!udpPacketQ_->pop(pkt, 50.0f)) continue;
		routeUdpPayload_(pkt);
	}
}

void soslab::LidarRuntime::frameCallbackWorker_()
{
	while (running_.load())
	{
		if (!dataCb_)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
			continue;
		}

		std::shared_ptr<soslab::FrameData> frame;
		if (frameQ_ && frameQ_->pop(frame, 50.0f))
		{
			dataCb_(frame);
		}
	}
}

void soslab::LidarRuntime::routeTcpPayload_(const std::vector<uint8_t>& pkt)
{
	if (pkt.size() >= 12 && pkt[2] == static_cast<uint8_t>('A') && pkt[3] == static_cast<uint8_t>('F'))
	{
		const char hdr[8] = { 'A','R','E','A','F','L','A','G' };
		bool isAreaFlag = true;
		for (size_t i = 0; i < 8; ++i)
		{
			if (pkt[4 + i] != static_cast<uint8_t>(hdr[i])) { isAreaFlag = false; break; }
		}
		if (isAreaFlag && areaAlarmPacketQ_)
		{
			areaAlarmPacketQ_->push(pkt);
			return;
		}
	}

	packetStatus status = sensor_->isValidPacket(pkt);
	commandQ_->push(pkt);
}

void soslab::LidarRuntime::routeUdpPayload_(const std::vector<uint8_t>& pkt)
{
	if (!sensor_) return;

	packetStatus status = sensor_->isValidPacket(pkt);

	if (status != packetStatus::INVALID)
	{
		if (status == packetStatus::STREAM)
		{
			sensor_->consumePacket(pkt);
			if (isRecordMode_.load())
			{
				fileIO_->write(pkt);
			}
		}
		else
		{
			commandQ_->push(pkt);
		}
	}

}

void soslab::LidarRuntime::areaAlarmParserWorker_()
{
	while (running_.load())
	{
		std::vector<uint8_t> raw;
		if (!areaAlarmPacketQ_ || !areaAlarmPacketQ_->pop(raw, 50.0f)) continue;

		soslab::Request req;
		req.feature = soslab::Feature::AreaAlarm;
		soslab::Message::GeneralMessage<soslab::AreaAlarmData> alarm;
		bool isFinish;


		if (sensor_->parseCommand(req, raw, alarm, isFinish))
		{
			if (areaCb_) areaCb_(alarm.data);
			else if (areaAlarmQ_) areaAlarmQ_->push(alarm.data);
		}
	}
}