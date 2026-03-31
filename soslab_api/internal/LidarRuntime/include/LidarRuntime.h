#ifndef SOSLAB_LIDARRUNTIME_H
#define SOSLAB_LIDARRUNTIME_H

#include <atomic>
#include <functional>
#include <memory>
#include <thread>
#include <vector>
#include <string>

#include "soslabTypedef.h"
#include "ringBuffer.h"
#include "json.hpp"

#include "Netlink.h"
#include "FileIO.h"

namespace soslab
{

	class Sensor;

	class SOSLAB_EXPORTS LidarRuntime
	{
	public:
		using json_t = nlohmann::json;

		struct QueueSizes
		{
			size_t rawPacketQ = 256; // UDP payload queue
			size_t tcpPacketQ = 8;   // TCP payload queue
			size_t frameQ = 8;   // parsed frames
			size_t commandQ = 3;   // json/binary ack/alarm
		};

		using LidarDataCallback = std::function<void(std::shared_ptr<const soslab::FrameData>)>;
		using AreaDataCallback = std::function<void(const soslab::AreaAlarmData&)>;

	public:
		LidarRuntime(std::shared_ptr<Sensor> sensor,
			const soslab::lidarParameters& params,
			const QueueSizes& sizes);

		~LidarRuntime();

		bool start();
		void stop();

		bool isConnected() const;
		bool isRunning() const { return running_.load(); }

		void setDataCallback(LidarDataCallback cb) { dataCb_ = std::move(cb); }
		void clearDataCallback() { dataCb_ = nullptr; }
		void setAreaCallback(AreaDataCallback cb) { areaCb_ = std::move(cb); }
		void clearAreaCallback() { areaCb_ = nullptr; }

		bool sendTcp(const std::vector<uint8_t>& bytes);
		bool sendUdp(const std::vector<uint8_t>& bytes);

		bool popJson(json_t& out, float timeout_ms);
		bool transactJson(const std::vector<uint8_t>& cmdBytes, json_t& out, float timeout_ms);

		bool popCommand(std::vector<uint8_t>& out, float timeout_ms);
		void clearCommand();

		bool popFrame(std::shared_ptr<soslab::FrameData>& out);
		bool popAlarm(soslab::AreaAlarmData& out, float timeout_ms);

		bool playStart(const std::string& filepath);
		void playStop();
		uint64_t maximumLoggingFrame() const;
		bool readScene(int idx);

		bool recordStart(const std::string& filepath);
		void recordStop();

		std::shared_ptr<Sensor> sensor() const { return sensor_; }
		soslab::lidarType lidarTypeValue() const { return params_.lidarTypeValue; }

	private:
		bool connect_();
		bool disconnect_();

		bool startWorkers_();
		void stopWorkers_();

		void createQueues_();
		void attachQueues_();
		void detachQueues_();

		bool connectNetlinks_();
		void disconnectNetlinks_();

		void tcpReceiverWorker_();
		void udpReceiverWorker_();
		void frameCallbackWorker_();
		void areaAlarmParserWorker_();

		void routeTcpPayload_(const std::vector<uint8_t>& pkt);
		void routeUdpPayload_(const std::vector<uint8_t>& pkt);

	private:
		std::shared_ptr<Sensor> sensor_;
		soslab::lidarParameters params_;
		QueueSizes sizes_;

		// Infrastructure
		std::shared_ptr<Netlink> tcp_;
		std::shared_ptr<Netlink> udp_;
		std::shared_ptr<FileIO>  fileIO_;

		// Queues
		std::shared_ptr<RingBuffer<std::vector<uint8_t>>>   tcpPacketQ_;
		std::shared_ptr<RingBuffer<std::vector<uint8_t>>>   udpPacketQ_;
		std::shared_ptr<RingBuffer<std::shared_ptr<FrameData>>> frameQ_;

		std::shared_ptr<RingBuffer<std::vector<uint8_t>>>   commandQ_;
		std::shared_ptr<RingBuffer<std::vector<uint8_t>>>   areaAlarmPacketQ_;
		std::shared_ptr<RingBuffer<AreaAlarmData>>          areaAlarmQ_;

		// Threads
		std::atomic<bool> running_{ false };
		std::shared_ptr<std::thread> tcpTh_;
		std::shared_ptr<std::thread> udpTh_;
		std::shared_ptr<std::thread> frameCbTh_;
		std::shared_ptr<std::thread> alarmTh_;

		// Callbacks
		LidarDataCallback dataCb_{ nullptr };
		AreaDataCallback  areaCb_{ nullptr };

		bool isPlayMode_ = false;
		std::atomic<bool> isRecordMode_{ false };
	};

}
#endif // !SOSLAB_LIDARRUNTIME_H
