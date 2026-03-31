/************************************************************************************************
Copyright (C) 2025 SOSLAB Co., Ltd.
All rights reserved.

TODO: LICENSE INFORMATION
************************************************************************************************/

#ifndef SOSLAB_NETLINK_H
#define SOSLAB_NETLINK_H

#include <malloc.h>
#include "soslabTypedef.h"
#include "ringBuffer.h"
#include <asio.hpp>

#define MAX_PACKET_SIZE 65535

namespace soslab
{

	struct NetlinkRuntime;

	enum class NetlinkType
	{
		TCP = 0,
		UDP = 1,
		SERIAL = 2
	};

	class SOSLAB_EXPORTS Netlink
	{
	public:
		class PortBase
		{
		public:
			PortBase() {};
		};

		Netlink();
		~Netlink();

		static std::shared_ptr<Netlink> getInstance(soslab::NetlinkType type);

		using receivePacketCallback = std::function<void(const std::vector<uint8_t>&)>;

		bool connect(PortBase& portBase);

		virtual void disconnect() = 0;
		virtual bool isConnected() = 0;
		virtual bool sendMessage(const std::vector<uint8_t>& msg) = 0;
		virtual bool receiveMessage() = 0;

		void registerRawBuffer(std::shared_ptr<soslab::RingBuffer<std::vector<uint8_t>>> buffer);
		void removeRawBuffer();

		void registerReceiveCallback(receivePacketCallback callback);
		bool unregisterReceiveCallback();

	protected:
		virtual bool connectImpl(asio::io_context& ioContext_, PortBase& portBase) = 0;
		std::array<uint8_t, MAX_PACKET_SIZE> buffer;
		void receivedPacket(const std::vector<uint8_t>& packet);
		std::shared_ptr<soslab::RingBuffer<std::vector<uint8_t>>> packetBuffer;

		std::shared_ptr<NetlinkRuntime> runtime_;

	private:
		receivePacketCallback packetCallback;

	};

	class TCPNetlink : public Netlink
	{
	public:
		class IpSettings : public PortBase
		{
		public:
			IpSettings(std::string lidarIP_, int lidarPort_, std::string pcIP_, int pcPort_, int timeOut_) :
				lidarIP(lidarIP_), lidarPort(lidarPort_), pcIP(pcIP_), pcPort(pcPort_), timeOut(timeOut_)
			{
			};
			~IpSettings() {};

			std::string lidarIP;
			int lidarPort;

			std::string pcIP;
			int pcPort;

			int timeOut;
		};

		TCPNetlink();
		~TCPNetlink();

		void disconnect() override;
		bool isConnected() override;
		bool sendMessage(const std::vector<uint8_t>& msg) override;
		bool receiveMessage() override;

	protected:
		bool connectImpl(asio::io_context& ioContext_, PortBase& portBase) override;
	private:
		void asycReceiver(const std::error_code& errorCode, std::size_t bytesReceived);
		std::unique_ptr<asio::ip::tcp::socket> tcpSocket;
	};

	class UDPNetlink : public Netlink
	{
	public:
		class IpSettings : public PortBase
		{
		public:
			IpSettings(std::string lidarIP_, int lidarPort_, std::string pcIP_, int pcPort_, int timeOut_) :
				lidarIP(lidarIP_), lidarPort(lidarPort_), pcIP(pcIP_), pcPort(pcPort_), timeOut(timeOut_)
			{
			};
			~IpSettings() {};

			std::string lidarIP;
			int lidarPort;

			std::string pcIP;
			int pcPort;

			int timeOut;
		};

		UDPNetlink();
		~UDPNetlink();

		void disconnect() override;
		bool isConnected() override;
		bool sendMessage(const std::vector<uint8_t>& msg) override;
		bool receiveMessage() override;

	protected:
		bool connectImpl(asio::io_context& ioContext_, PortBase& portBase) override;
	private:
		void asycReceiver(const std::error_code& errorCode, std::size_t bytesReceived);
		std::unique_ptr<asio::ip::udp::socket> udpSocket;
	};

	class SerialNetlink : public Netlink
	{
	public:
		class serialSettings : public PortBase
		{
		public:
			serialSettings(std::string portName_, int serialBaudrate_, int timeOut_) :
				portName(portName_), serialBaudrate(serialBaudrate_), timeOut(timeOut_)
			{
			};
			~serialSettings() {};

			std::string portName;
			int serialBaudrate;

			int timeOut;
		};

		SerialNetlink();
		~SerialNetlink();

		void disconnect() override;
		bool isConnected() override;
		bool sendMessage(const std::vector<uint8_t>& msg) override;
		bool receiveMessage() override;

	protected:
		bool connectImpl(asio::io_context& ioContext_, PortBase& portBase) override;
	};
}

#endif
