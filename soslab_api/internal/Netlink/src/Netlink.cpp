#include "Netlink.h"
#include <thread>

namespace soslab
{
	struct NetlinkRuntime
	{
		asio::io_context ioc;
		asio::executor_work_guard<asio::io_context::executor_type> guard;
		std::thread worker;

		NetlinkRuntime()
			: guard(asio::make_work_guard(ioc)),
			worker([this]()
				{
					// Runs until ioc.stop() is called.
					ioc.run();
				})
		{
		}

		~NetlinkRuntime()
		{
			guard.reset();
			ioc.stop();
			if (worker.joinable())
			{
				// Best-effort: avoid self-join deadlock.
				if (worker.get_id() == std::this_thread::get_id())
				{
					worker.detach();
				}
				else
				{
					worker.join();
				}
			}
		}

		static std::shared_ptr<NetlinkRuntime> get()
		{
			static std::shared_ptr<NetlinkRuntime> inst = std::make_shared<NetlinkRuntime>();
			return inst;
		}
	};
}

soslab::Netlink::Netlink()
{
}

soslab::Netlink::~Netlink()
{
}

bool soslab::Netlink::connect(PortBase& portBase)
{
	runtime_ = soslab::NetlinkRuntime::get();
	return connectImpl(runtime_->ioc, portBase);
}

std::shared_ptr<soslab::Netlink> soslab::Netlink::getInstance(soslab::NetlinkType type)
{
	switch (type)
	{
	case soslab::NetlinkType::TCP:
		return std::make_shared<soslab::TCPNetlink>();
		break;
	case soslab::NetlinkType::UDP:
		return std::make_shared<soslab::UDPNetlink>();
		break;
	case soslab::NetlinkType::SERIAL:
		return std::make_shared<soslab::SerialNetlink>();
		break;
	default:
		return nullptr;
	}
}

void soslab::Netlink::registerRawBuffer(std::shared_ptr<soslab::RingBuffer<std::vector<uint8_t>>> buffer)
{
	packetBuffer = buffer;
}

void soslab::Netlink::removeRawBuffer()
{
	if (packetBuffer != nullptr)
	{
		packetBuffer.reset();
	}
}

void soslab::Netlink::registerReceiveCallback(receivePacketCallback callback)
{
	packetCallback = callback;
}

bool soslab::Netlink::unregisterReceiveCallback()
{
	bool retval = false;
	if (packetCallback != nullptr)
	{
		packetCallback = nullptr;
		retval = true;
	}
	return retval;
}

void soslab::Netlink::receivedPacket(const std::vector<uint8_t>& packet)
{
	if (packetCallback != nullptr)
	{
		packetCallback(packet);
	}
}

//////////////////////////////////////////////////////////////////////////////

soslab::TCPNetlink::TCPNetlink()
{
	tcpSocket = nullptr;
}

soslab::TCPNetlink::~TCPNetlink()
{
}

bool soslab::TCPNetlink::connectImpl(asio::io_context& ioContext_, PortBase& portBase)
{
	bool retval = false;

	soslab::TCPNetlink::IpSettings& tcpInfo = static_cast<soslab::TCPNetlink::IpSettings&>(portBase);

	asio::ip::tcp::endpoint localEndpoint = asio::ip::tcp::endpoint(asio::ip::address::from_string(tcpInfo.pcIP), tcpInfo.pcPort);
	asio::ip::tcp::endpoint serverEndpoint = asio::ip::tcp::endpoint(asio::ip::address::from_string(tcpInfo.lidarIP), tcpInfo.lidarPort);

	if (tcpSocket == nullptr)
	{
		tcpSocket = std::make_unique<asio::ip::tcp::socket>(ioContext_);
	}

	try
	{
		if (tcpSocket->is_open())
		{
			tcpSocket->close();
		}

		tcpSocket->open(asio::ip::tcp::v4());
		tcpSocket->set_option(asio::ip::tcp::socket::reuse_address(true));
		tcpSocket->bind(localEndpoint);
		tcpSocket->connect(serverEndpoint);

		std::cout << "TCP :: Connected :: " << tcpSocket->local_endpoint() << std::endl;
		tcpInfo.pcIP = tcpSocket->local_endpoint().address().to_string();
		tcpInfo.pcPort = tcpSocket->local_endpoint().port();

		if (!receiveMessage())
		{
			std::cerr << "TCP :: Failed to setup receive message" << std::endl;
			if (tcpSocket != nullptr && tcpSocket->is_open())
			{
				tcpSocket->close();
			}
			return false;
		}
		retval = true;
	}
	catch (std::exception& err)
	{
		std::cerr << "TCP :: Connect Fail :: " << err.what() << std::endl;
		if (tcpSocket != nullptr && tcpSocket->is_open())
		{
			tcpSocket->close();
		}
	}

	return retval;
}

void soslab::TCPNetlink::disconnect()
{
	if (tcpSocket != nullptr)
	{
		tcpSocket->cancel();
		tcpSocket->close();
	}
}

bool soslab::TCPNetlink::isConnected()
{
	bool retval = false;
	if (tcpSocket != nullptr)
	{
		retval = tcpSocket->is_open();
	}
	return retval;
}

bool soslab::TCPNetlink::sendMessage(const std::vector<uint8_t>& msg)
{
	bool retval = false;
	try
	{
		if (tcpSocket != nullptr && tcpSocket->is_open())
		{
			std::size_t bytesSent = tcpSocket->send(asio::buffer(msg));
			if (bytesSent > 0)
			{
				retval = true;
			}
		}
	}
	catch (std::exception& errorMsg)
	{
		std::cerr << "TCP :: Send Fail :: " << errorMsg.what() << std::endl;
	}
	return retval;
}

void soslab::TCPNetlink::asycReceiver(const std::error_code& errorCode, std::size_t bytesReceived)
{
	if (!errorCode && bytesReceived > 0)
	{
		std::vector<uint8_t> packet(buffer.begin(), buffer.begin() + bytesReceived);
		if (packetBuffer != nullptr)
		{
			packetBuffer->push(packet);
		}
		else
		{
			receivedPacket(packet);
		}

		tcpSocket->async_receive(asio::buffer(buffer), std::bind(&TCPNetlink::asycReceiver, this, std::placeholders::_1, std::placeholders::_2));
	}
	else if (errorCode.value() != asio::error::operation_aborted)
	{
		std::cerr << "TCP :: AsyncReceiver error :: " << errorCode.message() << std::endl;
	}
}

bool soslab::TCPNetlink::receiveMessage()
{
	bool retval = false;

	if (tcpSocket != nullptr && tcpSocket->is_open())
	{
		try
		{
			tcpSocket->async_receive(asio::buffer(buffer),
				std::bind(&TCPNetlink::asycReceiver, this, std::placeholders::_1, std::placeholders::_2));
			retval = true;
		}
		catch (const std::exception& e)
		{
			std::cerr << "TCP :: Async receive setup failed: " << e.what() << std::endl;
		}
	}
	else
	{
		std::cerr << "TCP :: Cannot setup async receive - socket is not open" << std::endl;
	}

	return retval;
}

//////////////////////////////////////////////////////////////////////////////

soslab::UDPNetlink::UDPNetlink()
{

}

soslab::UDPNetlink::~UDPNetlink()
{
}

bool soslab::UDPNetlink::connectImpl(asio::io_context& ioContext_, PortBase& portBase)
{
	bool retval = false;

	const soslab::UDPNetlink::IpSettings& udpInfo = static_cast<const soslab::UDPNetlink::IpSettings&>(portBase);

	asio::ip::udp::endpoint localEndpoint = asio::ip::udp::endpoint(asio::ip::address::from_string(udpInfo.pcIP), udpInfo.pcPort);
	asio::ip::udp::endpoint serverEndpoint = asio::ip::udp::endpoint(asio::ip::address::from_string(udpInfo.lidarIP), udpInfo.lidarPort);

	if (udpSocket == nullptr)
	{
		udpSocket = std::make_unique<asio::ip::udp::socket>(ioContext_);
	}

	try
	{
		udpSocket->open(asio::ip::udp::v4());
		udpSocket->bind(localEndpoint);

		// IF UDP Send is required, connect() is more effective
		udpSocket->connect(serverEndpoint);

		std::cout << "UDP :: Connected :: " << udpSocket->local_endpoint() << std::endl;

		if (!receiveMessage())
		{
			std::cerr << "UDP :: Failed to setup receive message" << std::endl;
		}
		retval = true;
	}
	catch (std::exception& err)
	{
		std::cerr << "UDP :: Connect Fail :: " << err.what() << std::endl;
	}

	return retval;
}

void soslab::UDPNetlink::disconnect()
{
	if (udpSocket != nullptr)
	{
		udpSocket->cancel();
		udpSocket->close();
	}
}

bool soslab::UDPNetlink::isConnected()
{
	bool retval = false;
	if (udpSocket != nullptr)
	{
		retval = udpSocket->is_open();
	}
	return retval;
}

bool soslab::UDPNetlink::sendMessage(const std::vector<uint8_t>& msg)
{
	bool retval = false;
	try
	{
		if (udpSocket != nullptr && udpSocket->is_open())
		{
			std::size_t bytesSent = udpSocket->send(asio::buffer(msg));
			if (bytesSent > 0)
			{
				retval = true;
			}
		}
	}
	catch (std::exception& errorMsg)
	{
		std::cerr << "TCP :: Send Fail :: " << errorMsg.what() << std::endl;
	}
	return retval;
}

void soslab::UDPNetlink::asycReceiver(const std::error_code& errorCode, std::size_t bytesReceived)
{
	if (!errorCode && bytesReceived > 0)
	{
		std::vector<uint8_t> packet(buffer.begin(), buffer.begin() + bytesReceived);

		if (packetBuffer != nullptr)
		{
			packetBuffer->push(packet);
		}
		else
		{
			receivedPacket(packet);
		}

		udpSocket->async_receive(asio::buffer(buffer), std::bind(&UDPNetlink::asycReceiver, this, std::placeholders::_1, std::placeholders::_2));
	}
	else if (errorCode.value() != asio::error::operation_aborted)
	{
		std::cerr << "UDP :: AsyncReceiver error :: " << errorCode.message() << std::endl;
	}
}

bool soslab::UDPNetlink::receiveMessage()
{
	bool retval = false;

	if (udpSocket != nullptr && udpSocket->is_open())
	{
		try
		{
			udpSocket->async_receive(asio::buffer(buffer),
				std::bind(&UDPNetlink::asycReceiver, this, std::placeholders::_1, std::placeholders::_2));
			retval = true;
		}
		catch (const std::exception& e)
		{
			std::cerr << "UDP :: Async receive setup failed: " << e.what() << std::endl;
		}
	}
	else
	{
		std::cerr << "UDP :: Cannot setup async receive - socket is not open" << std::endl;
	}

	return retval;
}

//////////////////////////////////////////////////////////////////////////////

soslab::SerialNetlink::SerialNetlink()
{
}

soslab::SerialNetlink::~SerialNetlink()
{
}

bool soslab::SerialNetlink::connectImpl(asio::io_context& ioContext_, PortBase& portBase)
{
	return false;
}

void soslab::SerialNetlink::disconnect()
{

}

bool soslab::SerialNetlink::isConnected()
{
	return false;
}

bool soslab::SerialNetlink::sendMessage(const std::vector<uint8_t>& msg)
{
	return false;
}

bool soslab::SerialNetlink::receiveMessage()
{
	return false;
}
