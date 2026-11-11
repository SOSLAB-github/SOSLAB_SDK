#ifndef SOSLAB_HELPERFUNCTIONS_H
#define SOSLAB_HELPERFUNCTIONS_H

#include <iostream>
#include <algorithm>
#include <cctype>
#include <cstdint>
#include <sstream>
#include <string>
#include <fstream>

#include "json.hpp"

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
		inline bool isValidPort(int port, bool allowZero = false)
		{
			const int minPort = allowZero ? 0 : 1;
			return port >= minPort && port <= 65535;
		}

		inline bool parseIpv4(const std::string& ip, uint32_t* out = nullptr)
		{
			if (ip.empty()) return false;

			std::istringstream ss(ip);
			std::string token;
			uint32_t value = 0;
			int count = 0;

			while (std::getline(ss, token, '.'))
			{
				if (token.empty()) return false;
				if (!std::all_of(token.begin(), token.end(),
					[](unsigned char c) { return std::isdigit(c); }))
				{
					return false;
				}

				int octet = 0;
				try
				{
					octet = std::stoi(token);
				}
				catch (...)
				{
					return false;
				}

				if (octet < 0 || octet > 255) return false;

				value = (value << 8) | static_cast<uint32_t>(octet);
				++count;
			}

			if (count != 4) return false;
			if (out) *out = value;
			return true;
		}

		inline bool isValidSubnetMask(const std::string& mask)
		{
			uint32_t value = 0;
			if (!parseIpv4(mask, &value)) return false;

			bool seenZero = false;
			for (int bit = 31; bit >= 0; --bit)
			{
				const bool isOne = ((value >> bit) & 0x1) != 0;
				if (!isOne) seenZero = true;
				else if (seenZero) return false;
			}

			return value != 0;
		}
	}
}

#endif