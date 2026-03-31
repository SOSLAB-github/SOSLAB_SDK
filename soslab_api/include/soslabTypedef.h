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

#ifndef SOSLAB_TYPEDEF_H
#define SOSLAB_TYPEDEF_H

#ifdef SOSLAB_API_EXPORTS
#if defined(_MSC_VER)
#define SOSLAB_EXPORTS __declspec(dllexport)
#elif defined(__GNUC__)
#define SOSLAB_EXPORTS __attribute__((visibility("default")))
#endif
#endif

#ifndef SOSLAB_EXPORTS
#define SOSLAB_EXPORTS
#endif

#include <iostream>
#include <string>
#include <vector>
#include <array>
#include <memory>
#include <queue>
#include <mutex>
#include <condition_variable>

namespace soslab
{


	struct Points
	{
		float x;
		float y;
		float z;

		Points() :
			x(0.0), y(0.0), z(0.0)
		{
		}

		Points(float _x, float _y, float _z) :
			x(_x), y(_y), z(_z)
		{
		}
	};

	class FrameData
	{
	public:
		uint16_t rows;
		uint16_t cols;
		uint8_t echoNum;
		uint8_t frameNum;
		uint8_t lidarId;

		std::vector<uint32_t> ambient;
		std::vector<std::vector<uint32_t>> intensity;
		std::vector<std::vector<uint32_t>> depth;
		std::vector<std::vector<Points>> points;

		FrameData() : lidarId(0), frameNum(0), cols(192), rows(56), echoNum(2), ambient(rows * 576), intensity(echoNum, std::vector<uint32_t>(rows* cols, 0)), depth(echoNum, std::vector<uint32_t>(rows* cols, 0)), points(echoNum, std::vector<Points>(rows* cols, Points())) {}
	};

	enum class lidarType
	{
		MLX,
		MLA,
		GL3,
		GL5,
		MLU,
		SLU,
		UNKNOWN
	};

	enum class connectionType
	{
		ETHERNET,
		SERIAL
	};

	struct lidarParameters
	{
		lidarType lidarTypeValue;
		connectionType conectionTypeValue;
		std::string pcIP;
		std::string lidarIP;
		int 		pcPort;
		int			lidarPort;
		std::string serialName;
		int 		serialBaudRate;

		lidarParameters()
			: lidarTypeValue(lidarType::MLX)
			, conectionTypeValue(connectionType::ETHERNET)
			, pcIP("")
			, lidarIP("")
			, pcPort(0)
			, lidarPort(0)
			, serialName("")
			, serialBaudRate(0)
		{
		}
	};

	struct AreaAlarmData
	{
		uint8_t areaIdx[4] = { 0,0,0,0 };
		uint8_t areaFlag[4] = { 0,0,0,0 };
		uint64_t errorBit = 0;
	};

	//Area Information
	namespace area
	{
		struct Polygon
		{
			std::vector<Points> pts;
		};

		struct Area
		{
			std::vector<Polygon> polygons;
			std::vector<uint32_t> minLUT;
			std::vector<uint32_t> maxLUT;
			uint16_t detectMinimumCount = 1;
		};

		struct ZoneConfig
		{
			std::vector<Area> areas;

			void isValidCheck()
			{
				for (int i = 0; i < areas.size(); i++)
				{
					for (int j = areas[i].polygons.size() - 1; j >= 0; j--)
					{
						bool empty = true;
						for (int k = 0; k < areas[i].polygons[j].pts.size(); k++)
						{
							Points& pts = areas[i].polygons[j].pts[k];
							if (pts.x != 0 || pts.y != 0 || pts.z != 0)
							{
								empty &= false;
							}
						}

						if (empty)
						{
							areas[i].polygons.erase(areas[i].polygons.begin() + j);
						}
					}
				}
			}
		};
	}

}

#endif
