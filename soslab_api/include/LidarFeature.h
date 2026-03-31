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

#ifndef SOSLAB_LIDARFEATURE_H
#define SOSLAB_LIDARFEATURE_H

#include "soslabTypedef.h"
#include <string>
#include <cstring>
#include <iostream>
#include <vector>

namespace soslab
{
	inline const char* toString(soslab::lidarType t)
	{
		switch (t)
		{
		case soslab::lidarType::GL5: return "GL5";
		case soslab::lidarType::MLX: return "MLX";
		case soslab::lidarType::MLU: return "MLU";
		case soslab::lidarType::SLU: return "SLU";
		default: return "Unknown";
		}
	}

	enum class SetMode : uint8_t { Get, Set };

	enum class Feature
	{
		// common
		StreamEnable,
		// GL5
		Console,
		OperationMode,
		StreamData,
		// StreamEnable,
		SerialNum,
		FWVersion,
		EthernetInfo,
		AreaLevelData,
		AreaDataFinish,
		AreaDataCompare,

		// MLX
		AreaAlarm,
		SetAreaLUT,
		SaveAreaLUT,
		GetAreaLUT,
		SelectArea,
		// MLU
		// SLU
		Setting,
		SDPSetting,

		Unknown
	};

	struct MessageBase
	{
		virtual ~MessageBase() = default;
	};

	struct Request
	{
		Feature feature;
		SetMode mode{ SetMode::Set };
	};

	enum class ErrorCode
	{
		Ok,
		NotSupported,
		NotConnected,
		Timeout,
		ProtocolError,
		InvalidArgument,
		IOError,
		Unknown
	};

	struct Status
	{
		ErrorCode code{ ErrorCode::Ok };
		std::string message;

		static Status OK() { return {}; }
		static Status NotSupported(std::string msg) { return { ErrorCode::NotSupported, std::move(msg) }; }
		explicit operator bool() const { return code == ErrorCode::Ok; }
	};

	typedef enum
	{
		H_0 = 0,
		H_3_125,
		H_6_25,
	} HYSTERESIS;

	typedef enum
	{
		POLYGON = 0,
		ARC
	} REGION_TYPE;

	typedef enum
	{
		STRAIGHT = 0,
		FAN,
		RATIO,
	} LEVEL_TYPE;

	typedef struct _REGIONINFO
	{
		uint8_t region_type = -1;
		std::vector<double> coordsX;
		std::vector<double> coordsY;

		uint8_t level_type = 1;
		std::vector<double> levels;

		uint8_t hysteresis = 1;
		uint16_t sizeFilterValue = 0;
	} region_info_t;

}
#endif