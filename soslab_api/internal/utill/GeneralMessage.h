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

#ifndef SOSLAB_GENERALMESSAGE_H
#define SOSLAB_GENERALMESSAGE_H

#include <vector>
#include <string>

#include "soslabTypedef.h"
#include "LidarFeature.h"

namespace soslab
{
	namespace Message
	{
		template<typename T>
		struct GeneralMessage : public MessageBase
		{
			T data{};
		};

		namespace GL3
		{

		}
		namespace GL5
		{
			struct AreaDataMessage : public MessageBase
			{
				uint8_t areaNum;
				std::vector<soslab::region_info_t> area;
			};

			struct AreaLevelDataMessage : public MessageBase
			{
				uint8_t inputArea;
				uint8_t outputLevel;
			};

			struct EthernetInfoMessage : public MessageBase
			{
				std::string sensorIp;
				int sensorPort;
				std::string pcIp;
				int pcPort;
				std::string subnetMask;
				std::string gatewayAddr;
				std::string macAddr;
			};

			struct FWVersionMessage : public MessageBase
			{
				uint8_t year;
				uint8_t month;
				uint8_t day;
			};

			struct HWVersionMessage : public MessageBase
			{
				uint8_t mainPCB;
				uint8_t ldPCB;
				uint8_t pdPCB;
				uint8_t pmPCB;
				uint8_t motor;
				uint8_t communication;
			};

			struct ExternalOutputFormatMessage : public MessageBase
			{
				uint8_t invert;		// 0: normal, 1: inverted
				uint8_t codeType;	// 0: binary, 1: gray code
				uint8_t direction;	// 0: near high, 1: near low
			};

			struct HomePositionDetectionParameterMessage : public MessageBase
			{
				uint8_t hpCnt;
				uint16_t hpLowerLimit1;
				uint16_t hpUpperLimit1;
				uint16_t hpLowerLimit2;
				uint16_t hpUpperLimit2;
			};

			struct PDPCBLEDMessage : public MessageBase
			{
				bool led3;
				bool led2;
				bool led1;
				bool led0;
				bool rsvd0;
				bool rsvd1;
				bool rsvd2;
				bool rsvd3;
			};

			struct LUTMessage : public MessageBase
			{
				uint8_t currentStep;
				std::vector<uint8_t> lutData;
			};
		}
		namespace MLX
		{
			struct AreaMessage : public MessageBase
			{
				uint8_t areaIndex;
				soslab::area::Area area;
				std::vector<uint32_t> minLut;
				std::vector<uint32_t> maxLut;
			};
		}
	}
}
#endif

