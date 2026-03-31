#ifndef SOSLAB_PROTOCOL_CONSTANTS_H
#define SOSLAB_PROTOCOL_CONSTANTS_H

#include <cstdint>
namespace soslab
{
	namespace protocol
	{
		namespace mlx
		{

#pragma pack(push, 1)
			struct MLXAreaHeader
			{
				uint16_t dataLength;
				char mainCmd;
				char subCmd;
				char header[8];
				uint16_t totalPage;
				uint16_t currentPage;
				uint16_t minimumCount;
				uint8_t polygonIndex;
				uint8_t vertexIndex;
			};
#pragma pack(pop)

			static constexpr uint16_t kRows = 56;
			static constexpr uint16_t kCols = 192;
			static constexpr uint8_t  kMaxAreas = 20;
			static constexpr uint8_t  kMaxPolygons = 5;
			static constexpr uint8_t  kMaxVertices = 8;
			static constexpr size_t   kLutPageBytes = 1024;

			static constexpr size_t kVertexBytes = 8;

			static constexpr size_t kFixedHeaderBytes = 20;
			static constexpr size_t kVertexSectionBytes = kMaxPolygons * kMaxVertices * kVertexBytes;
			static constexpr size_t kMetaBytes = kFixedHeaderBytes + kVertexSectionBytes;

			static constexpr size_t kLutBytes = static_cast<size_t>(kRows) * static_cast<size_t>(kCols) * 2u * sizeof(uint32_t) + kVertexSectionBytes; // 86016

			static constexpr size_t kPacketBytes = 86364;
		}

	}
}

#endif