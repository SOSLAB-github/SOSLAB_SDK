#pragma once

#include <cstdint>
#include <vector>
#include <algorithm>

namespace soslab
{
	namespace util
	{
		enum class Endianness { Little, Big };

		// ===== uint16 =====
		inline uint16_t read_u16_le(const std::vector<uint8_t>& in, size_t off)
		{
			return static_cast<uint16_t>((static_cast<uint16_t>(in[off + 1]) << 8) | static_cast<uint16_t>(in[off]));
		}

		inline uint16_t read_u16_be(const std::vector<uint8_t>& in, size_t off)
		{
			return static_cast<uint16_t>((static_cast<uint16_t>(in[off]) << 8) | static_cast<uint16_t>(in[off + 1]));
		}

		inline uint16_t read_u16(const std::vector<uint8_t>& in, size_t off, Endianness e = Endianness::Little)
		{
			if (off + 1 >= in.size())
				return 0;

			if (in[off] == 0xFF && in[off + 1] == 0xFF)
			{
				return 0;
			}

			if (e == Endianness::Little) return read_u16_le(in, off);
			else return read_u16_be(in, off);
		}

		inline void write_u16_le(std::vector<uint8_t>& out, size_t off, uint16_t v)
		{
			out.at(off + 0) = static_cast<uint8_t>(v & 0xFFu);
			out.at(off + 1) = static_cast<uint8_t>((v >> 8) & 0xFFu);
		}

		inline void write_u16_be(std::vector<uint8_t>& out, size_t off, uint16_t v)
		{
			out.at(off + 0) = static_cast<uint8_t>((v >> 8) & 0xFFu);
			out.at(off + 1) = static_cast<uint8_t>(v & 0xFFu);
		}

		inline void write_u16(std::vector<uint8_t>& out, size_t off, uint16_t v, Endianness e = Endianness::Little)
		{
			if (e == Endianness::Little) write_u16_le(out, off, v);
			else write_u16_be(out, off, v);
		}

		// ===== int16 (signed) =====
		inline int16_t read_i16_le(const std::vector<uint8_t>& in, size_t off)
		{
			return static_cast<uint16_t>((static_cast<uint16_t>(in[off + 1]) << 8) | static_cast<uint16_t>(in[off]));
		}

		inline int16_t read_i16_be(const std::vector<uint8_t>& in, size_t off)
		{
			return static_cast<uint16_t>((static_cast<uint16_t>(in[off]) << 8) | static_cast<uint16_t>(in[off + 1]));
		}

		inline int16_t read_i16(const std::vector<uint8_t>& in, size_t off, Endianness e = Endianness::Little)
		{
			if (off + 1 >= in.size())
				return 0;

			if (in[off] == 0xFF && in[off + 1] == 0xFF)
				return 0;

			if (e == Endianness::Little) return read_i16_le(in, off);
			else return read_i16_be(in, off);
		}

		inline void write_i16_le(std::vector<uint8_t>& out, size_t off, int16_t v)
		{
			const uint16_t u = static_cast<uint16_t>(v);
			out.at(off + 0) = static_cast<uint8_t>(u & 0xFFu);
			out.at(off + 1) = static_cast<uint8_t>((u >> 8) & 0xFFu);
		}

		inline void write_i16_be(std::vector<uint8_t>& out, size_t off, int16_t v)
		{
			const uint16_t u = static_cast<uint16_t>(v);
			out.at(off + 0) = static_cast<uint8_t>((u >> 8) & 0xFFu);
			out.at(off + 1) = static_cast<uint8_t>(u & 0xFFu);
		}

		inline void write_i16(std::vector<uint8_t>& out, size_t off, int16_t v, Endianness e = Endianness::Little)
		{
			if (e == Endianness::Little) write_i16_le(out, off, v);
			else write_i16_be(out, off, v);
		}

		// ===== uint32 =====
		inline int32_t read_i24_le(const std::vector<uint8_t>& in, size_t off)
		{
			uint32_t v = 0;
			v = static_cast<uint32_t>(in[off]) |
				(static_cast<uint32_t>(in[off + 1]) << 8) |
				(static_cast<uint32_t>(in[off + 2]) << 16);
			if (v & 0x00800000)
				v |= 0xFF000000;

			return static_cast<int32_t>(v);
		}

		inline int32_t read_i24_be(const std::vector<uint8_t>& in, size_t off)
		{
			uint32_t v = 0;
			v = static_cast<uint32_t>(in[off] << 16) |
				(static_cast<uint32_t>(in[off + 1]) << 8) |
				(static_cast<uint32_t>(in[off + 2]));
			if (v & 0x00800000)
				v |= 0xFF000000;

			return static_cast<int32_t>(v);
		}

		inline int32_t read_i24(const std::vector<uint8_t>& in, size_t off, Endianness e = Endianness::Little)
		{
			if (off + 2 >= in.size())
				return 0;

			if (in[off] == 0xFF && in[off + 1] == 0xFF && in[off + 2] == 0xFF)
			{
				return 0;
			}

			if (e == Endianness::Little) return read_i24_le(in, off);
			else return read_i24_be(in, off);
		}

		inline void write_u32_le(std::vector<uint8_t>& out, size_t off, uint32_t v)
		{
			out.at(off + 0) = static_cast<uint8_t>(v & 0xFFu);
			out.at(off + 1) = static_cast<uint8_t>((v >> 8) & 0xFFu);
			out.at(off + 2) = static_cast<uint8_t>((v >> 16) & 0xFFu);
			out.at(off + 3) = static_cast<uint8_t>((v >> 24) & 0xFFu);
		}

		inline void write_u32_be(std::vector<uint8_t>& out, size_t off, uint32_t v)
		{
			out.at(off + 0) = static_cast<uint8_t>((v >> 24) & 0xFFu);
			out.at(off + 1) = static_cast<uint8_t>((v >> 16) & 0xFFu);
			out.at(off + 2) = static_cast<uint8_t>((v >> 8) & 0xFFu);
			out.at(off + 3) = static_cast<uint8_t>(v & 0xFFu);
		}

		inline void write_u32(std::vector<uint8_t>& out, size_t off, uint32_t v, Endianness e = Endianness::Little)
		{
			if (e == Endianness::Little) write_u32_le(out, off, v);
			else write_u32_be(out, off, v);
		}

		// ===== int24 (signed, clamped) =====
		inline int32_t clamp_i24(int32_t v)
		{
			constexpr int32_t kMin = -(1 << 23);
			constexpr int32_t kMax = (1 << 23) - 1;
			return std::max(kMin, std::min(kMax, v));
		}

		inline void write_i24_le(std::vector<uint8_t>& out, size_t off, int32_t v)
		{
			v = clamp_i24(v);
			const uint32_t u = static_cast<uint32_t>(v) & 0x00FFFFFFu;
			out.at(off + 0) = static_cast<uint8_t>(u & 0xFFu);
			out.at(off + 1) = static_cast<uint8_t>((u >> 8) & 0xFFu);
			out.at(off + 2) = static_cast<uint8_t>((u >> 16) & 0xFFu);
		}

		inline void write_i24_be(std::vector<uint8_t>& out, size_t off, int32_t v)
		{
			v = clamp_i24(v);
			const uint32_t u = static_cast<uint32_t>(v) & 0x00FFFFFFu;
			out.at(off + 0) = static_cast<uint8_t>((u >> 16) & 0xFFu);
			out.at(off + 1) = static_cast<uint8_t>((u >> 8) & 0xFFu);
			out.at(off + 2) = static_cast<uint8_t>(u & 0xFFu);
		}

		inline void write_i24(std::vector<uint8_t>& out, size_t off, int32_t v, Endianness e = Endianness::Little)
		{
			if (e == Endianness::Little) write_i24_le(out, off, v);
			else write_i24_be(out, off, v);
		}
	}
}
