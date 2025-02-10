/*
Copyright (c) 2023 Grid-based Path Planning Competition and Contributors <https://gppc.search-conference.org/>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef GPPC_MAPLOADER_H
#define GPPC_MAPLOADER_H

#include <vector>
#include <cstring>
#include <string>
#include <istream>
#include <iomanip>
#include <memory_resource>
#include <cassert>
#include "GPPC.h"
#include "Entry.h"

namespace GPPC {

struct Map
{
	uint8_t* bitarray;
	uint32_t width, height;
};
struct Patch
{
	const Map* map;
	::gppc_point pos;
};

inline ::gppc_patch to_gppc_patch(Map m)
{
	return ::gppc_patch{
		m.bitarray,
		static_cast<uint16_t>(m.width), static_cast<uint16_t>(m.height),
		0, 0
	};
}
inline ::gppc_patch to_gppc_patch(Patch m)
{
	assert(m.map != nullptr);
	::gppc_patch p = to_gppc_patch(*m.map);
	p.pos = m.pos;
	return p;
}

inline void map_set(Map& map, int i, bool value)
{
	assert(static_cast<uint32_t>(i) < map.width * map.height);
	uint32_t loc = map.bitarray[(i >> 3)];
	uint32_t shift = i & 7;
	loc &= ~(static_cast<uint32_t>(1u) << shift); // clear loc bit
	loc |= static_cast<uint32_t>(value) << shift; // set loc bit
	map.bitarray[(i >> 3)] = static_cast<uint8_t>(loc);
}
inline bool map_get(const Map& map, int i)
{
	assert(static_cast<uint32_t>(i) < map.width * map.height);
	return ( map.bitarray[(i >> 3)] >> (i & 7) ) & 1;
}
inline bool map_get(const Map& map, int x, int y)
{
	assert(static_cast<uint32_t>(x) < map.width);
	assert(static_cast<uint32_t>(y) < map.height);
	return map_get(map, map.width * y + x);
}
// return allocation size
inline size_t map_bytes(int width, int height)
{
	// count bits
	size_t count = static_cast<size_t>(width) * static_cast<size_t>(height);
	// divide to bytes, round up
	return (count + 7) >> 3;
}

inline bool point_in_bounds(const Map& map, int x, int y) noexcept
{
	if (static_cast<uint32_t>(x) >= map.width)
		return false;
	if (static_cast<uint32_t>(y) >= map.height)
		return false;
	return true;
}

inline bool patch_in_bounds(const Map& map, const Patch& patch) noexcept
{
	if (patch.map == nullptr)
		return false;
	if (patch.pos.x < 0 || patch.map->width > map.width || static_cast<uint32_t>(patch.pos.x) + patch.map->width > map.width)
		return false;
	if (patch.pos.y < 0 || patch.map->height > map.height || static_cast<uint32_t>(patch.pos.y) + patch.map->height > map.height)
		return false;
	return true;
}

inline void apply_patch(Map& map, Patch patch)
{
	assert(patch_in_bounds(map, patch));
	int map_width = map.width;
	int map_i = patch.pos.y * map_width + patch.pos.x;
	int patch_i = 0;
	int patch_width = patch.map->width;
	int patch_height = patch.map->height;
	map_width -= patch_width; // amount to adjust map_i after copy of whole patch row
	for (int y = 0; y < patch_height; ++y) {
		for (int x = 0; x < patch_width; ++x) {
			map_set(map, map_i++, map_get(*patch.map, patch_i++));
		}
		map_i += map_width;
	}
}

bool load_map_data(std::istream& in, Map &map, std::pmr::memory_resource* res = nullptr);

} // namespace GPPC

#endif // GPPC_MAPLOADER_H
