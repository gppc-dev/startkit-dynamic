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

constexpr size_t GPPC_HARD_MAP_LIMIT = 8000;

struct Map
{
	std::pmr::vector<bool> bitmap;
	int width, height;
};
struct Patch
{
	const Map* map;
	int x, y;
};

inline void map_set(Map& map, int i, bool value)
{
	assert(0 <= i && i < map.width * map.height);
	map.bitmap[i] = value;
}
inline bool map_get(const Map& map, int i)
{
	assert(0 <= i && i < map.width * map.height);
	return map.bitmap[i];
}
inline bool map_get(const Map& map, int x, int y)
{
	assert(0 <= x && x < map.width);
	assert(0 <= y && y < map.height);
	return map.bitmap[map.width * y + x];
}

inline bool point_in_bounds(const Map& map, int x, int y) noexcept
{
	if (x < 0 || x >= map.width)
		return false;
	if (y < 0 || y >= map.height)
		return false;
	return true;
}

inline bool patch_in_bounds(const Map& map, const Patch& patch) noexcept
{
	if (patch.map == nullptr)
		return false;
	if (patch.x < 0 || patch.x + patch.map->width > map.width)
		return false;
	if (patch.y < 0 || patch.y + patch.map->height > map.height)
		return false;
	return true;
}

inline void apply_patch(Map& map, Patch patch)
{
	assert(patch_in_bounds(map, patch));
	int map_width = map.width;
	int map_i = patch.y * map_width + patch.x;
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

inline bool load_map_data(std::istream& in, Map &map, std::pmr::memory_resource* res = nullptr)
{
	char buffer[static_cast<int>(GPPC_HARD_MAP_LIMIT) + 10];
	int width, height;
	// read header
	if (!(in >> std::setw(8) >> buffer >> height))
		return false;
	if (std::strcmp(buffer, "height") != 0 || height < 1 || height > static_cast<int>(GPPC_HARD_MAP_LIMIT))
		return false;
	map.height = height;
	if (!(in >> std::setw(8) >> buffer >> width))
		return false;
	if (std::strcmp(buffer, "width") != 0 || width < 1 || width > static_cast<int>(GPPC_HARD_MAP_LIMIT))
		return false;
	map.width = width;
	int cells = map.height * map.width;
	map.bitmap = std::pmr::vector<bool>(cells, false, res);
	// read body
	if (!(in >> std::setw(8) >> buffer))
		return false;
	if (std::strcmp(buffer, "map") != 0)
		return false;
	for (int y = 0, i = 0; y < height; ++y) {
		// read row
		in >> std::ws;
		if (!in.read(buffer, width) || in.gcount() != width)
			return false;
		if (auto c = in.peek(); c != std::istream::traits_type::eof() && !std::isspace(static_cast<unsigned char>(c)))
			return false; // not delimited by whitespace or eof
		for (int x = 0; x < width; ++x, ++i) {
			switch (buffer[x]) {
			case '.':
			case 'G':
			case 'S':
				map_set(map, i, true);
				break;
			case '@':
			case 'O':
			case 'T':
			case 'W':
				break;
			default: // unknown character
				return false;
			}
		}
	}
	return true;
}

#endif // GPPC_MAPLOADER_H
