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

#include "MapLoader.h"

namespace GPPC {

bool load_map_data(std::istream &in, Map &map, std::pmr::memory_resource *res)
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
	// no need to call res->deallocate
	map.bitarray = static_cast<uint8_t*>(res->allocate(map_bytes(map.width, map.height), 1));
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

} // namespace GPPC
