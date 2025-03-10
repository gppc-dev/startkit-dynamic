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

#ifndef GPPC_VALIDATEPATH_HPP
#define GPPC_VALIDATEPATH_HPP

#include <vector>
#include <cstddef>
#include <cassert>
#include <cstdlib>
#include <MapLoader.h>
#include <cmath>

namespace GPPC::validate {

using std::size_t;

struct alignas(size_t)
Point
{
	int32_t x;
	int32_t y;
};

inline Point operator+(Point lhs, Point rhs) noexcept { return Point{lhs.x + rhs.x, lhs.y + rhs.y}; }
inline Point operator-(Point lhs, Point rhs) noexcept { return Point{lhs.x - rhs.x, lhs.y - rhs.y}; }

inline bool operator==(Point lhs, Point rhs) noexcept { return lhs.x == rhs.x && lhs.y == rhs.y; }
inline bool operator!=(Point lhs, Point rhs) noexcept { return lhs.x != rhs.x || lhs.y != rhs.y; }


inline long double EuclideanDist(Point a, Point b) {
	int64_t dx = std::abs(static_cast<int64_t>(b.x) - a.x);
	int64_t dy = std::abs(static_cast<int64_t>(b.y) - a.y);
	long double res = std::sqrt(static_cast<long double>(dx * dx) + static_cast<long double>(dy * dy));
	return res;
}

inline long double GetPathLength(const std::vector<Point>& path)
{
	if (path.empty())
		return -1;
	long double len = 0;
	for (int x = 0, xe = (int)path.size()-1; x < xe; x++)
		len += EuclideanDist(path[x], path[x+1]);
	return len;
}

class PathValidator
{
public:
	PathValidator(Map map)
		: m_map(map)
	{ }

	bool get(Point u) const noexcept
	{
		return get(u.x, u.y);
	}
	bool get(int x, int y) const noexcept
	{
		return map_get(m_map, y * m_map.width + x);
	}

	bool validPoint(Point u) const noexcept
	{
		return static_cast<size_t>(u.x) < static_cast<size_t>(m_map.width) &&
			static_cast<size_t>(u.y) < static_cast<size_t>(m_map.height) &&
			get(u);
	}

	bool validEdge(Point u, Point v) const noexcept
	{
		Point uv = v - u;
		Point diff{};
		bool cardinals{};
		if (uv.x == 0) {
			cardinals = true;
			if (uv.y == 0) // u = v
				return true;
			// hori line
			diff = Point{0, uv.y > 0 ? 1 : -1};
		} else if (uv.y == 0) {
			cardinals = true;
			// vert line
			diff = Point{uv.x > 0 ? 1 : -1, 0};
		} else { // non-cardinal line
			cardinals = false;
			if (std::abs(uv.x) != std::abs(uv.y))
				return false; // non-ordinal
			// ordinal line
			diff = Point{uv.x > 0 ? 1 : -1, uv.y > 0 ? 1 : -1};
		}
		// check cells are clear in grid
		if (cardinals)
			return validCardinal(u, v, diff);
		else
			return validOrdinal(u, v, diff);
	}

private:
	bool validCardinal(Point u, Point v, Point diff) const noexcept
	{
		for (Point x = u; x != v; x = x + diff) {
			if (!get(x))
				return false;
		}
		return true;
	}
	bool validOrdinal(Point u, Point v, Point diff) const noexcept
	{
		// check every 2x2 square along u-v
		for (Point x = u; x != v; x = x + diff) {
			if (!get(x) || !get(x.x + diff.x, x.y) || !get(x.x, x.y + diff.y))
				return false;
		}
		if (!get(v)) // seperate from loop to prevent 2x2 squares past v getting checked
			return false;
		return true;
	}

private:
	Map m_map;
};

template <typename PathContainer>
inline int ValidatePath(const Map& map, const PathContainer& thePath)
{
	size_t S = static_cast<size_t>(thePath.size());
	if (S == 0)
		return -1;
	if (S == 1)
		return 0;
	PathValidator validator(map);
	// check each point in path
	for (size_t i = 0; i < S; ++i) {
		Point u{static_cast<int>(thePath[i].x), static_cast<int>(thePath[i].y)};
		if (!validator.validPoint(u))
			return static_cast<int>(i);
	}
	// check each segment
	for (size_t i = 0; i < S-1; ++i) {
		Point u{static_cast<int>(thePath[i].x), static_cast<int>(thePath[i].y)};
		Point v{static_cast<int>(thePath[i+1].x), static_cast<int>(thePath[i+1].y)};
		if (!validator.validEdge(u, v))
			return static_cast<int>(i);
	}
	return -1;
}

} // namespace GPPC::validate

#endif // GPPC_VALIDATEPATH_HPP
