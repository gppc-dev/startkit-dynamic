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

#ifndef GPPC_VALIDATESERIALIZE_HPP
#define GPPC_VALIDATESERIALIZE_HPP

#include "ValidatePath.hpp"
#include <iostream>
#include <ScenarioLoader.h>

namespace GPPC::validate {

struct Query {
	int32_t id;
	int32_t state_id;
	Point start;
	Point goal;
	double cost;
};

enum class State : uint8_t {
	Begin,
	Incomplete,
	StartMismatch, /// start incorrect
	GoalMismatch, /// goal incorrect
	PrefixMismatch, /// path 
	InvalidEdge, /// invlid path segment
	Complete
};
struct Check {
	State code;
	int value; /// code == InvalidEdge: segment no; code == InvalidPrefix: mismatch pos
};


class Serialize
{
public:
	Serialize();
	void Setup(Map map, std::ostream& out);
	void PrintHeader();
	void AddQuery(Query Q);
	template <typename PathContainer>
	void AddPath(const PathContainer& path, bool incomplete)
	{
		m_currentPath.clear();
		for (const auto& p : path) {
			m_currentPath.push_back(Point{static_cast<int>(p.x), static_cast<int>(p.y)});
		}
		AddPath(m_currentPath, incomplete);
	}
	/**
	 * @param incomplete Path is incomplete.
	 */
	void AddPath(const std::vector<Point>& path, bool incomplete);
	void FinQuery();

	void ClearState();

	operator bool() const noexcept { return m_out != nullptr; }

private:
	Map m_map;
	std::ostream* m_out;
	Query m_current;
	std::vector<Point> m_prevPath;
	std::vector<Point> m_currentPath;
	Check m_currentState;
	double m_currentCost;
};

} // namespace GPPC::validate

std::ostream& operator<<(std::ostream& out, GPPC::validate::Check check);
std::istream& operator>>(std::istream& out, GPPC::validate::Check& check);

#endif // GPPC_VALIDATESERIALIZE_HPP
