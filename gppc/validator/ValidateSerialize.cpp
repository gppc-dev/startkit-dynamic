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

#include "ValidateSerialize.hpp"
#include <iomanip>
#include <string_view>
#include <charconv>
#include <algorithm>

namespace GPPC::validate {

Serialize::Serialize() : m_out{}, m_current{}, m_currentState{}, m_currentCost{}
{ }

void Serialize::Setup(Map map, std::ostream& out)
{
	m_map = map;
	m_out = &out;
}

void Serialize::PrintHeader()
{
	if (!*this)
		throw std::logic_error("Must call Serialize::SetOutput");
	*m_out << "# Format:\n"
	       << "# query [id] [state_id] [sx] [sy] [gx] [gy]\n"
		   << "# path [complete|incomplete] [path_points_count] [path_x path_y]...\n"
		   << "# eval [check] [distance]\n"
		   << "# final [check] [distance]" << std::endl;
}

void Serialize::AddQuery(Query Q)
{
	m_current = Q;
	*m_out << "query " << Q.id << ' ' << Q.state_id << ' '
	       << Q.start.x << ' ' << Q.start.y << ' '
		   << Q.goal.x << ' ' << Q.goal.y << std::endl;
	m_prevPath.clear();
	ClearState();
}

void Serialize::AddPath(const std::vector<Point> &path, bool incomplete)
{
	ClearState();
	[&] () {
		int prefix_mismatch = std::mismatch(m_prevPath.begin(), m_prevPath.end(), path.begin(), path.end()).first - m_prevPath.begin();
		if (prefix_mismatch != static_cast<int>(m_prevPath.size())) {
			m_currentState = Check{State::PrefixMismatch, prefix_mismatch};
			return;
		}
		if (!path.empty()) {
			if (path.front() != m_current.start) {
				m_currentState = Check{State::StartMismatch, 0};
				return;
			}
			int path_res = ValidatePath(m_map, path);
			if (path_res >= 0) {
				m_currentState = Check{State::InvalidEdge, path_res};
				return;
			}
			if (!incomplete && (path.size() < 2 || path.back() != m_current.goal)) {
				m_currentState = Check{State::GoalMismatch, 0};
				return;
			}
		}
		m_currentState = incomplete ? Check{State::Incomplete, 0} : Check{State::Complete, 0};
	}();
	m_currentCost = GetPathLength(path);
	// print results
	*m_out << "path " << (incomplete ? "incomplete" : "complete") << ' ' << path.size();
	for (Point p : path) {
		*m_out << ' ' << p.x << ' ' << p.y;
	}
	*m_out << "\neval " << m_currentState << ' ' << m_currentCost << std::endl;
}

void Serialize::FinQuery()
{
	*m_out << "final " << m_currentState << ' ' << m_currentCost << std::endl;
	m_prevPath.clear();
	m_currentPath.clear();
}

void Serialize::ClearState()
{
	m_currentState = {};
	m_currentCost = {};
}

} // namespace GPPC::validate

std::ostream &operator<<(std::ostream &out, GPPC::validate::Check check)
{
	using GPPC::validate::State;
	switch (check.code) {
	case State::Begin:
		out << "begin";
		break;
	case State::Incomplete:
		out << "incomplete";
		break;
	case State::StartMismatch:
		out << "start-mismatch";
		break;
	case State::GoalMismatch:
		out << "goal-mismatch";
		break;
	case State::PrefixMismatch:
		out << "prefix-" << check.value;
		break;
	case State::InvalidEdge:
		out << "invalid-" << check.value;
		break;
	case State::Complete:
		out << "complete";
		break;
	default:
		out.setstate(std::ios::failbit);
	}
	return out;
}

std::istream &operator>>(std::istream &out, GPPC::validate::Check &check)
{
	using GPPC::validate::State;
	using namespace std::string_view_literals;
	char buffer[32];
	out >> std::setw(32) >> buffer;
	std::string_view token(buffer);
	if (token == "begin"sv) {
		check = {State::Begin, 0};
	} else if (token == "incomplete"sv) {
		check = {State::Incomplete, 0};
	} else if (token == "start-mismatch"sv) {
		check = {State::StartMismatch, 0};
	} else if (token == "goal-mismatch"sv) {
		check = {State::GoalMismatch, 0};
	} else if (token == "complete"sv) {
		check = {State::Complete, 0};
	} else {
		// check for prefix
		bool prefix = token.substr(0, "prefix-"sv.length()) == "prefix-"sv;
		bool invalid = !prefix && token.substr(0, "invalid-"sv.length()) == "invalid-"sv;
		if (!prefix || !invalid) {
			out.setstate(std::ios::failbit);
		} else {
			token = prefix ? token.substr("prefix-"sv.length()) : token.substr("invalid-"sv.length());
			if (auto res = std::from_chars(token.begin(), token.end(), check.value);
				res.ptr == token.end() && res.ec == std::errc{}) {
					check.code = prefix ? State::PrefixMismatch : State::InvalidEdge;
			} else {
				out.setstate(std::ios::failbit);
			}
		}
	}
	return out;
}
