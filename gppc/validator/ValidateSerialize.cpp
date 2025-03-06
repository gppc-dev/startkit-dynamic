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
		throw std::logic_error("Must call Serialize::Setup");
	*m_out << "# Format:\n"
	       << "# query [id] [state_id] [sx] [sy] [gx] [gy]\n"
		   << "# path [complete|incomplete] [path_points_count] [path_x path_y]{path_points_count}\n"
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

void Serialize::AddSubPath(const std::vector<Point> &path, bool incomplete)
{
	ClearState();
	m_connectedPath.clear();
	if (!path.empty()) {
		if (!m_prevPath.empty()) {
			if (m_prevPath.back() != path.front()) {
				m_connectedPath.push_back(m_prevPath.back());
			}
		}
		m_connectedPath.insert(m_connectedPath.end(), path.begin(), path.end());
	}
	[&] () {
		if (!m_connectedPath.empty()) {
			if (m_prevPath.empty() && m_connectedPath.front() != m_current.start) {
				m_currentState = Check{State::StartMismatch, 0};
				return;
			}
			int path_res = ValidatePath(m_map, m_connectedPath);
			if (path_res >= 0) {
				m_currentState = Check{State::InvalidEdge, path_res};
				return;
			}
			if (!incomplete && (m_connectedPath.size() < 2 || m_connectedPath.back() != m_current.goal)) {
				m_currentState = Check{State::GoalMismatch, 0};
				return;
			}
			m_currentState = incomplete ? Check{State::Incomplete, 0} : Check{State::Complete, 0};
		} else {
			m_currentState = Check{State::EmptyPath, 0};
		}
	}();
	if (m_currentState.code == State::EmptyPath) {
		*m_out << "path complete -1\n"
			"eval " << m_currentState << " -1" << std::endl;
	} else {
		m_currentCost = GetPathLength(path);
		// print results
		*m_out << "path " << (incomplete ? "incomplete" : "complete") << ' ' << path.size();
		for (Point p : path) {
			*m_out << ' ' << p.x << ' ' << p.y;
		}
		*m_out << "\neval " << m_currentState << ' ' << std::setprecision(15) << m_currentCost << std::endl;
	}
	m_prevPath = m_currentPath;
}

void Serialize::FinQuery()
{
	*m_out << "final " << m_currentState << ' ' << std::setprecision(15) <<  m_currentCost << std::endl;
	m_prevPath.clear();
	m_currentPath.clear();
}

void Serialize::ClearState()
{
	m_currentState = {};
	m_currentCost = {};
}


// Deserialize
Deserialize::Deserialize() = default;

void Deserialize::Setup(Map map, std::istream& out)
{
	m_map = map;
	m_in = &out;
}

auto Deserialize::ParseCurrentQuery(Error& errc) -> Query
{
	if (m_cmd.cmd != Command::Query) {
		errc = Error::InvalidCommand;
		return {};
	}
	if (m_prevCmd != Command::Final) {
		errc = Error::InvalidCommand;
		return {};
	}
	errc = Error::Ok;
	auto& parse = m_parser;
	parse.str(m_cmd.line);
	parse.clear();
	char buffer[32];
	if (!(parse >> std::setw(32) >> buffer) || std::strcmp(buffer, "query") != 0) {
		errc = Error::InvalidCommand;
		return {};
	}
	Query Q{};
	if (!(parse >> Q.id >> Q.state_id >> Q.start.x >> Q.start.y >> Q.goal.x >> Q.goal.y)) {
		errc = Error::InvalidCommand;
		return {};
	}
	m_query = Q;
	m_fullPath.clear();
	m_subPath.clear();
	return Q;
}

auto Deserialize::ParseCurrentSubPath(Error& errc) -> std::pair<Check, const std::vector<Point>*>
{
	if (m_cmd.cmd != Command::Path) {
		errc = Error::InvalidCommand;
		return {};
	}
	if (m_prevCmd != Command::Query && m_prevCmd != Command::Eval) {
		errc = Error::InvalidCommand;
		return {};
	}
	errc = Error::Ok; // pre-set error to Ok so only return is needed.
	auto& parse = m_parser;
	parse.str(m_cmd.line);
	parse.clear();
	char buffer[32];
	if (!(parse >> std::setw(32) >> buffer) || std::strcmp(buffer, "path") != 0) {
		errc = Error::InvalidCommand;
		return {};
	}
	auto& S = m_currentState;
	if (!(parse >> S) || (S.code != State::Complete && S.code != State::Incomplete)) {
		errc = Error::InvalidState;
		return {};
	}
	size_t count;
	if (!(parse >> count) || count > GPPC_PATCH_LIMIT) {
		errc = Error::InvalidPath;
		return {};
	}
	if (count == 0) {
		m_subPath.clear();
		return {S, &m_subPath};
	}
	m_subPath.resize(count + 1);
	for (size_t i = 1; i <= count; ++i) {
		if (!(parse >> m_subPath[i].x >> m_subPath[i].y)) {
			errc = Error::InvalidPath;
			return {};
		}
	}
	if (m_fullPath.empty() || m_subPath[1] == m_fullPath.back()) {
		m_subPath.erase(m_subPath.begin()); // remove first element
	} else {
		m_subPath[0] = m_fullPath.back(); // copy end of path to start of subpath
	}
	// insert subpath into fullpath, ignoring first element unless fullpath is empty
	m_fullPath.insert(m_fullPath.end(), m_subPath.begin() + (m_fullPath.empty() ? 0 : 1), m_subPath.end());
	return {S, &m_subPath};
}

auto Deserialize::ParseCurrentSubPathEval(Error& errc) -> std::pair<Check, double>
{
	if (m_cmd.cmd != Command::Eval) {
		errc = Error::InvalidCommand;
		return {};
	}
	if (m_prevCmd != Command::Path) {
		errc = Error::InvalidCommand;
		return {};
	}
	errc = Error::Ok; // pre-set error to Ok so only return is needed.
	auto& parse = m_parser;
	parse.str(m_cmd.line);
	parse.clear();
	char buffer[32];
	if (!(parse >> std::setw(32) >> buffer) || std::strcmp(buffer, "eval") != 0) {
		errc = Error::InvalidCommand;
		return {};
	}
	Check user_valid;
	double user_dist;
	if (!(parse >> user_valid >> user_dist)) {
		errc = Error::InvalidCommand;
		return {};
	}
	auto res = ValidatePath(m_subPath);
	return res;
}

auto Deserialize::ParseCurrentQueryFinal(Error& errc) -> std::pair<Check, double>
{
	if (m_cmd.cmd != Command::Final) {
		errc = Error::InvalidCommand;
		return {};
	}
	if (m_prevCmd != Command::Eval) {
		errc = Error::InvalidCommand;
		return {};
	}
	errc = Error::Ok; // pre-set error to Ok so only return is needed.
	auto& parse = m_parser;
	parse.str(m_cmd.line);
	parse.clear();
	char buffer[32];
	if (!(parse >> std::setw(32) >> buffer) || std::strcmp(buffer, "final") != 0) {
		errc = Error::InvalidCommand;
		return {};
	}
	Check user_valid;
	double user_dist;
	if (!(parse >> user_valid >> user_dist)) {
		errc = Error::InvalidCommand;
		return {};
	}
	auto res = ValidatePath(m_subPath);
	if (!m_subPath.empty()) {
		if (m_subPath.front() != m_query.start) {
			res.first = Check{State::StartMismatch, 0};
		} else if (m_subPath.back() != m_query.goal) {
			res.first = Check{State::GoalMismatch, 0};
		}
	}
	return res;
}

auto Deserialize::ValidatePath(const std::vector<Point>& path) -> std::pair<Check, double>
{
	std::pair<Check, double> res{Check{}, -1.0};
	if (path.empty()) {
		res.first = Check{State::EmptyPath, 0};
		return res;
	}
	int path_res = validate::ValidatePath(m_map, path);
	if (path_res >= 0) {
		res.first = Check{State::InvalidEdge, path_res};
		return res;
	}
	res.first = Check{State::Complete, 0};
	long double sum = 0;
	for (int i = 1, ie = static_cast<int>(path.size()); i < ie; ++i) {
		int64_t dx = static_cast<int64_t>(path[i].x) - static_cast<int64_t>(path[i-1].x);
		int64_t dy = static_cast<int64_t>(path[i].y) - static_cast<int64_t>(path[i-1].y);
		double dadd = static_cast<double>(dx * dx + dy * dy);
		sum += std::sqrt(dadd);
	}
	res.second = static_cast<double>(sum);
	return res;
}

auto Deserialize::NextCommand() -> const CommandLine&
{
	m_prevCmd = m_cmd.cmd;
	while (*m_in)
	{
		*m_in >> std::ws;
		// check for end-of-file
		if (m_in->eof()) {
			m_cmd.eof();
			return m_cmd;
		}
		std::getline(*m_in, m_cmd.line);
		// check for comment
		if (m_cmd.line.compare(0, 1, "#", 1) == 0) {
			continue;
		}
		m_cmd.update_cmd();
		return m_cmd;
	}
	m_cmd.inv();
	return m_cmd;
}

void Deserialize::CommandLine::eof()
{
	cmd = Command::Eof;
	line.clear();
}
void Deserialize::CommandLine::inv()
{
	cmd = Command::Invalid;
}
void Deserialize::CommandLine::update_cmd()
{
	using namespace std::string_view_literals;
	if (line.compare(0, 6, "query ", 6) == 0)
		cmd = Command::Query;
	else if (line.compare(0, 5, "path ", 5) == 0)
		cmd = Command::Path;
	else if (line.compare(0, 5, "eval ", 5) == 0)
		cmd = Command::Eval;
	else if (line.compare(0, 6, "final ", 6) == 0)
		cmd = Command::Final;
	else
		cmd = Command::Invalid;
}

} // namespace GPPC::validate

std::ostream &operator<<(std::ostream &out, GPPC::validate::Check check)
{
	using GPPC::validate::State;
	switch (check.code) {
	case State::Incomplete:
		out << "incomplete";
		break;
	case State::StartMismatch:
		out << "start-mismatch";
		break;
	case State::GoalMismatch:
		out << "goal-mismatch";
		break;
	case State::InvalidEdge:
		out << "invalid-" << check.value;
		break;
	case State::Complete:
		out << "complete";
		break;
	case State::EmptyPath:
		out << "empty-path";
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
	if (token == "incomplete"sv) {
		check = {State::Incomplete, 0};
	} else if (token == "start-mismatch"sv) {
		check = {State::StartMismatch, 0};
	} else if (token == "goal-mismatch"sv) {
		check = {State::GoalMismatch, 0};
	} else if (token == "complete"sv) {
		check = {State::Complete, 0};
	} else if (token.substr(0, "invalid-"sv.length()) == "invalid-"sv) {
		// check for prefix
		token = token.substr("invalid-"sv.length());
		if (auto res = std::from_chars(token.begin(), token.end(), check.value);
			res.ptr == token.end() && res.ec == std::errc{}) {
			check.code = State::InvalidEdge;
		} else {
			out.setstate(std::ios::failbit);
		}
	} else {
		out.setstate(std::ios::failbit);
	}
	return out;
}
