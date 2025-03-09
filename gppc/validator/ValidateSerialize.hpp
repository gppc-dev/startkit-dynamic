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
#include <sstream>
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
	Incomplete,
	StartMismatch, /// start incorrect
	GoalMismatch, /// goal incorrect
	InvalidEdge, /// invlid path segment
	Complete,
	EmptyPath
};
struct Check {
	State code;
	int value; /// code == InvalidEdge: segment no; code == InvalidPrefix: mismatch pos
};
enum class Command : uint8_t {
	Invalid,
	Eof,
	Query,
	Path,
	Eval,
	Final,
};


class Serialize
{
public:
	Serialize();
	/**
	 * @param map Dynamic map state. Must be kept inscope and up to date outside of this class.
	 * @param out Where to print output to.
	 */
	void Setup(Map map, std::ostream& out);
	void PrintHeader();
	void AddQuery(Query Q);
	template <typename PathContainer>
	void AddSubPath(const PathContainer& path, bool incomplete)
	{
		m_currentPath.clear();
		for (const auto& p : path) {
			m_currentPath.push_back(Point{static_cast<int>(p.x), static_cast<int>(p.y)});
		}
		AddSubPath(m_currentPath, incomplete);
	}
	/**
	 * @param incomplete Path is incomplete.
	 */
	void AddSubPath(const std::vector<Point>& path, bool incomplete);
	void FinQuery();

	void ClearState();

	operator bool() const noexcept { return m_out != nullptr; }

private:
	Map m_map;
	std::ostream* m_out;
	Query m_current;
	std::vector<Point> m_prevPath;
	std::vector<Point> m_currentPath;
	std::vector<Point> m_connectedPath;
	Check m_currentState;
	long double m_currentCost;
};


class Deserialize
{
public:
	enum class Error : uint8_t
	{
		Ok = 0,
		Eof,
		InvalidCommand,
		InvalidState,
		InvalidPath,
		InvalidStart,
		InvalidTarget
	};
	struct CommandLine
	{
		Command cmd;
		std::string line;

		void eof();
		void inv();
		void update_cmd();
	};
	Deserialize();
	/**
	 * @param map Dynamic map state. Must be kept inscope and up to date outside of this class.
	 * @param out Where to print output to.
	 */
	void Setup(Map map, std::istream& in);
	
	const CommandLine& GetNextCommand()
	{
		return NextCommand();
	}
	const CommandLine& GetCurrentCommand() noexcept
	{
		return m_cmd;
	}

	/// @brief Parse current command as a query.
	/// @param errc Sets errc to non-zero on a parsing error.
	/// @return The parsed query. This must ALWAYS match the scenario.
	Query ParseCurrentQuery(Error& errc);
	/// @brief Parse current command as a path (sub).
	/// @param errc Sets errc to non-zero on a parsing error.
	/// @return (Sub)Path state (complete/incomplete) and user provided path.
	std::pair<Check, const std::vector<Point>*> ParseCurrentSubPath(Error& errc);
	/// @brief Parse current command as eval. Expected after a (Sub)Path
	/// @param errc Sets errc to non-zero on a parsing error.
	/// @return (Sub)Path state (any state) and length of (Sub)Path.
	std::pair<Check, double> ParseCurrentSubPathEval(Error& errc);
	/// @brief Parse current command as final, end of Query block.
	/// @param errc Sets errc to non-zero on a parsing error.
	/// @return State and the full path.
	std::pair<Check, double> ParseCurrentQueryFinal(Error& errc);

	std::pair<Check, double> ValidatePath(const std::vector<Point>& path);

	operator bool() const noexcept { return m_in != nullptr; }

protected:
	/**
	 * Gets the next validation command.  Will skip empty lines and lines starting with #.
	 * Command.cmd == Command::Eof is the end of file, thus no more validation.
	 */
	const CommandLine& NextCommand();

private:
	Map m_map = {};
	std::istream* m_in = nullptr;
	std::istringstream m_parser;
	Command m_prevCmd = Command::Invalid;
	CommandLine m_cmd = {Command::Final, {}};
	std::vector<Point> m_subPath;
	std::vector<Point> m_fullPath;
	Query m_query = {};
	Check m_currentState = {};
};

} // namespace GPPC::validate

std::ostream& operator<<(std::ostream& out, GPPC::validate::Check check);
std::istream& operator>>(std::istream& out, GPPC::validate::Check& check);

#endif // GPPC_VALIDATESERIALIZE_HPP
