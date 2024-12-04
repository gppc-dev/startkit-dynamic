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

#ifndef GPPC_SCENARIOLOADER_H
#define GPPC_SCENARIOLOADER_H

#include <vector>
#include <cstring>
#include <string>
#include <filesystem>
#include <memory_resource>
#include <cassert>
#include <cstdint>
#include "MapLoader.h"

constexpr size_t GPPC_PATCH_LIMIT = 100'000'000;

/** 
 * Experiments stored by the ScenarioLoader class. 
 */
class ScenarioLoader;

struct Command
{
	enum class Type : uint8_t
	{
		patch,
		query
	};
	Type type;
	uint16_t bucket;
	union {
		struct {
			uint32_t id;
			uint16_t x;
			uint16_t y;
		} patch;
		struct {
			uint16_t sx;
			uint16_t sy;
			uint16_t gx;
			uint16_t gy;
			// double dist; stored out of command
		} query;
	} cmd;
};

struct Query
{
	int32_t query_id;
	int32_t bucket;
	uint16_t sx;
	uint16_t sy;
	uint16_t gx;
	uint16_t gy;
	double cost;
};

/** A class which loads and stores scenarios from files.  
 * Versions currently handled: 0.0 and 1.0 (includes scale). 
 */

class ScenarioLoader
{
public:
	ScenarioLoader();
	/**
	 * @param filename The in stream filename, if blank, assume cwd for matching patch name
	 */
	bool load(std::istream& in, const std::filesystem::path& filename = {});
	bool load(const std::filesystem::path& filename);

	void clear();

	operator bool() const noexcept { return width != 0; }

	const auto& getPatches() const noexcept { return patchGrid; }
	const auto& getCommands() const noexcept { return commands; }
	const auto& getQueryCost() const noexcept { return queryCost; }
	int getWidth() const noexcept { return width; }
	int getHeight() const noexcept { return height; }
	int getPatchCommands() const noexcept { return patchCommands; }
	int getQueryCommands() const noexcept { return queryCommands; }

private:
	bool load_map(const std::filesystem::path& filename);

private:
	std::pmr::monotonic_buffer_resource patchRes;
	std::vector<Map> patchGrid;
	std::vector<Command> commands;
	std::vector<double> queryCost;
	int width;
	int height;
	int patchCommands;
	int queryCommands;
};

class ScenarioRunner
{
public:
	ScenarioRunner();
	void linkScen(const ScenarioLoader& scen);
	/**
	 * Reach next query.
	 * @return number of patches applied (0 or more), -1 for end of benchmark
	 */
	int nextQuery();
	Query getCurrentQuery() const;
	bool done() { return commandAt >= commandCount; }
	const Map& getActiveMap() const noexcept { return activeMap; }
	const auto& getAppliedPatches() const noexcept { return appliedPatch; }

private:
	const ScenarioLoader* scenario;
	Map activeMap;
	std::vector<Patch> appliedPatch;
	int scenarioAt;
	int commandAt;
	int commandCount;
};

#endif // GPPC_SCENARIOLOADER_H
