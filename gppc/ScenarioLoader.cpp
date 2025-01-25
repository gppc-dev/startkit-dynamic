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

#include "ScenarioLoader.h"
#include <filesystem>
#include <iomanip>
#include <fstream>

namespace GPPC {

/** 
 * Loads the experiments from the scenario file. 
 */
ScenarioLoader::ScenarioLoader() : patchRes(4 * 1024 * 1024), width{}, height{}, patchCommands{}, queryCommands{}
{ }

void ScenarioLoader::clear()
{
	if (*this) {
		queryCost.clear();
		commands.clear();
		patchGrid.clear();
		patchRes.release();
		width = height = 0;
		patchCommands = 0;
		queryCommands = 0;
	}
}

bool ScenarioLoader::load(std::istream& in, const std::filesystem::path& scenFilename)
{
	clear();
	std::string tmp;
	int value;
	std::filesystem::path patchFile;
	// read map header
	if (!(in >> tmp >> value))
		return false;
	if (tmp != "version" || value != 2)
		return false;
	if (!(in >> tmp >> height))
		return false;
	if (tmp != "height" || height < 1 || height > static_cast<int>(GPPC_HARD_MAP_LIMIT))
		return false;
	if (!(in >> tmp >> width))
		return false;
	if (tmp != "width" || width < 1 || width > static_cast<int>(GPPC_HARD_MAP_LIMIT))
		return false;
	// read cost type and position
	int cost_pos = -1;
	int costs;
	if (!(in >> tmp >> costs))
		return false;
	for (int i = 0; i < costs; ++i) {
		if (!(in >> tmp))
			return false;
		if (tmp == "octile") {
			if (cost_pos >= 0) // duplicate octile, ill-formed
				return false;
			cost_pos = i;
		}
	}
	if (cost_pos < 0)
		return false; // missing octile cost
	// read patch filename
	if (!(in >> tmp))
		return false;
	if (tmp != "patch")
		return false;
	if (!(in >> tmp))
		return false;
	patchFile = tmp;
	if (patchFile.is_relative()) {
		// try get relative path
		std::filesystem::path alteredPath;
		if (scenFilename.empty()) {
			alteredPath = std::filesystem::current_path();
		} else {
			alteredPath = scenFilename.parent_path();
		}
		alteredPath /= patchFile;
		patchFile = std::move(alteredPath);
	}
	if (!load_map(patchFile))
		return false; // patch file does not exists

	// read commands
	patchCommands = 0;
	queryCommands = 0;
	commands.reserve(1024);
	queryCost.reserve(1024);

	Map bounds_check{}; // used for checking bounds
	bounds_check.width = width;
	bounds_check.height = height;

	if (!(in >> tmp))
		return false;
	if (tmp != "commands")
		return false;

	while ((in >> tmp)) {
		if (tmp == "P") {
			// patch
			Command cmd = {Command::Type::patch, 0, { .patch = {} }};
			in >> cmd.bucket >> cmd.cmd.patch.id >> cmd.cmd.patch.pos.x >> cmd.cmd.patch.pos.y;
			// check is valid patch
			if (int pid = cmd.cmd.patch.id; pid < 0 && pid >= static_cast<int>(patchGrid.size()))
				return false;
			const Map& grid = patchGrid[cmd.cmd.patch.id];
			if (!patch_in_bounds(bounds_check, Patch{&grid, cmd.cmd.patch.pos.x, cmd.cmd.patch.pos.y}))
				return false;
			commands.push_back(cmd);
			patchCommands += 1;
		} else if (tmp == "Q") {
			// query
			Command cmd = {Command::Type::query, 0, { .query = {} }};
			in >> cmd.bucket >> cmd.cmd.query.start.x >> cmd.cmd.query.start.y >> cmd.cmd.query.goal.x >> cmd.cmd.query.goal.y;
			if (!point_in_bounds(bounds_check, cmd.cmd.query.start.x, cmd.cmd.query.start.y))
				return false;
			if (!point_in_bounds(bounds_check, cmd.cmd.query.goal.x, cmd.cmd.query.goal.y))
				return false;
			// read all costs and select the one we want
			double query_dist{};
			for (int i = 0; i < costs; ++i) {
				double dist;
				in >> dist;
				if (i == cost_pos)
					query_dist = dist;
			}
			commands.push_back(cmd);
			queryCost.push_back(query_dist);
			queryCommands += 1;
		} else {
			return false; // unknown command
		}
	}
	if (!in.eof())
		return false; // bad read
	return true;
}
bool ScenarioLoader::load(const std::filesystem::path& filename)
{
	std::ifstream file(filename);
	if (!file)
		return false;
	return load(file, filename);
}

bool ScenarioLoader::load_map(const std::filesystem::path& filename)
{
	std::ifstream in(filename);
	char buffer1[12];
	char buffer2[12];
	using std::setw;
	if (!(in >> setw(12) >> buffer1 >> setw(12) >> buffer2))
		return false;
	if (std::strcmp(buffer1, "type") != 0 || strcmp(buffer2, "patch") != 0)
		return false;
	int patch_count;
	if (!(in >> setw(12) >> buffer1 >> patch_count))
		return false;
	if (std::strcmp(buffer1, "patches") != 0 || patch_count < 0 || patch_count > static_cast<int>(GPPC_PATCH_LIMIT))
		return false;
	patchGrid.resize(patch_count);
	Map bounds_check;
	bounds_check.width = width;
	bounds_check.height = height;
	for (int patch_i = 0; patch_i < patch_count; ++patch_i) {
		int patch_id;
		if (!(in >> setw(12) >> buffer1 >> patch_id))
			return false;
		if (std::strcmp(buffer1, "patch") != 0 || patch_id != patch_i)
			return false;
		if (!load_map_data(in, patchGrid[patch_i], &patchRes))
			return false;
		if (!patch_in_bounds(bounds_check, Patch{&patchGrid[patch_i], 0, 0}))
			return false;
	}
	return true;
}

ScenarioRunner::ScenarioRunner() : scenario{},
	activeMap{}, scenarioAt{}, commandAt{}, commandCount{}
{ }

void ScenarioRunner::linkScen(const ScenarioLoader& scen)
{
	scenario = &scen;
	activeMap.width = scen.getWidth();
	activeMap.height = scen.getHeight();
	size_t bytes_size = map_bytes(activeMap.width, activeMap.height);
	activeMapData = std::make_unique<uint8_t[]>(bytes_size);
	std::fill_n(activeMapData.get(), bytes_size, static_cast<uint8_t>(~0u));
	activeMap.bitarray = activeMapData.get();
	appliedPatch.clear();
	scenarioAt = -1;
	commandAt = -1;
	commandCount = scen.getCommands().size();
}

int ScenarioRunner::nextQuery()
{
	appliedPatch.clear();
	int command_i = commandAt + 1;
	assert(command_i >= 0);
	int command_n = scenario->getCommands().size();
	int command_done = 0; // how much to adjust commandAt at the end
	for ( ; command_i < command_n; ++command_i) {
		command_done += 1;
		Command cmd = scenario->getCommands()[command_i];
		if (cmd.type == Command::Type::query)
			break; // reached next query
		assert(cmd.type == Command::Type::patch); // must be patch
		// apply patch
		const Map& patch_grid = scenario->getPatches()[cmd.cmd.patch.id];
		Patch patch{&patch_grid, cmd.cmd.patch.pos.x, cmd.cmd.patch.pos.y};
		apply_patch(activeMap, patch);
		appliedPatch.push_back(to_gppc_patch(patch));
	}
	// reached past last command or query
	commandAt += command_done;
	if (command_i >= command_n) {
		return -1;
	}
	scenarioAt += 1;
	return appliedPatch.size();
}

Query ScenarioRunner::getCurrentQuery() const
{
	Command cmd = scenario->getCommands().at(commandAt);
	assert(cmd.type == Command::Type::query); // must be on query
	if (cmd.type != Command::Type::query) {
		throw std::logic_error("ScenarioRunner::getCurrentQuery must be on Query");
	}
	Query query;
	query.query_id = scenarioAt;
	query.bucket = cmd.bucket;
	query.start = cmd.cmd.query.start;
	query.goal = cmd.cmd.query.goal;
	query.cost = scenario->getQueryCost().at(scenarioAt);
	return query;
}

} // namespace GPPC
