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

/** 
 * Loads the experiments from the scenario file. 
 */
ScenarioLoader::ScenarioLoader() : patchRes(4 * 1024 * 1024), width{}, height{}, patch_commands{}, query_commands{}
{ }

void ScenarioLoader::clear()
{
	if (*this) {
		queryCost.clear();
		commands.clear();
		patchGrid.clear();
		patchRes.release();
		width = height = 0;
		patch_commands = 0;
		query_commands = 0;
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
	if (tmp != "height" || height < 1 || height > GPPC_HARD_MAP_LIMIT)
		return false;
	if (!(in >> tmp >> width))
		return false;
	if (tmp != "width" || width < 1 || width > GPPC_HARD_MAP_LIMIT)
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
	if (tmp != "patches")
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
	patch_commands = 0;
	query_commands = 0;
	commands.reserve(1024);
	queryCost.reserve(1024);

	Map bounds_check{}; // used for checking bounds
	bounds_check.width = width;
	bounds_check.height = height;

	while ((in >> tmp)) {
		if (tmp == "P") {
			// patch
			Command cmd = {Command::Type::patch, 0, { .patch = {} }};
			in >> cmd.bucket >> cmd.cmd.patch.id >> cmd.cmd.patch.x >> cmd.cmd.patch.y;
			// check is valid patch
			if (int pid = cmd.cmd.patch.id; pid < 0 && pid >= patchGrid.size())
				return false;
			const Map& grid = patchGrid[cmd.cmd.patch.id];
			if (!patch_in_bounds(bounds_check, Patch{&grid, cmd.cmd.patch.x, cmd.cmd.patch.y}))
				return false;
			commands.push_back(cmd);
		} else if (tmp == "Q") {
			// query
			Command cmd = {Command::Type::query, 0, { .query = {} }};
			in >> cmd.bucket >> cmd.cmd.query.sx >> cmd.cmd.query.sy >> cmd.cmd.query.gx >> cmd.cmd.query.gy;
			if (!point_in_bounds(bounds_check, cmd.cmd.query.sx, cmd.cmd.query.sy))
				return false;
			if (!point_in_bounds(bounds_check, cmd.cmd.query.gx, cmd.cmd.query.gy))
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
	if (std::strcmp(buffer1, "patches") != 0 || patch_count < 0 || patch_count > GPPC_PATCH_LIMIT)
		return false;
	patchGrid.resize(patch_count);
	Map bounds_check;
	bounds_check.width = width;
	bounds_check.height = height;
	for (int patch_i = 0; patch_i < patch_count; ++patch_i) {
		if (!(in >> setw(12) >> buffer1))
			return false;
		if (std::strcmp(buffer1, "patch") != 0)
			return false;
		if (!load_map_data(in, patchGrid[patch_i], &patchRes))
			return false;
		if (!patch_in_bounds(bounds_check, Patch{&patchGrid[patch_i], 0, 0}))
			return false;
	}
}

ScenarioRunner::ScenarioRunner() : scenario{},
	activeMap{}, scenarioAt{}, commandAt{}, commandCount{}
{ }

void ScenarioRunner::linkScen(const ScenarioLoader& scen)
{
	scenario = &scen;
	activeMap.width = scen.getWidth();
	activeMap.height = scen.getHeight();
	activeMap.bitmap.assign(scen.getWidth() * scen.getHeight(), false);
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
		Patch patch{&patch_grid, cmd.cmd.patch.x, cmd.cmd.patch.y};
		apply_patch(activeMap, patch);
		appliedPatch.push_back(std::move(patch));
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
	query.sx = cmd.cmd.query.sx;
	query.sy = cmd.cmd.query.sy;
	query.gx = cmd.cmd.query.gx;
	query.gy = cmd.cmd.query.gy;
	query.cost = scenario->getQueryCost().at(scenarioAt);
	return query;
}
