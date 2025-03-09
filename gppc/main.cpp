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

#include <cstdio>
#include <ios>
#include <numeric>
#include <algorithm>
#include <string>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <limits>
#include <filesystem>
#include "GPPC.h"
#include "ScenarioLoader.h"
#include "Timer.h"
#include "validator/ValidateSerialize.hpp"
#include "Entry.h"

#if __linux__
#define GPPC_MEMORY_RECORD
#endif

#ifdef GPPC_MEMORY_RECORD
// track memory to output, only in linux
#include <unistd.h>
#endif

namespace GPPC {

using path_type = std::vector<::gppc_point>;
constexpr ::gppc_point point_invalid = ::gppc_point{
	std::numeric_limits<decltype(::gppc_point::x)>::max(),
	std::numeric_limits<decltype(::gppc_point::y)>::max()
};

double EuclideanDist(::gppc_point a, ::gppc_point b) {
	int64_t dx = (int64_t)b.x - (int64_t)a.x;
	int64_t dy = (int64_t)b.y - (int64_t)a.y;
	long double res = std::sqrt(static_cast<long double>(dx * dx) + static_cast<long double>(dy * dy));
	return res;
}

long double GetPathLength(const path_type& path)
{
	long double len = 0;
	for (int x = 0, xe = (int)path.size()-1; x < xe; x++)
		len += EuclideanDist(path[x], path[x+1]);
	return len;
}
long double GetPathLength(const ::gppc_point* path, uint32_t count, ::gppc_point prefix = point_invalid)
{
	if (count == 0)
		return 0;
	if (prefix.x == point_invalid.x && prefix.y == point_invalid.y) {
		prefix = *(path++);
		--count;
	}
	long double len = 0;
	while (count != 0) {
		::gppc_point current = *(path++);
		--count;
		len += EuclideanDist(prefix, current);
		prefix = current;
	}
	return len;
}

class App
{
public:
	struct ResultRow
	{
		// std::string scen; // implicit
		uint32_t experiment_id;
		uint32_t snapshot_id;
		uint64_t snapshot_time;
		uint32_t path_size;
		double path_length;
		double ref_length;
		uint64_t time_cost;
		uint64_t _20steps_cost;
		uint64_t max_step_time;
	};

	bool ParseArgs(int argc, char **argv) {
		if (argc < 2) return false;
		flag = std::string(argv[1]);
		if (flag == "-full") pre = run = true;
		else if (flag == "-pre") pre = true;
		else if (flag == "-run") run = true;
		else if (flag == "-check") run = check = true;
		else return false;

		if (argc < 3) return false;
		scenfile = argv[2];
		return true;
	}

	void PrintHelp(char **argv) {
		std::printf("Invalid Arguments\nUsage %s <flag> <scenario>\n", argv[0]);
		std::printf("Flags:\n");
		std::printf("\t-full : Preprocess map and run scenario\n");
		std::printf("\t-pre : Preprocess map\n");
		std::printf("\t-run : Run scenario without preprocessing\n");
		std::printf("\t-check: Run for validation\n");
	}

	int RunExperiment(ScenarioRunner& scen_run, void* data) {
		result_csv.assign(scen_run.getLoader()->getQueryCommands(), ResultRow{});
		Timer t;
		path_type thePath;
		if (check) {
			thePath.reserve(1024);
		}

		validate::Serialize validator;
		if (check) {
			validator.Setup(scen_run.getActiveMapReal(), std::cout);
			validator.PrintHeader();
		}

		for (int query_id = 0; ; query_id++)
		{
			typedef Timer::duration dur;
			dur snapshot_time = dur::zero();
			if (query_id != 0) {
				int patch_changes = scen_run.nextQuery();
				if (patch_changes < 0)
					break; // no more queries
				else if (patch_changes != 0) {
					// map changed
					auto& patches = scen_run.getAppliedPatches();
					t.StartTimer();
					::gppc_map_change(data, patches.data(), patches.size());
					t.EndTimer();
					snapshot_time = t.GetElapsedTime();
				}
			}
			auto scen = scen_run.getCurrentQuery();
			int state_id = scen.bucket;
			if (check) {
				validator.AddQuery({query_id, state_id,
					{scen.start.x, scen.start.y},
					{scen.goal.x, scen.goal.y},
					scen.cost});
			}

			thePath.clear();
			dur tcost_curr = snapshot_time, tcost = dur::zero(), tcost_first = dur::zero(), max_step = dur::zero();
			// tcost_curr is the virtual timer of gppc_get_path, where the first call includes snapshot_time but following do not
			// tcost appends all tcost_curr
			// tcost_first appends tcost_curr until at least path length PATH_FIRST_STEP_LENGTH (20)
			// max_step is the maximum tcost_curr
			bool done = false, done_first = false;
			::gppc_path result_path;
			uint32_t run_len = 0;
			long double run_cost = 0;
			::gppc_point run_cost_prefix = point_invalid;
			do {
				t.StartTimer();
				result_path = ::gppc_get_path(data, scen.start, scen.goal);
				t.EndTimer();
				tcost_curr += t.GetElapsedTime();
				// move result_path into thePath
				done = !result_path.incomplete;
				if (result_path.length < 0) {
					std::cerr << "Negative path length provided (" << result_path.length << ")\n";
					return 2;
				} else if (result_path.path == nullptr && result_path.length > 0) {
					std::cerr << "Null path has length > 0 (" << result_path.length << ")\n";
					return 2;
				} else if (result_path.length == 0 && result_path.incomplete != 0) {
					std::cerr << "Empty path is not marked as incomplete\n";
					return 2;
				}
				if (check) {
					if (result_path.length == 0)
						thePath.clear();
					else
						thePath.assign(result_path.path, result_path.path + result_path.length);
				}
				if (run) {
					if (result_path.length != 0) {
						run_len += result_path.length;
						run_cost += GetPathLength(result_path.path, result_path.length, run_cost_prefix);
						run_cost_prefix = result_path.path[result_path.length-1];
					}
				}

				// handle time
				max_step = std::max(max_step, tcost_curr);
				tcost += tcost_curr;
				if (!done_first) {
					tcost_first += tcost_curr;
					done_first = GetPathLength(thePath) >= PATH_FIRST_STEP_LENGTH - 1e-6;
				}
				tcost_curr = dur::zero(); // zero for next iteration

				if (check) {
					validator.AddSubPath(thePath, !done);
				}
			} while (!done);
			if (check) {
				validator.FinQuery();
			}
			double ref_len = scen.cost;
			double plen;
			uint64_t plen_size;
			if (!done) {
				plen = 0;
				plen_size = 0;
			} else {
				plen = run_len != 0 ? static_cast<double>(run_cost) : -1.0;
				plen_size = run_len;
			}

			ResultRow row;
			row.experiment_id = query_id;
			row.snapshot_id = state_id;
			row.snapshot_time = snapshot_time.count();
			row.path_size = plen_size;
			row.path_length = plen;
			row.ref_length = ref_len;
			row.time_cost = tcost.count();
			row._20steps_cost = tcost_first.count();
			row.max_step_time = max_step.count();
			result_csv[query_id] = row;
		}
		return 0;
	}

	int Run(int argc, char **argv)
	{
		if (!ParseArgs(argc, argv)) {
			PrintHelp(argv);
			return 1;
		}

		bool redirect_output = std::getenv("GPPC_REDIRECT_OUTPUT") != nullptr;
		if (redirect_output) {
			// redirect stdout to file
			std::freopen("run.stdout", "w", stdout);
			std::freopen("run.stderr", "w", stderr);
		}

		// in mapData, 1: traversable, 0: obstacle
		ScenarioLoader scen;
		if (!scen.load(scenfile)) {
			std::cerr << "Failed to load scenario file: " << scenfile << std::endl;
			return 1;
		}
		datafile = index_dir / (std::string(::gppc_get_name()) + "-" + scenfile.stem().string());

		ScenarioRunner scenRun;
		scenRun.linkScen(scen);
		int qid = scenRun.nextQuery();
		if (qid < 0)
			return 1; // no queries to run

		if (pre)
			::gppc_preprocess_init_map(scenRun.getActiveMap(), datafile.c_str());
		
		if (!run)
			return 0;

		void *reference = nullptr;
		{
			Timer timer;
			timer.StartTimer();
			reference = ::gppc_search_init(scenRun.getActiveMap(), datafile.c_str());
			timer.EndTimer();
			std::ofstream fout("run.info");
			fout << "search_init " << timer.GetElapsedTime().count() << std::endl;
		}

		bool memory_track = std::getenv("GPPC_MEMORY_TRACK") != nullptr;
#ifdef GPPC_MEMORY_RECORD
		if (memory_track) {
			char argument[256];
			std::sprintf(argument, "pmap -x %d | tail -n 1 >> run.info", getpid());
			std::system(argument);
		}
#else
		if (memory_track) {
			std::cerr << "env GPPC_MEMORY_TRACK set but only available on linux.\n";
		}
#endif
		RunExperiment(scenRun, reference);
		{
			std::string resultfile = "result.csv";
			std::ofstream fout(resultfile);
			PrintResult(fout);
		}
#ifdef GPPC_MEMORY_RECORD
		if (memory_track) {
			char argument[256];
			std::sprintf(argument, "pmap -x %d | tail -n 1 >> run.info", getpid());
			std::system(argument);
		}
#endif

		::gppc_free_data(reference);

		return 0;
	}

	void PrintResult(std::ostream& out)
	{
		out << "scen,experiment_id,snapshot_id,snapshot_time,path_size,path_length,ref_length,time_cost,20steps_cost,max_step_time\n";
		for (const ResultRow& row : result_csv) {
			out << std::setprecision(14) << std::fixed;
			out << scenfile.string() << ','
				<< row.experiment_id << ',' << row.snapshot_id << ','
				<< row.snapshot_time << ','
				<< row.path_size << ','
				<< row.path_length << ',' << row.ref_length << ','
				<< row.time_cost << ',' << row._20steps_cost << ','
				<< row.max_step_time << '\n';
		}
	}

public:
	std::filesystem::path datafile, scenfile, flag;
	const std::filesystem::path index_dir = "index_data";
	bool pre	 = false;
	bool run	 = false;
	bool check = false;
	std::vector<ResultRow> result_csv;
};

};

int main(int argc, char **argv)
{
	GPPC::App app;

	return app.Run(argc, argv);
}
