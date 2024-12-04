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
#include "ScenarioLoader.h"
#include "Timer.h"
#include "Entry.h"
#include "validator/ValidatePath.hpp"

#if __linux__
#define GPPC_MEMORY_RECORD
#endif

#ifdef GPPC_MEMORY_RECORD
// track memory to output, only in linux
#include <unistd.h>
#endif

std::string datafile, scenfile, flag;
const std::string index_dir = "index_data";
constexpr double PATH_FIRST_STEP_LENGTH = 20.0;
std::vector<bool> mapData;
int width, height;
bool pre   = false;
bool run   = false;
bool check = false;

double euclidean_dist(const xyLoc& a, const xyLoc& b) {
  int dx = std::abs(b.x - a.x);
  int dy = std::abs(b.y - a.y);
  double res = std::sqrt(dx * dx + dy * dy);
  return res;
}

double GetPathLength(const std::vector<xyLoc>& path)
{
  double len = 0;
  for (int x = 0; x < (int)path.size()-1; x++)
    len += euclidean_dist(path[x], path[x+1]);
  return len;
}

// returns -1 if valid path, otherwise id of segment where invalidness was detetcted
int ValidatePath(const std::vector<xyLoc>& thePath)
{
  return inx::ValidatePath(mapData, width, height, thePath);
}

void RunExperiment(ScenarioRunner& run, void* data) {
  Timer t;
  std::vector<xyLoc> thePath;
  thePath.reserve(1024);

  std::string resultfile = "result.csv";
  std::ofstream fout(resultfile);
  const std::string header = "scen,experiment_id,path_size,path_length,ref_length,time_cost,20steps_cost,max_step_time";

  fout << header << std::endl;
  for (int query_id = 0; ; query_id++)
  {
    if (query_id != 0) {
      int patch_changes = run.nextQuery();
      if (patch_changes < 0)
        break; // no more queries
      else if (patch_changes != 0) {
        // map changed
        DynamicMapChange(data, run.getActiveMap(), run.getAppliedPatches());
      }
    }
    xyLoc s, g;
    auto scen = run.getCurrentQuery();
    s.x = scen.sx;
    s.y = scen.sy;
    g.x = scen.gx;
    g.y = scen.gy;

    thePath.clear();
    typedef Timer::duration dur;
    dur max_step = dur::zero(), tcost = dur::zero(), tcost_first = dur::zero();
    bool done = false, done_first = false;
    do {
      t.StartTimer();
      done = GetPath(data, s, g, thePath);
      t.EndTimer();
      max_step = std::max(max_step, t.GetElapsedTime());
      tcost += t.GetElapsedTime();
      if (!done_first) {
        tcost_first += t.GetElapsedTime();
        done_first = GetPathLength(thePath) >= PATH_FIRST_STEP_LENGTH - 1e-6;
      }
    } while (!done);
    double plen = done?GetPathLength(thePath): 0;
    double ref_len = scen.cost;


    fout << std::setprecision(9) << std::fixed;
    fout << scenfile       << ","
         << query_id        << "," << thePath.size() << ","
         << plen     << "," << ref_len        << ","
         << tcost.count() << "," << tcost_first.count() << ","
         << max_step.count() << std::endl;

    if (check) {
      std::printf("%d %d %d %d", s.x, s.y, g.x, g.y);
      int validness = ValidatePath(thePath);
      if (validness < 0) {
        std::printf(" valid");
      } else {
        std::printf(" invalid-%d", validness);
      }
      std::printf(" %d", static_cast<int>(thePath.size()));
      for (const auto& it: thePath) {
        std::printf(" %d %d", it.x, it.y);
      }
      std::printf(" %.5f\n", plen);
    }
  }
}

void print_help(char **argv) {
  std::printf("Invalid Arguments\nUsage %s <flag> <scenario>\n", argv[0]);
  std::printf("Flags:\n");
  std::printf("\t-full : Preprocess map and run scenario\n");
  std::printf("\t-pre : Preprocess map\n");
  std::printf("\t-run : Run scenario without preprocessing\n");
  std::printf("\t-check: Run for validation\n");
}

bool parse_argv(int argc, char **argv) {
  if (argc < 2) return false;
  flag = std::string(argv[1]);
  if (flag == "-full") pre = run = true;
  else if (flag == "-pre") pre = true;
  else if (flag == "-run") run = true;
  else if (flag == "-check") run = check = true;
  else return false;

  if (argc < 3) return false;
  scenfile = std::string(argv[2]);
  return true;
}

std::string basename(const std::string& path) {
  std::size_t l = path.find_last_of('/');
  if (l == std::string::npos) l = 0;
  else l += 1;
  std::size_t r = path.find_last_of('.');
  if (r == std::string::npos) r = path.size()-1;
  return path.substr(l, r-l);
}

int main(int argc, char **argv)
{

  if (!parse_argv(argc, argv)) {
    print_help(argv);
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
  datafile = index_dir + "/" + GetName() + "-" + basename(scenfile);

  ScenarioRunner scenRun;
  scenRun.linkScen(scen);
  int qid = scenRun.nextQuery();
  if (qid < 0)
    return 1; // no queries to run

  if (pre)
    PreprocessMap(scenRun.getActiveMap(), datafile);
  
  if (!run)
    return 0;

  void *reference = SearchInit(scenRun.getActiveMap(), datafile);

#ifdef GPPC_MEMORY_RECORD
  bool memory_track = std::getenv("GPPC_MEMORY_TRACK") != nullptr;
  char argument[256];
  if (memory_track) {
    std::sprintf(argument, "pmap -x %d | tail -n 1 > run.info", getpid());
    std::system(argument);
  }
#endif
  RunExperiment(scenRun, reference);
#ifdef GPPC_MEMORY_RECORD
  if (memory_track) {
    std::sprintf(argument, "pmap -x %d | tail -n 1 >> run.info", getpid());
    std::system(argument);
  }
#endif
  return 0;
}
