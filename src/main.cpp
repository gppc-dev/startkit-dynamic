#include <cstdio>
#include <numeric>
#include <algorithm>
#include <string>
#include <cmath>
#include <iostream>
#include "ScenarioLoader.h"
#include "Timer.h"
#include "Entry.h"

using namespace std;

string datafile, mapfile, scenfile, flag;
vector<bool> mapData;
int width, height;
bool pre   = false;
bool run   = false;
bool check = false;

void LoadMap(const char *fname, vector<bool> &map, int &width, int &height)
{
  FILE *f;
  f = fopen(fname, "r");
  if (f)
  {
    fscanf(f, "type octile\nheight %d\nwidth %d\nmap\n", &height, &width);
    map.resize(height*width);
    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < width; x++)
      {
        char c;
        do {
          fscanf(f, "%c", &c);
        } while (isspace(c));
        map[y*width+x] = (c == '.' || c == 'G' || c == 'S');
      }
    }
    fclose(f);
  }
}

double GetPathLength(const vector<xyLoc>& path)
{
  double len = 0;
  for (int x = 0; x < (int)path.size()-1; x++)
  {
    if (path[x].x == path[x+1].x || path[x].y == path[x+1].y)
    {
      len++;
    }
    else {
      len += 1.4142;
    }
  }
  return len;
}

void RunExperiment(void* data) {
  Timer t;
  ScenarioLoader scen(scenfile.c_str());
  vector<xyLoc> thePath;

  string resultfile = "result.csv";
  ofstream fout(resultfile);
  const string header = "map,scen,experiment_id,path_size,path_length,ref_length,time_cost";

  fout << header << endl;
  for (int x = 0; x < scen.GetNumExperiments(); x++)
  {
    bool done;
      xyLoc s, g;
    s.x = scen.GetNthExperiment(x).GetStartX();
    s.y = scen.GetNthExperiment(x).GetStartY();
    g.x = scen.GetNthExperiment(x).GetGoalX();
    g.y = scen.GetNthExperiment(x).GetGoalY();

    thePath.clear();
    t.StartTimer();
    done = GetPath(data, s, g, thePath);
    t.EndTimer();
    double tcost = t.GetElapsedTime();
    double plen = done?GetPathLength(thePath): 0;
    double ref_len = scen.GetNthExperiment(x).GetDistance();


    fout << mapfile << "," << scenfile       << ","
         << x       << "," << thePath.size() << ","
         << plen    << "," << ref_len        << ","
         << tcost   << endl;

    if (check) {
      printf("%d %d %d %d %d", s.x, s.y, g.x, g.y, (int)thePath.size());
      for (const auto& it: thePath) {
        printf(" %d %d", it.x, it.y);
      }
      printf(" %.5f\n", plen);
    }
  }
}

void print_help(char **argv) {
  printf("Invalid Arguments\nUsage %s <flag> <map> <scenario>\n", argv[0]);
  printf("Flags:\n");
  printf("\t-full : Preprocess map and run scenario\n");
  printf("\t-pre : Preprocess map\n");
  printf("\t-run : Run scenario without preprocessing\n");
  printf("\t-check: Run for validation\n");
}

bool parse_argv(int argc, char **argv) {
  if (argc < 2) return false;
  flag = string(argv[1]);
  if (flag== "-full") pre = run = true;
  else if (flag == "-pre") pre = true;
  else if (flag == "-run") run = true;
  else if (flag == "-check") run = check = true;

  if (argc < 3) return false;
  mapfile = string(argv[2]);

  if (run) {
    if (argc < 4) return false;
    scenfile = string(argv[3]);
  }
  return true;
}

int main(int argc, char **argv)
{

  // redirect stdout to file
  freopen("run.stdout", "w", stdout);
  freopen("run.stderr", "w", stderr);

  if (!parse_argv(argc, argv)) {
    print_help(argv);
    exit(1);
  }

  LoadMap(mapfile.c_str(), mapData, width, height);
  datafile = GetName() + "-" + mapfile;

  if (pre)
    PreprocessMap(mapData, width, height, datafile);
  
  if (!run)
    return 0;

  void *reference = PrepareForSearch(mapData, width, height, datafile);

  RunExperiment(reference);
  return 0;
}
