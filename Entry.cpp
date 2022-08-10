#include <algorithm>
#include <map>
#include "Astar.h"
#include "Entry.h"


void PreprocessMap(vector<bool> &bits, int width, int height, const string filename) {}

void *PrepareForSearch(vector<bool> &bits, int width, int height, const string filename) {
  Astar* astar = new Astar(&bits, width, height);
  return astar;
}

bool GetPath(void *data, xyLoc s, xyLoc g, vector<xyLoc> &path) {

  Astar* astar = (Astar*)(data);
  int16_t w = astar->width;
  astar->get_path(s, g, path);
  return true;
}

const string GetName() { return "example-A*"; }
