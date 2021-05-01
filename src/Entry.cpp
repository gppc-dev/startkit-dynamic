#include <algorithm>
#include <map>
#include "Dijkstra.h"
#include "Entry.h"


void PreprocessMap(vector<bool> &bits, int width, int height, const string filename) {}

void *PrepareForSearch(vector<bool> &bits, int width, int height, const string filename) {
  Dijkstra* dij = new Dijkstra(&bits, width, height);
  return dij;
}

bool GetPath(void *data, xyLoc s, xyLoc g, vector<xyLoc> &path) {

  Dijkstra* dij = (Dijkstra*)(data);
  int16_t w = dij->width;

  vector<int> pa(dij->bits->size(), -1);
  bool res = dij->run({s.x, s.y}, {g.x, g.y}, pa);
  if (res) {
    int16_t x = g.x, y = g.y;
    while (true) {
      path.push_back({x, y});
      if (x == s.x && y == s.y) break;
      int cid = y * w + x;
      x = pa[cid] % w;
      y = pa[cid] / w;
    }
    reverse(path.begin(), path.end());
  }
  return res;
}

const string GetName() { return "example"; }
