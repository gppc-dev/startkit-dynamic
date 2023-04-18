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

#ifndef GPPC_ASTAR_H
#define GPPC_ASTAR_H

#include <cstdio>
#include <limits>
#include <queue>
#include <vector>
#include <math.h>
#include <map>
#include <algorithm>
#include "GPPC.h"
using namespace std;
typedef GPPC::xyLoc xyLoc;

class Astar {

const double SQRT2 = sqrt(2);
int search_id;

struct Node {
  int x, y;
  double g = 0;
  double h = 0;

  inline double f() const { return g + h; }

  bool operator< (const Node& rhs) const {
    if (f() == rhs.f()) return g > rhs.g;
    else return f() > rhs.f();
  }
};

struct LazyDist {
  int sid;
  double g;
};

public:
  const vector<bool>* bits; // grid map
  int width, height;
  vector<LazyDist> dist;
  vector<int> pa;
  // vector<double> dist;
  // vector<int> pa;
  priority_queue<Node, vector<Node>, less<Node>> q;

  Astar(const vector<bool>* mapData, int w, int h): 
    bits(mapData), width(w), height(h) {
      dist = vector<LazyDist>(bits->size(), {0, numeric_limits<double>::max()});
      pa = vector<int>(bits->size(), -1);
      search_id = 0;
    };

  inline int id(const Node&loc) const {
    return loc.y * width + loc.x;
  }
    
  inline bool traversable(const Node& loc) const {
    return bits->at(id(loc));
  }

  double hVal(const Node& a, const Node& b) {
    int diag = min(abs(a.x - b.x), abs(a.y - b.y));
    int card = abs(a.x - b.x) + abs(a.y - b.y) - 2*diag;
    return card + diag * SQRT2;
  }

  double run(int sx, int sy, int gx, int gy) {
    ++search_id;
    // std pq has no clear operation
    q = priority_queue<Node, vector<Node>, less<Node>>();
    Node g{gx, gy, 0, 0};
    Node s{sx, sy, 0, 0};
    s.h = hVal(s, g);

    dist[id(s)] = {search_id, 0};
    pa[id(s)] = -1;
    q.push(s);

    const int dx[] = {0, 0, 1, -1, 1, -1, 1, -1};
    const int dy[] = {1, -1, 0, 0, 1, -1, -1, 1};
    while (!q.empty()) {
      Node c = q.top(); q.pop();
      if (dist[id(c)].sid == search_id && c.g != dist[id(c)].g) continue;
      if (c.x == g.x && c.y == g.y) return c.g;
      for (int i=0; i<8; i++) {
        int x = c.x + dx[i];
        int y = c.y + dy[i];
        if (0 <= x && x < width && 0 <= y && y < height) {
          // no corner cutting
          if (!traversable({c.x, y}) || 
              !traversable({x, c.y}) ||
              !traversable({x, y})) 
            continue;
          double w = (c.x == x || c.y == y)? 1: SQRT2;
          if (dist[id({x, y})].sid != search_id || dist[id({x, y})].g > c.g + w) {
            dist[id({x, y})] = {search_id, c.g + w};
            pa[id({x, y})] = id({c.x, c.y});
            Node nxt = {x, y, c.g+w};
            nxt.h = hVal(nxt, g);
            q.push(nxt);
          }
        }
      }
    }
    return -1;
  }

  void get_path(xyLoc s, xyLoc g, vector<xyLoc> &path) {
    int16_t w = width;
    double d = run(s.x, s.y, g.x, g.y);
    path.clear();
    if (d > 0) {
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
  }
};

#endif // GPPC_ASTAR_H
