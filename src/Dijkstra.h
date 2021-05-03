#pragma once
#include <cstdio>
#include <limits>
#include <queue>
#include <vector>
#include <map>
using namespace std;

class Dijkstra {

struct Node {
  int x, y;
  double d = 0;

  bool operator< (const Node& rhs) const {
    return d > rhs.d;
  }
};

public:
  const vector<bool>* bits; // grid map
  int width, height;
  vector<double> dist;

  Dijkstra(const vector<bool>* mapData, int w, int h): 
    bits(mapData), width(w), height(h) {};

  inline int id(const Node&loc) const {
    return loc.y * width + loc.x;
  }
    
  inline bool traversable(const Node& loc) const {
    return bits->at(id(loc));
  }

  bool run(int sx, int sy, int gx, int gy, vector<int>& pa) {
    priority_queue<Node, vector<Node>> q;
    dist = vector<double>(bits->size(), numeric_limits<double>::max());
    Node s{sx, sy, 0};
    Node g{gx, gy, 0};

    dist[id(s)] = 0;
    pa[id(s)] = -1;
    q.push(s);

    const int dx[] = {0, 0, 1, -1, 1, -1, 1, -1};
    const int dy[] = {1, -1, 0, 0, 1, -1, -1, 1};
    while (!q.empty()) {
      Node c = q.top(); q.pop();
      if (c.d != dist[id(c)]) continue;
      if (c.x == g.x && c.y == g.y) return true;
      for (int i=0; i<8; i++) {
        int x = c.x + dx[i];
        int y = c.y + dy[i];
        if (0 <= x && x < width && 0 <= y && y < height) {
          // no corner cutting
          if (!traversable({c.x, y}) || 
              !traversable({x, c.y}) ||
              !traversable({x, y})) 
            continue;
          double w = (c.x == x || c.y == y)? 1: 1.4142;
          if (dist[id({x, y})] > c.d + w) {
            dist[id({x, y})] = c.d + w;
            pa[id({x, y})] = id({c.x, c.y});
            q.push({x, y, c.d+w});
          }
        }
      }
    }
    return false;
  }
};
