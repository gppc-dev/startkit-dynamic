#pragma once
#include <cstdio>
#include <limits>
#include <queue>
#include <vector>
#include <map>
using namespace std;

class Dijkstra {

typedef pair<int, int> xy;

struct Node {
  int x, y;
  double d;

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

  int xy2id(xy loc) {
    return loc.second * width + loc.first;
  }
    
  bool traversable(xy loc) {
    return bits->at(xy2id(loc));
  }

  bool run(xy s, xy g, vector<int>& pa) {
    priority_queue<Node, vector<Node>> q;
    dist = vector<double>(bits->size(), numeric_limits<double>::max());

    dist[xy2id(s)] = 0;
    pa[xy2id(s)] = -1;
    q.push({s.first, s.second, 0});

    const int dx[] = {0, 0, 1, -1, 1, -1, 1, -1};
    const int dy[] = {1, -1, 0, 0, 1, -1, -1, 1};
    while (!q.empty()) {
      Node c = q.top(); q.pop();
      if (c.d != dist[xy2id({c.x, c.y})]) continue;
      if (c.x == g.first && c.y == g.second) return true;
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
          if (dist[xy2id({x, y})] > c.d + w) {
            dist[xy2id({x, y})] = c.d + w;
            pa[xy2id({x, y})] = xy2id({c.x, c.y});
            q.push({x, y, c.d+w});
          }
        }
      }
    }
    return false;
  }
};
