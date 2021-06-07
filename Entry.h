#pragma once
#include <vector>
#include <string>

// include common used class in GPPC
#include "GPPC.h"
using namespace std;
typedef GPPC::xyLoc xyLoc;

void PreprocessMap(vector<bool> &bits, int width, int height, const string filename);
void *PrepareForSearch(vector<bool> &bits, int width, int height, const string filename);


/*
return true if the pathfinding is completed (even if not path exist), 
usually this function always return true;

return false if the pathfinding is not completed and requires further function calls, 
e.g.:
  the shortest path from s to g is <s,v1,v2,g>
  GetPath(data, s, g, path);  // get the prefix <s, v1>
  GetPath(data, v1, g, path); // get the prefix <s,v1,v2>
  GetPath(data, v2, g, path); // get the entire <s,v1,v2,g>
  
*/
bool GetPath(void *data, xyLoc s, xyLoc g, vector<xyLoc> &path);

const string GetName();
