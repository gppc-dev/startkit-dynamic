#pragma once
#include <stdint.h>
#include <vector>
#include <string>
using namespace std;

struct xyLoc {
  int16_t x;
  int16_t y;
};

void PreprocessMap(vector<bool> &bits, int width, int height, const string filename);
void *PrepareForSearch(vector<bool> &bits, int width, int height, const string filename);
bool GetPath(void *data, xyLoc s, xyLoc g, vector<xyLoc> &path);
const string GetName();
