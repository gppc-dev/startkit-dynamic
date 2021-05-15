#pragma once
#include <vector>
#include <string>

// include common used class in GPPC
#include "GPPC.h"
using namespace std;
typedef GPPC::xyLoc xyLoc;

void PreprocessMap(vector<bool> &bits, int width, int height, const string filename);
void *PrepareForSearch(vector<bool> &bits, int width, int height, const string filename);
bool GetPath(void *data, xyLoc s, xyLoc g, vector<xyLoc> &path);
const string GetName();
