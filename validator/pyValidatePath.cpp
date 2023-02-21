#include <pybind11/pybind11.h>
#include "ValidatePath.hpp"

namespace py = pybind11;

struct xyLoc {
    int16_t x;
    int16_t y;
};

struct Checker{
    std::vector<bool> map;
    int width;
    int height;
    Checker(py::list& theMap, int width, int height):width(width),height(height)
    {
        map.resize(py::len(theMap));
        for(int i = 0 ; i<py::len(theMap);i++){
            map[i] = theMap[i].cast<bool>();
        }

    }
    int validatePath(py::list thePath){
        std::vector<xyLoc> path;
        path.resize(py::len(thePath));
        for(int i=0;i<py::len(thePath);i++){
            xyLoc loc;
            loc.x = thePath[i].attr("x").cast<int>();
            loc.y = thePath[i].attr("y").cast<int>();
            path[i] = loc;
        }

        return inx::ValidatePath(map, width, height, path);
    };
};




PYBIND11_MODULE(Grid_Path_Checker, m) {
    py::class_<Checker>(m, "Grid_Path_Checker")
        .def(py::init<py::list&, int, int>())
        .def("validatePath", &Checker::validatePath);
}


