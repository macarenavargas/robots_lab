#include <nanobind/nanobind.h>
#include <nanobind/stl/vector.h>
#include <vector>

namespace nb = nanobind;

// Declared in pf_core.cpp
std::vector<double> sense_cpp(
    const std::vector<double> &pose,
    const std::vector<std::vector<std::vector<double>>> &map_segments,
    int num_rays,
    double sensor_range_max);

NB_MODULE(cpp_module, m) {
    m.def("sense", &sense_cpp,
          nb::arg("pose"),
          nb::arg("map_segments"),
          nb::arg("num_rays"),
          nb::arg("sensor_range_max"));
}