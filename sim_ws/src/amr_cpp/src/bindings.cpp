#include <nanobind/nanobind.h>
#include <nanobind/stl/vector.h>
#include <vector>

namespace nb = nanobind;

std::vector<float> sense_cpp(float x);

NB_MODULE(pf_cpp, m) {
    m.def("sense", &sense_cpp);
}