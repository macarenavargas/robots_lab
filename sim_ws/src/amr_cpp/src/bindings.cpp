#include <nanobind/nanobind.h>

namespace nb = nanobind;

std::vector<float> sense_cpp(...);

NB_MODULE(pf_cpp, m) {
    m.def("sense", &sense_cpp);
}