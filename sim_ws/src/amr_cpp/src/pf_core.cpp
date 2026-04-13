#include <nanobind/nanobind.h>

namespace nb = nanobind;

NB_MODULE(pf_cpp, m) {
    m.def("test", []() { return 42; });
}