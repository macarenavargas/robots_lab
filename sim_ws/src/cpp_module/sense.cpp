#include <nanobind/nanobind.h>
#include <nanobind/stl.h>
#include <vector>
#include <array>

namespace nb = nanobind;
extern "C" {
    #include "intersect.h"
}

std::vector<double> sense(
    std::array<double, 3> pose,
    std::vector<std::array<std::array<double, 2>, 2>> segments,
    int num_rays,
    double max_range
) {
    std::vector<double> z_hat;

    // TEST: devolver max_range siempre
    for (int i = 0; i < num_rays; i++) {
        z_hat.push_back(max_range);
    }

    return z_hat;
}

NB_MODULE(cpp_module, m) {
    m.def("sense", &sense);
}