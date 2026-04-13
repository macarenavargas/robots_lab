#include <nanobind/nanobind.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/pair.h>
#include <vector>

namespace nb = nanobind;

// ── Declared in sense.cpp ─────────────────────────────────────────────────────
std::vector<double> sense_cpp(
    const std::vector<double>& pose,
    const std::vector<std::vector<std::vector<double>>>& map_segments,
    int num_rays,
    double sensor_range_max);

// ── Declared in resample.cpp ──────────────────────────────────────────────────
std::pair<std::vector<std::vector<double>>, double> resample_cpp(
    const std::vector<std::vector<double>>& particles,
    const std::vector<double>& measurements,
    const std::vector<std::vector<std::vector<double>>>& map_segments,
    int    num_rays,
    double sensor_range_max,
    double sensor_range_min,
    double sigma_z,
    int    particle_count);

NB_MODULE(cpp_module, m) {
    m.def("sense", &sense_cpp,
          nb::arg("pose"),
          nb::arg("map_segments"),
          nb::arg("num_rays"),
          nb::arg("sensor_range_max"));

    m.def("resample", &resample_cpp,
          nb::arg("particles"),
          nb::arg("measurements"),
          nb::arg("map_segments"),
          nb::arg("num_rays"),
          nb::arg("sensor_range_max"),
          nb::arg("sensor_range_min"),
          nb::arg("sigma_z"),
          nb::arg("particle_count"));
}