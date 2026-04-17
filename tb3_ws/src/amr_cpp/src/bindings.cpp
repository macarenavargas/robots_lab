
// skipping unchanged file content ...
#include <nanobind/nanobind.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/tuple.h>
#include <vector>

namespace nb = nanobind;

// ── Declared in sense.cpp ─────────────────────────────────────────────────────
std::vector<double> sense_cpp(
    const std::vector<double>& pose,
    const std::vector<std::vector<std::vector<double>>>& map_segments,
    int num_rays,
    double sensor_range_max);

// ── Declared in resample.cpp ──────────────────────────────────────────────────
std::tuple<std::vector<std::vector<double>>, double, std::vector<double>> resample_cpp(
    const std::vector<std::vector<double>>& particles,
    const std::vector<double>& measurements,
    const std::vector<std::vector<std::vector<double>>>& map_segments,
    int    num_rays,
    double sensor_range_max,
    double sensor_range_min,
    double sigma_z,
    int    particle_count,
    const std::vector<double>& prev_raw_measurements);

// ── Declared in move.cpp ──────────────────────────────────────────────────────
std::vector<std::vector<double>> move_cpp(
    const std::vector<std::vector<double>>& particles,
    double v,
    double w,
    double dt,
    double sigma_v,
    double sigma_w,
    bool simulation,
    const std::vector<std::vector<std::vector<double>>>& map_segments);

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
          nb::arg("particle_count"),
          nb::arg("prev_raw_measurements"));

    m.def("move", &move_cpp,
          nb::arg("particles"),
          nb::arg("v"),
          nb::arg("w"),
          nb::arg("dt"),
          nb::arg("sigma_v"),
          nb::arg("sigma_w"),
          nb::arg("simulation"),
          nb::arg("map_segments"));
}