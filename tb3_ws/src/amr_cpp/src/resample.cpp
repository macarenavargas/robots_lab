/**
 * @file resample.cpp
 *
 * @brief C++ implementation of the particle filter resample step.
 *        Direct equivalent of ParticleFilter.resample() in particle_filter.py.
 *
 * Steps implemented (mirrors the Python version exactly):
 *   1. Extract num_rays robust LiDAR values from the full scan
 *   2. Compute the weight of every particle (sense + Gaussian likelihood)
 *   3. Normalize weights (uniform fallback if all zero)
 *   4. Build cumulative weight sum
 *   5. Systematic resampling → select particle_count survivors
 *   6. Return (new_particles, average_likelihood)
 */

#include <algorithm>
#include <cmath>
#include <random>
#include <vector>
#include <tuple>

// ─────────────────────────────────────────────────────────────────────────────
//  Forward declaration — implemented in sense.cpp
// ─────────────────────────────────────────────────────────────────────────────
std::vector<double> sense_cpp(
    const std::vector<double>& pose,
    const std::vector<std::vector<std::vector<double>>>& map_segments,
    int num_rays,
    double sensor_range_max);

// ─────────────────────────────────────────────────────────────────────────────
//  Private helpers
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Gaussian probability.
 *        Mirrors ParticleFilter._gaussian(mu, sigma, x).
 */
static double gaussian(double mu, double sigma, double x)
{
    double error = x - mu;
    double num   = std::exp(-(error * error) / (2.0 * sigma * sigma));
    double den   = sigma * std::sqrt(2.0 * M_PI);
    return num / den;
}

/**
 * @brief Extracts num_rays robust values from the full LiDAR scan.
 *        Mirrors ParticleFilter._extract_robust_measurements().
 *
 *  - Selects num_rays equally-spaced indices via linspace(0, n-1, num_rays, dtype=int).
 *  - Replaces NaN / Inf / non-positive values with sensor_range_min.
 */
static std::pair<std::vector<double>, std::vector<double>> extract_robust_measurements(
    const std::vector<double>& measurements,
    const std::vector<double>& prev_raw_measurements,
    int    num_rays,
    double sensor_range_min)
{
    int n = static_cast<int>(measurements.size());

    if (n == 0) {
        return {std::vector<double>(num_rays, sensor_range_min),
                std::vector<double>(num_rays, sensor_range_min)};
    }

    std::vector<double> z_real(num_rays);
    std::vector<double> current_raw(num_rays);

    for (int i = 0; i < num_rays; ++i) {
        // int(np.linspace(0, n-1, num_rays)[i])  → truncating cast, same as numpy dtype=int
        int    idx = (num_rays > 1) ? static_cast<int>((double)(n - 1) * i / (num_rays - 1)) : 0;
        double val = measurements[idx];
        current_raw[i] = val;

        if (std::isnan(val) || std::isinf(val) || val <= 0.0) {
            if (!prev_raw_measurements.empty() && i < prev_raw_measurements.size()) {
                double prev_val = prev_raw_measurements[i];
                if (std::isnan(prev_val) || std::isinf(prev_val) || prev_val <= 0.0) {
                    z_real[i] = sensor_range_min;
                } else {
                    z_real[i] = prev_val;
                }
            } else {
                z_real[i] = sensor_range_min;
            }
        } else {
            z_real[i] = val;
        }
    }
    return {z_real, current_raw};
}

/**
 * @brief Weight of a single particle given the real LiDAR measurements.
 *        Mirrors ParticleFilter._measurement_probability().
 *
 *  1. Simulates LiDAR rays for the particle via sense_cpp.
 *  2. Replaces NaN values with sensor_range_min (np.nan_to_num).
 *  3. Returns the product of Gaussian likelihoods across all rays.
 */
static double measurement_probability(
    const std::vector<double>& z_real,
    const std::vector<double>& particle,
    const std::vector<std::vector<std::vector<double>>>& map_segments,
    int    num_rays,
    double sensor_range_max,
    double sensor_range_min,
    double sigma_z)
{
    std::vector<double> z_hat =
        sense_cpp(particle, map_segments, num_rays, sensor_range_max);

    // np.nan_to_num(z_hat, nan=sensor_range_min)
    for (double& v : z_hat)
        if (std::isnan(v)) v = sensor_range_min;

    double probability = 1.0;
    for (int i = 0; i < num_rays; ++i)
        probability *= gaussian(z_hat[i], sigma_z, z_real[i]);

    return probability;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Main public function
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Full C++ resampling step — equivalent of ParticleFilter.resample().
 *
 * Python call-site (particle_filter.py _resample_cpp):
 *   cpp_module.resample(
 *       self._particles.tolist(), list(measurements),
 *       self._map._map_segments,
 *       self._num_rays, self._sensor_range_max, self._sensor_range_min,
 *       self._sigma_z, self._particle_count)
 *
 * @param particles         Current particles (N×3): [[x, y, theta], ...]
 * @param measurements      Full LiDAR scan (e.g. 360 values) [m]
 * @param map_segments      Wall segments: [[[x0,y0],[x1,y1]], ...]
 * @param num_rays          Number of rays to use (typically 8)
 * @param sensor_range_max  Maximum sensor range [m]  (e.g. 8.0)
 * @param sensor_range_min  Minimum sensor range [m]  (e.g. 0.16)
 * @param sigma_z           Measurement noise std deviation
 * @param particle_count    Target number of particles after resampling
 *                          (can differ from N when DBSCAN adapts it, e.g. 50)
 * @param prev_raw_measurements Previous raw LiDAR measurements to serve as fallback
 *
 * @return { new_particles (M×3), average_likelihood, current_raw_measurements (1x8) }
 */
std::tuple<std::vector<std::vector<double>>, double, std::vector<double>> resample_cpp(
    const std::vector<std::vector<double>>& particles,
    const std::vector<double>& measurements,
    const std::vector<std::vector<std::vector<double>>>& map_segments,
    int    num_rays,
    double sensor_range_max,
    double sensor_range_min,
    double sigma_z,
    int    particle_count,
    const std::vector<double>& prev_raw_measurements)
{
    int N = static_cast<int>(particles.size());

    // ── STEP 1: extract robust measurements ─────────────────────────────────
    auto robust_res = extract_robust_measurements(measurements, prev_raw_measurements, num_rays, sensor_range_min);
    std::vector<double> z_real = robust_res.first;
    std::vector<double> current_raw = robust_res.second;

    // ── STEP 2: compute weight for every particle ────────────────────────────
    std::vector<double> weights(N);
    for (int i = 0; i < N; ++i)
        weights[i] = measurement_probability(
            z_real, particles[i], map_segments,
            num_rays, sensor_range_max, sensor_range_min, sigma_z);

    // ── STEP 3: normalize weights ────────────────────────────────────────────
    double weight_sum = 0.0;
    for (double w : weights) weight_sum += w;

    double average_likelihood = weight_sum / N;

    if (weight_sum > 0.0) {
        for (double& w : weights) w /= weight_sum;
    } else {
        // Safety fallback: uniform weights (same as Python version)
        double uniform = 1.0 / N;
        for (double& w : weights) w = uniform;
    }

    // ── STEP 4: cumulative sum ───────────────────────────────────────────────
    std::vector<double> cumulative(N);
    cumulative[0] = weights[0];
    for (int i = 1; i < N; ++i)
        cumulative[i] = cumulative[i - 1] + weights[i];

    // ── STEP 5: systematic resampling ────────────────────────────────────────
    // Python: positions = (np.random.random() + np.arange(N)) / N
    //         indices   = np.digitize(positions, cumulative_sum)
    // C++:    np.digitize with right=False  ==  std::upper_bound
    int M = particle_count;

    std::mt19937 rng(std::random_device{}());
    std::uniform_real_distribution<double> uniform_dist(0.0, 1.0);
    double r = uniform_dist(rng); // single random draw, same as Python

    std::vector<std::vector<double>> new_particles(M, std::vector<double>(3));
    for (int i = 0; i < M; ++i) {
        double pos = (r + i) / static_cast<double>(M);
        // First cumulative element strictly greater than pos
        auto it  = std::upper_bound(cumulative.begin(), cumulative.end(), pos);
        int  idx = static_cast<int>(it - cumulative.begin());
        if (idx >= N) idx = N - 1; // defensive clamp
        new_particles[i] = particles[idx];
    }

    // ── STEP 6: return ───────────────────────────────────────────────────────
    return {new_particles, average_likelihood, current_raw};
}
