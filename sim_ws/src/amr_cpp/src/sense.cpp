/**
 * @file pf_core.cpp
 *
 * @brief C++ implementation of the particle filter sense function.
 *        Replicates _sense_python() + _lidar_rays() + check_collision()
 *        from the Python particle filter for improved performance.
 *
 * Python equivalent (particle_filter.py):
 *   _sense_python(pose)  →  sense_cpp(pose, map_segments, num_rays, sensor_range_max)
 */

#include <cmath>
#include <limits>
#include <vector>

// ─────────────────────────────────────────────────────────────────────────────
//  Geometry helpers  (mirrors intersect.c / intersect.py logic)
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Rounds a value to 6 decimal places.
 *        Equivalent to Python's round(x, 6).
 */
static double round6(double x)
{
    return std::round(x * 1e6) / 1e6;
}

/**
 * @brief Computes the slope of a line defined by two points.
 *        Returns +Infinity if the line is vertical (dx < 1e-6).
 */
static double compute_slope(double x0, double y0, double x1, double y1)
{
    if (std::abs(x1 - x0) < 1e-6)
        return std::numeric_limits<double>::infinity();
    return (y1 - y0) / (x1 - x0);
}

/**
 * @brief Computes the y-intercept of a line given its slope and one point.
 */
static double compute_intercept(double slope, double x, double y)
{
    return y - slope * x;
}

/**
 * @brief Computes the intersection point of two *segments*.
 *        Mirrors the C library function segment_intersect() in intersect.c
 *        and the Python class method Intersect.segment_intersect().
 *
 * @param[out] xi  x coordinate of intersection point.
 * @param[out] yi  y coordinate of intersection point.
 * @param[in]  x0,y0  Start of segment 1 (the LiDAR ray).
 * @param[in]  x1,y1  End   of segment 1.
 * @param[in]  x2,y2  Start of segment 2 (a wall segment).
 * @param[in]  x3,y3  End   of segment 2.
 *
 * @return true if the segments intersect; xi and yi are set.
 *         false otherwise.
 */
static bool segment_intersect(
    double &xi, double &yi,
    double x0, double y0, double x1, double y1,
    double x2, double y2, double x3, double y3)
{
    xi = std::numeric_limits<double>::quiet_NaN();
    yi = std::numeric_limits<double>::quiet_NaN();

    double m1 = compute_slope(x0, y0, x1, y1);
    double m2 = compute_slope(x2, y2, x3, y3);

    // ── Compute infinite-line intersection ──────────────────────────────────
    if (!std::isinf(m1) && !std::isinf(m2)) {
        // Both lines have finite slope
        if (std::abs(m1 - m2) < 1e-6)
            return false; // parallel (or coincident)
        double b1 = compute_intercept(m1, x0, y0);
        double b2 = compute_intercept(m2, x2, y2);
        xi = (b2 - b1) / (m1 - m2);
        yi = m1 * xi + b1;
    }
    else if (std::isinf(m1) && !std::isinf(m2)) {
        // Segment 1 is vertical
        double b2 = compute_intercept(m2, x2, y2);
        xi = x0;
        yi = m2 * xi + b2;
    }
    else if (std::isinf(m2) && !std::isinf(m1)) {
        // Segment 2 is vertical
        double b1 = compute_intercept(m1, x0, y0);
        xi = x2;
        yi = m1 * xi + b1;
    }
    else {
        // Both vertical → parallel
        return false;
    }

    if (std::isnan(xi) || std::isnan(yi))
        return false;

    // ── Check that the intersection lies inside both segments ───────────────
    // (mirrors the Python round-and-check logic exactly)
    double xri = round6(xi), yri = round6(yi);

    double xr0 = round6(x0), xr1 = round6(x1);
    double yr0 = round6(y0), yr1 = round6(y1);
    double xr2 = round6(x2), xr3 = round6(x3);
    double yr2 = round6(y2), yr3 = round6(y3);

    // Segment 1 – x axis
    if (xr0 < xr1) { if (xri < xr0 || xri > xr1) return false; }
    else            { if (xri > xr0 || xri < xr1) return false; }

    // Segment 1 – y axis
    if (yr0 < yr1) { if (yri < yr0 || yri > yr1) return false; }
    else           { if (yri > yr0 || yri < yr1) return false; }

    // Segment 2 – x axis
    if (xr2 < xr3) { if (xri < xr2 || xri > xr3) return false; }
    else            { if (xri > xr2 || xri < xr3) return false; }

    // Segment 2 – y axis
    if (yr2 < yr3) { if (yri < yr2 || yri > yr3) return false; }
    else           { if (yri > yr2 || yri < yr3) return false; }

    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Main public function
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Simulates LiDAR readings for a single particle pose.
 *        Direct C++ equivalent of ParticleFilter._sense_python().
 *
 * Python call-site (particle_filter.py):
 *   cpp_module.sense(pose.tolist(), self._map._map_segments,
 *                    self._num_rays, self._sensor_range_max)
 *
 * @param pose             [x, y, theta] in [m, m, rad].
 * @param map_segments     Wall segments: list of [[x0,y0],[x1,y1]].
 * @param num_rays         Number of LiDAR rays to simulate (typically 8).
 * @param sensor_range_max Maximum sensor range [m] (typically 8.0).
 *
 * @return Vector of distances [m] per ray; NaN if no wall was hit.
 */
std::vector<double> sense_cpp(
    const std::vector<double> &pose,
    const std::vector<std::vector<std::vector<double>>> &map_segments,
    int num_rays,
    double sensor_range_max)
{
    // ── 1. Parse robot pose ─────────────────────────────────────────────────
    const double x     = pose[0];
    const double y     = pose[1];
    const double theta = pose[2];

    // ── 2. LiDAR origin: 3.5 cm behind robot center ─────────────────────────
    //    Mirrors: x_start = x - 0.035 * cos(theta)
    //             y_start = y - 0.035 * sin(theta)
    const double x_start = x - 0.035 * std::cos(theta);
    const double y_start = y - 0.035 * std::sin(theta);

    // ── 3. Select ray indices ────────────────────────────────────────────────
    //    Mirrors: np.linspace(0, 239, num_rays, dtype=int)
    //    numpy dtype=int truncates (floor for non-negative values).
    //    For num_rays=8: [0, 34, 68, 102, 136, 170, 204, 239]
    constexpr double degree_increment = 1.5; // degrees per LiDAR tick

    std::vector<double> z_hat;
    z_hat.reserve(num_rays);

    for (int i = 0; i < num_rays; ++i) {
        // Equivalent to int(linspace(0, 239, num_rays)[i])
        int idx = (num_rays > 1) ? static_cast<int>(239.0 * i / (num_rays - 1)) : 0;

        // ── 4. Build the ray segment ─────────────────────────────────────────
        //    ray_angle = radians(degree_increment * idx)
        //    x_end = x_start + sensor_range_max * cos(theta + ray_angle)
        //    y_end = y_start + sensor_range_max * sin(theta + ray_angle)
        double ray_angle = (degree_increment * idx) * M_PI / 180.0;
        double x_end = x_start + sensor_range_max * std::cos(theta + ray_angle);
        double y_end = y_start + sensor_range_max * std::sin(theta + ray_angle);

        // ── 5. Find the closest wall intersection ────────────────────────────
        //    Mirrors: self._map.check_collision(ray, compute_distance=True)
        double min_dist = std::numeric_limits<double>::infinity();
        bool   found    = false;

        for (const auto &seg : map_segments) {
            // seg[0] = {x2, y2},  seg[1] = {x3, y3}
            double xi, yi;
            if (segment_intersect(xi, yi,
                                  x_start, y_start, x_end, y_end,
                                  seg[0][0], seg[0][1],
                                  seg[1][0], seg[1][1]))
            {
                double dx   = xi - x_start;
                double dy   = yi - y_start;
                double dist = std::sqrt(dx * dx + dy * dy);
                if (dist < min_dist) {
                    min_dist = dist;
                    found    = true;
                }
            }
        }

        // ── 6. Append result (NaN if no intersection found) ──────────────────
        z_hat.push_back(found ? min_dist
                               : std::numeric_limits<double>::quiet_NaN());
    }

    return z_hat;
}