#include <cmath>
#include <limits>
#include <vector>
#include <random>

// ─────────────────────────────────────────────────────────────────────────────
//  Geometry helpers duplicated from sense.cpp for self-containment
// ─────────────────────────────────────────────────────────────────────────────
static double round6(double x) { return std::round(x * 1e6) / 1e6; }
static double compute_slope(double x0, double y0, double x1, double y1) {
    if (std::abs(x1 - x0) < 1e-6) return std::numeric_limits<double>::infinity();
    return (y1 - y0) / (x1 - x0);
}
static double compute_intercept(double slope, double x, double y) {
    return y - slope * x;
}
static bool segment_intersect(
    double &xi, double &yi,
    double x0, double y0, double x1, double y1,
    double x2, double y2, double x3, double y3)
{
    xi = std::numeric_limits<double>::quiet_NaN();
    yi = std::numeric_limits<double>::quiet_NaN();

    double m1 = compute_slope(x0, y0, x1, y1);
    double m2 = compute_slope(x2, y2, x3, y3);

    if (!std::isinf(m1) && !std::isinf(m2)) {
        if (std::abs(m1 - m2) < 1e-6) return false;
        double b1 = compute_intercept(m1, x0, y0);
        double b2 = compute_intercept(m2, x2, y2);
        xi = (b2 - b1) / (m1 - m2);
        yi = m1 * xi + b1;
    } else if (std::isinf(m1) && !std::isinf(m2)) {
        double b2 = compute_intercept(m2, x2, y2);
        xi = x0;
        yi = m2 * xi + b2;
    } else if (std::isinf(m2) && !std::isinf(m1)) {
        double b1 = compute_intercept(m1, x0, y0);
        xi = x2;
        yi = m1 * xi + b1;
    } else {
        return false;
    }

    if (std::isnan(xi) || std::isnan(yi)) return false;

    double xri = round6(xi), yri = round6(yi);
    double xr0 = round6(x0), xr1 = round6(x1);
    double yr0 = round6(y0), yr1 = round6(y1);
    double xr2 = round6(x2), xr3 = round6(x3);
    double yr2 = round6(y2), yr3 = round6(y3);

    if (xr0 < xr1) { if (xri < xr0 || xri > xr1) return false; }
    else           { if (xri > xr0 || xri < xr1) return false; }
    if (yr0 < yr1) { if (yri < yr0 || yri > yr1) return false; }
    else           { if (yri > yr0 || yri < yr1) return false; }
    if (xr2 < xr3) { if (xri < xr2 || xri > xr3) return false; }
    else           { if (xri > xr2 || xri < xr3) return false; }
    if (yr2 < yr3) { if (yri < yr2 || yri > yr3) return false; }
    else           { if (yri > yr2 || yri < yr3) return false; }

    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Main move function
// ─────────────────────────────────────────────────────────────────────────────
std::vector<std::vector<double>> move_cpp(
    const std::vector<std::vector<double>>& particles,
    double v,
    double w,
    double dt,
    double sigma_v,
    double sigma_w,
    bool simulation,
    const std::vector<std::vector<std::vector<double>>>& map_segments)
{
    int n_particles = particles.size();
    std::vector<std::vector<double>> new_particles(n_particles, std::vector<double>(3));

    std::mt19937 rng(std::random_device{}());
    std::normal_distribution<double> dist_v(0.0, sigma_v);
    std::normal_distribution<double> dist_w(0.0, sigma_w);

    for (int i = 0; i < n_particles; ++i) {
        double v_gauss = v + dist_v(rng);
        double w_gauss = w + dist_w(rng);

        double x_prev = particles[i][0];
        double y_prev = particles[i][1];
        double theta_prev = particles[i][2];

        double x_new = x_prev + v_gauss * std::cos(theta_prev) * dt;
        double y_new = y_prev + v_gauss * std::sin(theta_prev) * dt;
        
        double theta_new_unmod;
        if (simulation) {
            // calulate theta and normalize so that the value is between [0,2pi]
            theta_new_unmod = theta_prev - w_gauss * dt;
        } else {
            theta_new_unmod = theta_prev + w_gauss * dt;
        }
        
        double theta_new = std::fmod(theta_new_unmod, 2.0 * M_PI);
        if (theta_new < 0) {
            theta_new += 2.0 * M_PI;
        }

        // Collision check
        double min_dist = std::numeric_limits<double>::infinity();
        double best_xi = x_new, best_yi = y_new;

        for (const auto& seg : map_segments) {
            double xi, yi;
            if (segment_intersect(xi, yi,
                                  x_prev, y_prev, x_new, y_new,
                                  seg[0][0], seg[0][1],
                                  seg[1][0], seg[1][1])) {
                double dx = xi - x_prev;
                double dy = yi - y_prev;
                double dist = std::sqrt(dx * dx + dy * dy);
                if (dist < min_dist) {
                    min_dist = dist;
                    best_xi = xi;
                    best_yi = yi;
                }
            }
        }

        new_particles[i][0] = best_xi;
        new_particles[i][1] = best_yi;
        new_particles[i][2] = theta_new;
    }

    return new_particles;
}
