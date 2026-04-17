#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <cmath>
#include <vector>
#include <algorithm>
#include <limits>
#include <tuple>
namespace py = pybind11;
// --- WALL FOLLOWER ---
double safe_min(const py::array_t<double>& ranges, int start, int end) {
    auto r = ranges.unchecked<1>();
    double min_val = 8.0;
    int N = r.shape(0);
    for (int i = start; i < end; ++i) {
        int idx = (i % N + N) % N; // Manejar arrays circulares (índices negativos)
        double val = r(idx);
        if (!std::isnan(val) && !std::isinf(val) && val < min_val) {
            min_val = val;
        }
    }
    return min_val;
}
std::tuple<double, double, double> compute_wall_distances(py::array_t<double> ranges) {
    int N = ranges.shape(0);
    if (N == 0) return {8.0, 8.0, 8.0};

    // Frente: primeros 5 y últimos 5
    double front_1 = safe_min(ranges, 0, 5);
    double front_2 = safe_min(ranges, N - 5, N);
    double front_dist = std::min(front_1, front_2);

    // Derecha e Izquierda (indices adaptados de tu python)
    double right_dist = safe_min(ranges, (3 * N / 4) - 5, (3 * N / 4) + 5);
    double left_dist = safe_min(ranges, (N / 4) - 5, (N / 4) + 5);

    return {front_dist, right_dist, left_dist};
}
// --- PURE PURSUIT ---
std::tuple<double, double, int, bool> compute_pure_pursuit(
    double x, double y, double theta,
    py::array_t<double> path,
    int last_closest_idx,
    double lookahead_distance,
    double v_max, double v_min, double w_max,
    bool is_aligned)
{
    auto p = path.unchecked<2>();
    int N = p.shape(0);
    if (N == 0) return {0.0, 0.0, 0, false};
    // 1. Buscar closest point (buscamos localmente)
    int start_idx = std::max(0, last_closest_idx - 5);
    int end_idx = std::min(N, last_closest_idx + 30);

    int closest_idx = start_idx;
    double min_sq_dist = std::numeric_limits<double>::max();
    for (int i = start_idx; i < end_idx; ++i) {
        double dx = p(i, 0) - x;
        double dy = p(i, 1) - y;
        double sq_dist = dx*dx + dy*dy;
        if (sq_dist < min_sq_dist) {
            min_sq_dist = sq_dist;
            closest_idx = i;
        }
    }
    // 2. Buscar target point a partir del closest_idx
    double lookahead_sqr = lookahead_distance * lookahead_distance;
    double target_x = p(N-1, 0);
    double target_y = p(N-1, 1);

    for (int i = closest_idx; i < N; ++i) {
        double dx = p(i, 0) - x;
        double dy = p(i, 1) - y;
        if (dx*dx + dy*dy >= lookahead_sqr) {
            target_x = p(i, 0);
            target_y = p(i, 1);
            break;
        }
    }
    // 3. Matemáticas del Pure Pursuit
    double vx = target_x - x;
    double vy = target_y - y;
    double l = std::sqrt(vx*vx + vy*vy);

    double raw_alpha = std::atan2(vy, vx) - theta;
    // Normalizar a [-pi, pi]
    double alpha = std::fmod(raw_alpha + M_PI, 2.0 * M_PI);
    if (alpha < 0) alpha += 2.0 * M_PI;
    alpha -= M_PI;
    double max_angle = M_PI / 3.5;
    double v = 0.0;
    double w = 0.0;
    if (!is_aligned) {
        if (std::abs(alpha) > max_angle) {
            w = (alpha > 0) ? w_max : -w_max;
            return {0.0, w, closest_idx, false};
        } else {
            is_aligned = true;
        }
    }
    double v_desired = v_max * (1.0 - std::abs(alpha) / max_angle);
    v = std::max(v_min, std::min(v_desired, v_max));

    w = (l > 0) ? (v * 2.0 * std::sin(alpha) / l) : 0.0;
    w = std::max(-w_max, std::min(w, w_max));
    return {v, w, closest_idx, is_aligned};
}
PYBIND11_MODULE(amr_control_cpp, m) {
    m.def("compute_wall_distances", &compute_wall_distances, "Extrae distancias del lidar ultrarrápido");
    m.def("compute_pure_pursuit", &compute_pure_pursuit, "Ejecuta pure pursuit en C++");
}
