#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <cmath>
#include <algorithm>
#include <limits>
#include <tuple>

namespace py = pybind11;

// --- FUNCIONES AUXILIARES ULTRARRÁPIDAS ---

// Normalización de ángulo sin usar el costoso std::fmod
inline double normalize_angle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

// --- WALL FOLLOWER ---

inline double safe_min(const double* r, int N, int start, int end) {
    double min_val = 8.0;
    for (int i = start; i < end; ++i) {
        // OPTIMIZACIÓN 1: Evitar el operador módulo (%) con condicionales simples
        int idx = i;
        if (idx < 0) idx += N;
        else if (idx >= N) idx -= N;

        double val = r[idx];
        
        // OPTIMIZACIÓN 2: isfinite verifica NaN e Inf simultáneamente
        if (std::isfinite(val) && val < min_val) {
            min_val = val;
        }
    }
    return min_val;
}

std::tuple<double, double, double> compute_wall_distances(py::array_t<double>& ranges) {
    py::buffer_info buf = ranges.request();
    double* r_ptr = static_cast<double*>(buf.ptr);
    int N = buf.shape[0];
    
    if (N == 0) return {8.0, 8.0, 8.0};
    
    // Soltamos GIL para no bloquear Python
    py::gil_scoped_release release;
    
    double front_1 = safe_min(r_ptr, N, 0, 5);
    double front_2 = safe_min(r_ptr, N, N - 5, N);
    double front_dist = std::min(front_1, front_2);
    
    double right_dist = safe_min(r_ptr, N, (3 * N / 4) - 5, (3 * N / 4) + 5);
    double left_dist = safe_min(r_ptr, N, (N / 4) - 5, (N / 4) + 5);
    
    return {front_dist, right_dist, left_dist};
}

// --- PURE PURSUIT ---

std::tuple<double, double, int, bool> compute_pure_pursuit(
    double x, double y, double theta,
    py::array_t<double>& path,
    int last_closest_idx,
    double lookahead_distance,
    double v_max, double v_min, double w_max,
    bool is_aligned) 
{
    py::buffer_info buf = path.request();
    double* p = static_cast<double*>(buf.ptr);
    int N = buf.shape[0];
    
    if (N == 0) return {0.0, 0.0, 0, false};

    // Soltamos GIL para rendimiento multihilo real en el lado de C++
    py::gil_scoped_release release;

    // 1. Buscar closest point (búsqueda local basada en punteros crudos)
    int start_idx = std::max(0, last_closest_idx - 5);
    int end_idx = std::min(N, last_closest_idx + 30);
    
    int closest_idx = start_idx;
    double min_sq_dist = std::numeric_limits<double>::max();
    
    for (int i = start_idx; i < end_idx; ++i) {
        // Acceso a memoria 1D ultra-rápido: p[fila * columnas + columna]
        double dx = p[i * 2 + 0] - x;
        double dy = p[i * 2 + 1] - y;
        double sq_dist = dx * dx + dy * dy;
        
        if (sq_dist < min_sq_dist) {
            min_sq_dist = sq_dist;
            closest_idx = i;
        }
    }

    // 2. Buscar target point a partir del closest_idx
    double lookahead_sqr = lookahead_distance * lookahead_distance;
    double target_x = p[(N - 1) * 2 + 0];
    double target_y = p[(N - 1) * 2 + 1];
    
    for (int i = closest_idx; i < N; ++i) {
        double px = p[i * 2 + 0];
        double py = p[i * 2 + 1];
        double dx = px - x;
        double dy = py - y;
        
        if (dx * dx + dy * dy >= lookahead_sqr) {
            target_x = px;
            target_y = py;
            break;
        }
    }

    // 3. Matemáticas del Pure Pursuit
    double vx = target_x - x;
    double vy = target_y - y;
    double l = std::sqrt(vx * vx + vy * vy);
    
    double raw_alpha = std::atan2(vy, vx) - theta;
    
    // OPTIMIZACIÓN 3: Normalización de ángulo nativa
    double alpha = normalize_angle(raw_alpha);
    
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
    
    w = (l > 0.0) ? (v * 2.0 * std::sin(alpha) / l) : 0.0;
    w = std::max(-w_max, std::min(w, w_max));
    
    return {v, w, closest_idx, is_aligned};
}

PYBIND11_MODULE(amr_control_cpp, m) {
    m.def("compute_wall_distances", &compute_wall_distances, "Extrae distancias del lidar optimizado");
    m.def("compute_pure_pursuit", &compute_pure_pursuit, "Ejecuta pure pursuit sin GIL y con punteros crudos");
}