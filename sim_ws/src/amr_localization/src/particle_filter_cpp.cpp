#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <random>
#include <cmath>
#include <vector>
#include <limits>
#include <algorithm>

namespace py = pybind11;

// --- ESTRUCTURAS DE DATOS ---
struct Point {
    double x, y;
};

struct Segment {
    Point p1, p2;
    // OPTIMIZACIÓN 1: Precalcular Bounding Box de las paredes
    double min_x, max_x, min_y, max_y; 
};

std::vector<Segment> global_map_segments;

// --- FUNCIONES MATEMÁTICAS OPTIMIZADAS ---
inline int orientation(const Point& p, const Point& q, const Point& r) {
    double val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
    if (val == 0.0) return 0; 
    return (val > 0.0) ? 1 : 2; 
}

// Intersección ultra-rápida asumiendo que ya pasamos el AABB
inline bool segment_intersect_fast(const Point& p1, const Point& q1, const Point& p2, const Point& q2, Point& out_intersection) {
    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);

    if (o1 != o2 && o3 != o4) {
        double a1 = q1.y - p1.y;
        double b1 = p1.x - q1.x;
        double c1 = a1 * p1.x + b1 * p1.y;

        double a2 = q2.y - p2.y;
        double b2 = p2.x - q2.x;
        double c2 = a2 * p2.x + b2 * p2.y;

        double determinant = a1 * b2 - a2 * b1;

        if (determinant != 0.0) {
            out_intersection.x = (b2 * c1 - b1 * c2) / determinant;
            out_intersection.y = (a1 * c2 - a2 * c1) / determinant;
            return true;
        }
    }
    return false; 
}

// --- 1. INITIALIZE MAP ---
void init_map_segments(py::array_t<double>& map_array) {
    global_map_segments.clear();
    py::buffer_info buf = map_array.request();
    double* ptr = static_cast<double*>(buf.ptr);
    int n_segments = buf.shape[0];

    for (int i = 0; i < n_segments; ++i) {
        Segment seg;
        seg.p1.x = ptr[i * 4 + 0];
        seg.p1.y = ptr[i * 4 + 1];
        seg.p2.x = ptr[i * 4 + 2];
        seg.p2.y = ptr[i * 4 + 3];
        
        // Cachear límites
        seg.min_x = std::min(seg.p1.x, seg.p2.x);
        seg.max_x = std::max(seg.p1.x, seg.p2.x);
        seg.min_y = std::min(seg.p1.y, seg.p2.y);
        seg.max_y = std::max(seg.p1.y, seg.p2.y);
        
        global_map_segments.push_back(seg);
    }
}

// --- 2. MOVE AND COLLIDE ---
void move_and_collide_particles(
    py::array_t<double>& particles_array, 
    double v, double w, double dt, 
    double sigma_v, double sigma_w) 
{
    py::buffer_info buf = particles_array.request();
    double* ptr = static_cast<double*>(buf.ptr);
    int n = buf.shape[0];

    py::gil_scoped_release release;

    // OPTIMIZACIÓN 4: RNG Thread-Local para paralelizar colisiones con OpenMP
    #pragma omp parallel
    {
        static thread_local std::mt19937 gen(std::random_device{}());
        std::normal_distribution<double> dist_v(0.0, sigma_v);
        std::normal_distribution<double> dist_w(0.0, sigma_w);

        #pragma omp for schedule(dynamic)
        for (int i = 0; i < n; ++i) {
            double x_o = ptr[i * 3 + 0];
            double y_o = ptr[i * 3 + 1];
            double theta_o = ptr[i * 3 + 2];

            double v_eff = v + dist_v(gen);
            double w_eff = w + dist_w(gen);

            double new_x = x_o + std::cos(theta_o) * v_eff * dt;
            double new_y = y_o + std::sin(theta_o) * v_eff * dt;
            double new_theta = theta_o + w_eff * dt;

            new_theta = std::fmod(new_theta, 2.0 * M_PI);
            if (new_theta < 0) new_theta += 2.0 * M_PI;

            Point start_point = {x_o, y_o};
            Point end_point = {new_x, new_y};
            
            double ray_min_x = std::min(x_o, new_x);
            double ray_max_x = std::max(x_o, new_x);
            double ray_min_y = std::min(y_o, new_y);
            double ray_max_y = std::max(y_o, new_y);

            bool collided = false;
            Point closest_intersection = {0, 0};
            double min_dist_sq = std::numeric_limits<double>::max();

            for (const auto& map_seg : global_map_segments) {
                // AABB Fast Rejection
                if (ray_max_x < map_seg.min_x || ray_min_x > map_seg.max_x ||
                    ray_max_y < map_seg.min_y || ray_min_y > map_seg.max_y) continue;

                Point intersection;
                if (segment_intersect_fast(start_point, end_point, map_seg.p1, map_seg.p2, intersection)) {
                    // OPTIMIZACIÓN 3: dx*dx en vez de std::pow
                    double dx = intersection.x - x_o;
                    double dy = intersection.y - y_o;
                    double dist_sq = dx * dx + dy * dy;
                    if (dist_sq < min_dist_sq) {
                        min_dist_sq = dist_sq;
                        closest_intersection = intersection;
                        collided = true;
                    }
                }
            }

            if (collided) {
                ptr[i * 3 + 0] = closest_intersection.x;
                ptr[i * 3 + 1] = closest_intersection.y;
            } else {
                ptr[i * 3 + 0] = new_x;
                ptr[i * 3 + 1] = new_y;
            }
            ptr[i * 3 + 2] = new_theta;
        }
    }
}

// --- 3. BATCH RAYCASTING ---
py::array_t<double> batch_raycasting(
    py::array_t<double>& particles_array,
    std::vector<int> ray_indices,
    double degree_increment,
    double sensor_offset_x,
    double max_range) 
{
    py::buffer_info buf = particles_array.request();
    double* ptr = static_cast<double*>(buf.ptr);
    int n_particles = buf.shape[0];
    int n_rays = ray_indices.size();

    auto result = py::array_t<double>({n_particles, n_rays});
    double* res_ptr = static_cast<double*>(result.request().ptr);

    // OPTIMIZACIÓN 2: Pre-calcular la trigonometría local de los rayos
    std::vector<double> cos_alpha(n_rays);
    std::vector<double> sin_alpha(n_rays);
    for (int j = 0; j < n_rays; ++j) {
        double alpha = ray_indices[j] * degree_increment * M_PI / 180.0;
        cos_alpha[j] = std::cos(alpha);
        sin_alpha[j] = std::sin(alpha);
    }

    py::gil_scoped_release release;

    #pragma omp parallel for schedule(dynamic)
    for (int i = 0; i < n_particles; ++i) {
        double x = ptr[i * 3 + 0];
        double y = ptr[i * 3 + 1];
        double theta = ptr[i * 3 + 2];

        // Calculamos trigonometría del robot UNA vez
        double cos_th = std::cos(theta);
        double sin_th = std::sin(theta);

        double l_x = x + sensor_offset_x * cos_th;
        double l_y = y + sensor_offset_x * sin_th;
        Point origin = {l_x, l_y};

        for (int j = 0; j < n_rays; ++j) {
            // Identidades trigonométricas: sin(a+b) y cos(a+b)
            // ¡Cero llamadas a std::cos / std::sin dentro de este bucle masivo!
            double ray_dx = cos_th * cos_alpha[j] - sin_th * sin_alpha[j];
            double ray_dy = sin_th * cos_alpha[j] + cos_th * sin_alpha[j];

            Point end = { origin.x + ray_dx * max_range, origin.y + ray_dy * max_range };
            
            double ray_min_x = std::min(origin.x, end.x);
            double ray_max_x = std::max(origin.x, end.x);
            double ray_min_y = std::min(origin.y, end.y);
            double ray_max_y = std::max(origin.y, end.y);

            double min_dist_sq = max_range * max_range;
            bool hit = false;

            for (const auto& seg : global_map_segments) {
                // AABB Fast Rejection para rayos
                if (ray_max_x < seg.min_x || ray_min_x > seg.max_x ||
                    ray_max_y < seg.min_y || ray_min_y > seg.max_y) continue;

                Point intersection;
                if (segment_intersect_fast(origin, end, seg.p1, seg.p2, intersection)) {
                    double dx = intersection.x - origin.x;
                    double dy = intersection.y - origin.y;
                    double dist_sq = dx * dx + dy * dy;
                    if (dist_sq < min_dist_sq) { 
                        min_dist_sq = dist_sq; 
                        hit = true; 
                    }
                }
            }
            res_ptr[i * n_rays + j] = hit ? std::sqrt(min_dist_sq) : std::numeric_limits<double>::quiet_NaN();
        }
    }
    return result;
}

// --- 4. COMPUTE WEIGHTS ---
void compute_weights(
    py::array_t<double> probabilities, 
    py::array_t<double> z_real, 
    py::array_t<double> all_z_hat, 
    double sigma_z, 
    double sensor_range_min) 
{
    // Usamos raw pointers para saltarnos las verificaciones de pybind11 (máxima velocidad)
    double* p_out = static_cast<double*>(probabilities.request().ptr);
    double* z_r = static_cast<double*>(z_real.request().ptr);
    double* z_sim = static_cast<double*>(all_z_hat.request().ptr);
    
    int num_particles = probabilities.shape(0);
    int num_rays = z_real.shape(0);
    
    double var_2 = 2.0 * sigma_z * sigma_z;
    double log_denominator = std::log(sigma_z * std::sqrt(2.0 * M_PI));
    double max_log_prob = -1e300; 

    py::gil_scoped_release release;

    // OPTIMIZACIÓN 5: OpenMP Reduction para encontrar el máximo global
    #pragma omp parallel for reduction(max: max_log_prob)
    for (int i = 0; i < num_particles; ++i) {
        double log_total_prob = 0.0;
        for (int j = 0; j < num_rays; ++j) {
            double real_val = std::isnan(z_r[j]) ? sensor_range_min : z_r[j];
            double sim_val = std::isnan(z_sim[i * num_rays + j]) ? sensor_range_min : z_sim[i * num_rays + j];
            double diff = sim_val - real_val;
            
            log_total_prob += -(diff * diff) / var_2 - log_denominator;
        }
        p_out[i] = log_total_prob; 
        if (log_total_prob > max_log_prob) {
            max_log_prob = log_total_prob;
        }
    }

    // Segunda reducción para sumar las probabilidades exponenciadas
    double sum_prob = 0.0;
    #pragma omp parallel for reduction(+: sum_prob)
    for (int i = 0; i < num_particles; ++i) {
        p_out[i] = std::exp(p_out[i] - max_log_prob);
        sum_prob += p_out[i];
    }
    
    // Normalización
    if (sum_prob > 0.0) {
        #pragma omp parallel for
        for (int i = 0; i < num_particles; ++i) p_out[i] /= sum_prob;
    } else {
        #pragma omp parallel for
        for (int i = 0; i < num_particles; ++i) p_out[i] = 1.0 / num_particles;
    }
}

// --- 5. RESAMPLE PARTICLES ---
void resample_particles(
    py::array_t<double> particles, 
    py::array_t<double> probabilities) 
{
    // Raw pointers para cero overhead de Python
    double* part = static_cast<double*>(particles.request().ptr);
    double* prob = static_cast<double*>(probabilities.request().ptr);
    int N = probabilities.shape(0);
    
    // Este algoritmo (Low Variance Resampling) es secuencial por naturaleza,
    // pero al usar raw pointers, vuela en un solo núcleo.
    std::vector<double> new_x(N), new_y(N), new_th(N);
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dis(0.0, 1.0 / N);
    
    double r = dis(gen);
    double c = prob[0];
    int i = 0;
    
    for (int m = 0; m < N; ++m) {
        double U = r + (double)m / N;
        while (U > c && i < N - 1) {
            i++;
            c += prob[i];
        }
        new_x[m] = part[i * 3 + 0];
        new_y[m] = part[i * 3 + 1];
        new_th[m] = part[i * 3 + 2];
    }
    
    for (int j = 0; j < N; ++j) {
        part[j * 3 + 0] = new_x[j];
        part[j * 3 + 1] = new_y[j];
        part[j * 3 + 2] = new_th[j];
    }
}

PYBIND11_MODULE(amr_localization_cpp, m) {
    m.def("init_map_segments", &init_map_segments, "Carga mapa y cachea AABB");
    m.def("move_and_collide_particles", &move_and_collide_particles, "Cinemática y colisión con OpenMP");
    m.def("batch_raycasting", &batch_raycasting, "Raycasting ultrarrápido con Math Caching");
    m.def("compute_weights", &compute_weights, "Cálculo de pesos multihilo");
    m.def("resample_particles", &resample_particles, "Low Variance Resampling C-Speed");
}