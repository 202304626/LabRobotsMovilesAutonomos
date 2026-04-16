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
};

// --- VARIABLE GLOBAL PARA EL MAPA ---
// Esto almacena el mapa en la memoria de C++ una vez inicializado
std::vector<Segment> global_map_segments;

// --- FUNCIONES MATEMÁTICAS ---

// Función auxiliar para determinar la orientación de tres puntos
int orientation(Point p, Point q, Point r) {
    double val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
    if (std::abs(val) < 1e-9) return 0; // Colineales
    return (val > 0) ? 1 : 2; // Reloj o anti-reloj
}

// Verifica si el punto q se encuentra sobre el segmento pr
bool on_segment(Point p, Point q, Point r) {
    if (q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) &&
        q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y)) {
        return true;
    }
    return false;
}

// Calcula la intersección de dos segmentos
bool segment_intersect(Point p1, Point q1, Point p2, Point q2, Point& out_intersection) {
    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);

    // Caso general
    if (o1 != o2 && o3 != o4) {
        // Calcular el punto exacto de intersección usando álgebra lineal
        double a1 = q1.y - p1.y;
        double b1 = p1.x - q1.x;
        double c1 = a1 * p1.x + b1 * p1.y;

        double a2 = q2.y - p2.y;
        double b2 = p2.x - q2.x;
        double c2 = a2 * p2.x + b2 * p2.y;

        double determinant = a1 * b2 - a2 * b1;

        if (std::abs(determinant) > 1e-9) {
            out_intersection.x = (b2 * c1 - b1 * c2) / determinant;
            out_intersection.y = (a1 * c2 - a2 * c1) / determinant;
            return true;
        }
    }

    return false; // Ignoramos colinealidad especial para simplificar el filtro
}

// --- FUNCIONES EXPUESTAS A PYTHON ---

// 1. Inicializar el mapa en la memoria C++
void init_map_segments(py::array_t<double>& map_array) {
    global_map_segments.clear();
    py::buffer_info buf = map_array.request();
    double* ptr = static_cast<double*>(buf.ptr);
    int n_segments = buf.shape[0];

    for (int i = 0; i < n_segments; ++i) {
        Segment seg;
        // Asumiendo que python pasa un array Nx4: [x1, y1, x2, y2]
        seg.p1.x = ptr[i * 4 + 0];
        seg.p1.y = ptr[i * 4 + 1];
        seg.p2.x = ptr[i * 4 + 2];
        seg.p2.y = ptr[i * 4 + 3];
        global_map_segments.push_back(seg);
    }
}

// 2. Movimiento + Colisión integrado (¡Máxima velocidad!)
void move_and_collide_particles(
    py::array_t<double>& particles_array, 
    double v, double w, double dt, 
    double sigma_v, double sigma_w) 
{
    py::buffer_info buf = particles_array.request();
    double* ptr = static_cast<double*>(buf.ptr);
    int n = buf.shape[0];

    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<double> dist_v(0.0, sigma_v);
    std::normal_distribution<double> dist_w(0.0, sigma_w);

    for (int i = 0; i < n; ++i) {
        double x_o = ptr[i * 3 + 0];
        double y_o = ptr[i * 3 + 1];
        double theta_o = ptr[i * 3 + 2];

        double noise_v = dist_v(gen);
        double noise_w = dist_w(gen);
        double v_eff = v + noise_v;

        // Nueva posición teórica
        double new_x = x_o + std::cos(theta_o) * v_eff * dt;
        double new_y = y_o + std::sin(theta_o) * v_eff * dt;
        double new_theta = theta_o + (w + noise_w) * dt;

        // Normalizar theta
        new_theta = std::fmod(new_theta, 2.0 * M_PI);
        if (new_theta < 0) new_theta += 2.0 * M_PI;

        // Comprobación de colisiones en C++
        Point start_point = {x_o, y_o};
        Point end_point = {new_x, new_y};
        bool collided = false;
        Point closest_intersection = {0, 0};
        double min_dist = std::numeric_limits<double>::max();

        for (const auto& map_seg : global_map_segments) {
            Point intersection;
            if (segment_intersect(start_point, end_point, map_seg.p1, map_seg.p2, intersection)) {
                double dist = std::pow(intersection.x - start_point.x, 2) + 
                              std::pow(intersection.y - start_point.y, 2);
                if (dist < min_dist) {
                    min_dist = dist;
                    closest_intersection = intersection;
                    collided = true;
                }
            }
        }

        // Aplicar resultados
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

// ######## SENSE ########
double trace_ray_internal(Point start, double angle, double max_range) {
    Point end = { start.x + max_range * std::cos(angle), start.y + max_range * std::sin(angle) };
    double min_dist = max_range;
    bool hit = false;

    for (const auto& seg : global_map_segments) {
        Point intersection;
        if (segment_intersect(start, end, seg.p1, seg.p2, intersection)) {
            double d = std::sqrt(std::pow(intersection.x - start.x, 2) + std::pow(intersection.y - start.y, 2));
            if (d < min_dist) { min_dist = d; hit = true; }
        }
    }
    return hit ? min_dist : std::numeric_limits<double>::quiet_NaN();
}

// --- LA FUNCIÓN BATCH (Opción B) ---
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

    // Creamos la matriz de salida NxM (Partículas x Rayos)
    auto result = py::array_t<double>({n_particles, n_rays});
    double* res_ptr = static_cast<double*>(result.request().ptr);

    for (int i = 0; i < n_particles; ++i) {
        double x = ptr[i * 3 + 0];
        double y = ptr[i * 3 + 1];
        double theta = ptr[i * 3 + 2];

        double l_x = x + sensor_offset_x * std::cos(theta);
        double l_y = y + sensor_offset_x * std::sin(theta);
        Point origin = {l_x, l_y};

        for (int j = 0; j < n_rays; ++j) {
            double ray_angle = theta + (ray_indices[j] * degree_increment * M_PI / 180.0);
            res_ptr[i * n_rays + j] = trace_ray_internal(origin, ray_angle, max_range);
        }
    }
    return result;
}

void compute_weights(
    py::array_t<double> probabilities, 
    py::array_t<double> z_real, 
    py::array_t<double> all_z_hat, 
    double sigma_z, 
    double sensor_range_min) 
{
    auto p_out = probabilities.mutable_unchecked<1>();
    auto z_r = z_real.unchecked<1>();
    auto z_sim = all_z_hat.unchecked<2>();
    int num_particles = p_out.shape(0);
    int num_rays = z_r.shape(0);
    
    double var_2 = 2.0 * sigma_z * sigma_z;
    double log_denominator = std::log(sigma_z * std::sqrt(2.0 * M_PI));
    double max_log_prob = -1e300; // Un número muy negativo
    // 1. Calcular el logaritmo de la probabilidad para cada partícula
    for (int i = 0; i < num_particles; ++i) {
        double log_total_prob = 0.0;
        for (int j = 0; j < num_rays; ++j) {
            double real_val = std::isnan(z_r(j)) ? sensor_range_min : z_r(j);
            double sim_val = std::isnan(z_sim(i, j)) ? sensor_range_min : z_sim(i, j);
            double diff = sim_val - real_val;
            
            // Logaritmo de la gaussiana: ln( e^(-diff^2 / 2*sigma^2) / denom )
            // = (-diff^2 / 2*sigma^2) - ln(denom)
            double log_prob = -(diff * diff) / var_2 - log_denominator;
            log_total_prob += log_prob;
        }
        p_out(i) = log_total_prob; // Guardamos temporalmente el logaritmo
        
        // Encontrar el logaritmo máximo para estabilizar el paso de exponenciación
        if (log_total_prob > max_log_prob) {
            max_log_prob = log_total_prob;
        }
    }
    // 2. Aplicar el truco Log-Sum-Exp para evitar el underflow a cero.
    // Restamos el logaritmo máximo antes de exponenciar. La suma total se estabiliza.
    double sum_prob = 0.0;
    for (int i = 0; i < num_particles; ++i) {
        // Exponenciamos el valor estabilizado
        p_out(i) = std::exp(p_out(i) - max_log_prob);
        sum_prob += p_out(i);
    }
    
    // 3. Normalizar para que la suma sea 1.0 (o si falló gravemente, asignar uniforme)
    if (sum_prob > 0.0) {
        for (int i = 0; i < num_particles; ++i) {
            p_out(i) /= sum_prob;
        }
    } else {
        // Fallback de seguridad (no debería ocurrir con el log-sum-exp)
        for (int i = 0; i < num_particles; ++i) {
            p_out(i) = 1.0 / num_particles;
        }
    }
}
// 2. Ruleta Rusa: Low Variance Resampling
void resample_particles(
    py::array_t<double> particles, 
    py::array_t<double> probabilities) 
{
    auto part = particles.mutable_unchecked<2>(); 
    auto prob = probabilities.mutable_unchecked<1>();
    int N = prob.shape(0);
    double sum = 0.0;
    for (int i = 0; i < N; ++i) sum += prob(i);
    
    if (sum > 0.0) {
        for (int i = 0; i < N; ++i) prob(i) /= sum;
    } else {
        for (int i = 0; i < N; ++i) prob(i) = 1.0 / N;
    }
    std::vector<double> new_x(N), new_y(N), new_th(N);
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dis(0.0, 1.0 / N);
    double r = dis(gen);
    double c = prob(0);
    int i = 0;
    for (int m = 0; m < N; ++m) {
        double U = r + (double)m / N;
        while (U > c && i < N - 1) {
            i++;
            c += prob(i);
        }
        new_x[m] = part(i, 0);
        new_y[m] = part(i, 1);
        new_th[m] = part(i, 2);
    }
    for (int j = 0; j < N; ++j) {
        part(j, 0) = new_x[j];
        part(j, 1) = new_y[j];
        part(j, 2) = new_th[j];
    }
}

PYBIND11_MODULE(amr_localization_cpp, m) {
    m.def("init_map_segments", &init_map_segments, "Carga los segmentos del mapa en la memoria C++");
    m.def("move_and_collide_particles", &move_and_collide_particles, "Cinematica y colisiones fusionadas in-place");
    m.def("batch_raycasting", &batch_raycasting, "Raycasting masivo para todas las partículas");
    m.def("compute_weights", &compute_weights, "Compute particle weights using Gaussian probability");
    m.def("resample_particles", &resample_particles, "Perform low variance resampling on particles");
}