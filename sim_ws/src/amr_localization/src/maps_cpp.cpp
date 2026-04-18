#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <algorithm>

namespace py = pybind11;

struct Point {
    double x, y;
};

// 1. OPTIMIZACIÓN: Matemáticas de signo directas, sin llamadas a std::abs
inline int orientation(const Point& p, const Point& q, const Point& r) {
    double val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
    if (val == 0.0) return 0;
    return (val > 0.0) ? 1 : 2;
}

// 2. OPTIMIZACIÓN: AABB Fast-Rejection + Paso por referencia (const Point&)
inline bool segment_intersect(const Point& p1, const Point& q1, const Point& p2, const Point& q2) {
    // Descarte rápido: Si las cajas delimitadoras no se tocan, es imposible que choquen.
    if (std::max(p1.x, q1.x) < std::min(p2.x, q2.x) ||
        std::min(p1.x, q1.x) > std::max(p2.x, q2.x) ||
        std::max(p1.y, q1.y) < std::min(p2.y, q2.y) ||
        std::min(p1.y, q1.y) > std::max(p2.y, q2.y)) {
        return false;
    }

    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);
    
    return (o1 != o2 && o3 != o4); 
}

// ----------------------------------------------------
// BATCH CROSSES
// ----------------------------------------------------
py::array_t<bool> batch_crosses(
    py::array_t<double>& segments_array,
    py::array_t<double>& map_segments_array) 
{
    py::buffer_info seg_buf = segments_array.request();
    double* seg_ptr = static_cast<double*>(seg_buf.ptr);
    int n_segments = seg_buf.shape[0];

    py::buffer_info map_buf = map_segments_array.request();
    double* map_ptr = static_cast<double*>(map_buf.ptr);
    int n_map_segments = map_buf.shape[0];

    auto result = py::array_t<bool>(n_segments);
    bool* res_ptr = static_cast<bool*>(result.request().ptr);

    // 3. OPTIMIZACIÓN: Soltar el GIL de Python para maximizar rendimiento multihilo
    py::gil_scoped_release release;

    // 4. OPTIMIZACIÓN: OpenMP añadido aquí también
    #pragma omp parallel for schedule(dynamic)
    for (int i = 0; i < n_segments; ++i) {
        Point p1 = {seg_ptr[i * 4 + 0], seg_ptr[i * 4 + 1]};
        Point q1 = {seg_ptr[i * 4 + 2], seg_ptr[i * 4 + 3]};
        
        // Pre-calcular la "caja" del segmento actual evita repetirlo en el bucle interior
        double min_x1 = std::min(p1.x, q1.x);
        double max_x1 = std::max(p1.x, q1.x);
        double min_y1 = std::min(p1.y, q1.y);
        double max_y1 = std::max(p1.y, q1.y);

        bool crosses = false;
        for (int j = 0; j < n_map_segments; ++j) {
            Point p2 = {map_ptr[j * 4 + 0], map_ptr[j * 4 + 1]};
            Point q2 = {map_ptr[j * 4 + 2], map_ptr[j * 4 + 3]};
            
            // Fast-rejection AABB ultra-optimizado en línea
            if (max_x1 < std::min(p2.x, q2.x) || min_x1 > std::max(p2.x, q2.x) ||
                max_y1 < std::min(p2.y, q2.y) || min_y1 > std::max(p2.y, q2.y)) {
                continue; 
            }

            if (segment_intersect(p1, q1, p2, q2)) {
                crosses = true;
                break; 
            }
        }
        res_ptr[i] = crosses;
    }
    return result;
}

// ----------------------------------------------------
// BATCH CONTAINS
// ----------------------------------------------------
py::array_t<bool> batch_contains(
    py::array_t<double>& points_array,
    py::array_t<double>& map_segments_array) 
{
    py::buffer_info pt_buf = points_array.request();
    double* pt_ptr = static_cast<double*>(pt_buf.ptr);
    int n_points = pt_buf.shape[0];

    py::buffer_info map_buf = map_segments_array.request();
    double* map_ptr = static_cast<double*>(map_buf.ptr);
    int n_map_segments = map_buf.shape[0];

    auto result = py::array_t<bool>(n_points);
    bool* res_ptr = static_cast<bool*>(result.request().ptr);

    // Soltar el GIL de Python
    py::gil_scoped_release release;

    #pragma omp parallel for schedule(dynamic)
    for (int i = 0; i < n_points; ++i) {
        double px = pt_ptr[i * 2 + 0];
        double py = pt_ptr[i * 2 + 1];
        
        bool inside = false;
        for (int j = 0; j < n_map_segments; ++j) {
            // 5. OPTIMIZACIÓN: Lectura perezosa de memoria (Lazy fetching)
            // Solo cargamos el eje Y primero.
            double sy1 = map_ptr[j * 4 + 1];
            double sy2 = map_ptr[j * 4 + 3];
            
            if ((sy1 > py) != (sy2 > py)) {
                // Solo si pasamos el filtro Y, cargamos el eje X y calculamos
                double sx1 = map_ptr[j * 4 + 0];
                double sx2 = map_ptr[j * 4 + 2];
                
                if (px < (sx2 - sx1) * (py - sy1) / (sy2 - sy1) + sx1) {
                    inside = !inside;
                }
            }
        }
        res_ptr[i] = inside;
    }
    return result;
}

PYBIND11_MODULE(maps_cpp, m) {
    m.def("batch_crosses", &batch_crosses, "Intersección masiva C++ optimizada");
    m.def("batch_contains", &batch_contains, "Puntos masivos C++ optimizados");
}