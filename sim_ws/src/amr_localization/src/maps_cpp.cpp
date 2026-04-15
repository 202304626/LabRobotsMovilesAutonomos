#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <cmath>
#include <algorithm>
namespace py = pybind11;
struct Point {
    double x, y;
};
// Matemáticas ultrarrápidas de intersección
int orientation(Point p, Point q, Point r) {
    double val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
    if (std::abs(val) < 1e-9) return 0;
    return (val > 0) ? 1 : 2;
}
bool segment_intersect(Point p1, Point q1, Point p2, Point q2) {
    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);
    
    if (o1 != o2 && o3 != o4) return true;
    return false; // Simplificado para acelerar colisiones
}
// ----------------------------------------------------
// BATCH CROSSES: Comprueba miles de líneas contra las paredes
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
    for (int i = 0; i < n_segments; ++i) {
        Point p1 = {seg_ptr[i * 4 + 0], seg_ptr[i * 4 + 1]};
        Point q1 = {seg_ptr[i * 4 + 2], seg_ptr[i * 4 + 3]};
        
        bool crosses = false;
        for (int j = 0; j < n_map_segments; ++j) {
            Point p2 = {map_ptr[j * 4 + 0], map_ptr[j * 4 + 1]};
            Point q2 = {map_ptr[j * 4 + 2], map_ptr[j * 4 + 3]};
            
            if (segment_intersect(p1, q1, p2, q2)) {
                crosses = true;
                break; // Cortocircuito: a la primera que choca, paramos
            }
        }
        res_ptr[i] = crosses;
    }
    return result;
}
// ----------------------------------------------------
// BATCH CONTAINS: Comprueba miles de puntos dentro del mapa libre
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
    for (int i = 0; i < n_points; ++i) {
        double px = pt_ptr[i * 2 + 0];
        double py = pt_ptr[i * 2 + 1];
        
        bool inside = false;
        // Matemáticas de Ray-Casting Point-in-Polygon
        for (int j = 0; j < n_map_segments; ++j) {
            double sx1 = map_ptr[j * 4 + 0];
            double sy1 = map_ptr[j * 4 + 1];
            double sx2 = map_ptr[j * 4 + 2];
            double sy2 = map_ptr[j * 4 + 3];
            
            if (((sy1 > py) != (sy2 > py)) &&
                (px < (sx2 - sx1) * (py - sy1) / (sy2 - sy1) + sx1)) {
                inside = !inside;
            }
        }
        res_ptr[i] = inside;
    }
    return result;
}
// El nombre del módulo en Python será "maps_cpp"
PYBIND11_MODULE(maps_cpp, m) {
    m.def("batch_crosses", &batch_crosses, "Intersección masiva C++");
    m.def("batch_contains", &batch_contains, "Puntos masivos C++");
}