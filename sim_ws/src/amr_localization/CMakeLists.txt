cmake_minimum_required(VERSION 3.8)
project(amr_localization)

# Encontrar paquetes necesarios
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

find_package(pybind11_vendor REQUIRED)
find_package(pybind11 REQUIRED)
find_package(Eigen3 REQUIRED)

# Instalar los scripts de Python (lo que hacía tu setup.py)
ament_python_install_package(${PROJECT_NAME})
# Instalar los nodos ejecutables
install(PROGRAMS
  amr_localization/particle_filter_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME particle_filter
)
# Instalar otras carpetas (maps, images, etc si es necesario)
install(DIRECTORY maps images resource
  DESTINATION share/${PROJECT_NAME}
)

# --- AQUI COMPILAMOS EL C++ ---
pybind11_add_module(amr_localization_cpp src/particle_filter_cpp.cpp)
target_include_directories(amr_localization_cpp PRIVATE ${EIGEN3_INCLUDE_DIRS})
target_compile_features(amr_localization_cpp PRIVATE cxx_std_17)
target_compile_options(amr_localization_cpp PRIVATE -O3 -ffast-math -march=native)

pybind11_add_module(maps_cpp src/maps_cpp.cpp)
target_compile_features(maps_cpp PRIVATE cxx_std_17)
target_compile_options(maps_cpp PRIVATE -O3 -ffast-math -march=native)

ament_get_python_install_dir(PYTHON_INSTALL_DIR)

# Instalar la librería generada junto al código python
install(TARGETS amr_localization_cpp maps_cpp
  DESTINATION "${PYTHON_INSTALL_DIR}/amr_localization"
)

ament_package()