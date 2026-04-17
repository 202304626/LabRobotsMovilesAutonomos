cmake_minimum_required(VERSION 3.8)
project(amr_control)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(pybind11_vendor REQUIRED)
find_package(pybind11 REQUIRED)
# Compilar la lógica de control en C++
pybind11_add_module(amr_control_cpp src/control_cpp.cpp)
target_compile_features(amr_control_cpp PRIVATE cxx_std_17)
target_compile_options(amr_control_cpp PRIVATE -O3 -ffast-math -march=native)
# Instalar módulos de Python
ament_python_install_package(${PROJECT_NAME})
# Instalar Nodos
install(PROGRAMS
  amr_control/pure_pursuit_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pure_pursuit
)
install(PROGRAMS
  amr_control/wall_follower_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME wall_follower
)
# Instalar la librería C++ junto a Python
ament_get_python_install_dir(PYTHON_INSTALL_DIR)
install(TARGETS amr_control_cpp
  DESTINATION "${PYTHON_INSTALL_DIR}/amr_control"
)
ament_package()
