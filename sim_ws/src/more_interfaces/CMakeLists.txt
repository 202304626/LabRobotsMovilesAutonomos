# Versión mínima de CMake requerida para compilar este paquete
cmake_minimum_required(VERSION 3.8)

# Nombre del proyecto / paquete
project(more_interfaces)

# Si el compilador es GCC o Clang, añade opciones extra de advertencias
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Busca el paquete base de ament (sistema de build de ROS 2)
find_package(ament_cmake REQUIRED)

# Busca el generador de interfaces (mensajes, servicios, acciones)
find_package(rosidl_default_generators REQUIRED)

# Lista de archivos de mensajes que se van a generar
set(msg_files
  "msg/KeyTouched.msg"
)

# Genera automáticamente el código necesario para los mensajes
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)

# Si está habilitada la compilación de tests...
if(BUILD_TESTING)

  # Busca herramientas automáticas de análisis de estilo
  find_package(ament_lint_auto REQUIRED)

  # Desactiva la comprobación de copyright
  set(ament_cmake_copyright_FOUND TRUE)

  # Desactiva la comprobación cpplint
  set(ament_cmake_cpplint_FOUND TRUE)

  # Añade dependencias necesarias para los tests
  ament_lint_auto_find_test_dependencies()

endif()

# Exporta la dependencia para que otros paquetes que usen este
# sepan que necesitan rosidl_default_runtime
ament_export_dependencies(rosidl_default_runtime)

# Marca este paquete como un paquete válido de ROS 2
ament_package()
