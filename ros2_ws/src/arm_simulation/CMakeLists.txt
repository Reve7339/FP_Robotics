cmake_minimum_required(VERSION 3.8)
project(arm_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Instalar directorios de recursos y archivos
install(
  DIRECTORY launch urdf config meshes
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES model.config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
