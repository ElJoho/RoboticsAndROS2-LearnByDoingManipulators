cmake_minimum_required(VERSION 3.8)
project(arduinobot_description) # <-- PROJECT_NAME

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1 Librerias
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)

# Instalar recursos del paquete
install(
  DIRECTORY meshes urdf launch rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
