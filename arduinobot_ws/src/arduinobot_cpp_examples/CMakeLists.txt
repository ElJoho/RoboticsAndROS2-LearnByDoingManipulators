cmake_minimum_required(VERSION 3.8)
project(arduinobot_cpp_examples) # <--- PROJECT_NAME

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. DEPENDENCIAS
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(arduinobot_msgs REQUIRED)
# 2. EJECUTABLES
add_executable(simple_publisher src/simple_publisher.cpp)
add_executable(simple_subscriber src/simple_subscriber.cpp)
add_executable(simple_parameter src/simple_parameter.cpp)
add_executable(simple_service_server src/simple_service_server.cpp)
# 3. DEPENDENCIAS
ament_target_dependencies(simple_publisher rclcpp std_msgs)
ament_target_dependencies(simple_subscriber  rclcpp std_msgs)
ament_target_dependencies(simple_parameter  rclcpp rcl_interfaces)
ament_target_dependencies(simple_service_server rclcpp arduinobot_msgs)
# 4. INSTALACION
install(TARGETS
  simple_publisher
  simple_subscriber
  simple_parameter
  simple_service_server
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
