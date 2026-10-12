cmake_minimum_required(VERSION 3.8)
project(pcl_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(pcl_publisher src/pcl_publisher.cpp)
target_compile_features(pcl_publisher PUBLIC cxx_std_17)
ament_target_dependencies(pcl_publisher ament_index_cpp rclcpp geometry_msgs sensor_msgs)
target_link_libraries(pcl_publisher Eigen3::Eigen yaml-cpp)

install(TARGETS pcl_publisher DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(FILES ../../config/lidar2baselink.yaml DESTINATION share/${PROJECT_NAME}/config)

ament_package()
