cmake_minimum_required(VERSION 3.16)
project(ly_bag_recorder LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(keyboard_handler REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_transport REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(ly_bag_recorder src/main.cpp)
target_compile_features(ly_bag_recorder PRIVATE cxx_std_17)
target_compile_options(ly_bag_recorder PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(ly_bag_recorder
  ament_index_cpp keyboard_handler rclcpp rmw rosbag2_cpp rosbag2_transport rosbag2_storage)
target_link_libraries(ly_bag_recorder yaml-cpp)

install(TARGETS ly_bag_recorder DESTINATION lib/${PROJECT_NAME})
# 此处只安装静态副本；运行时从ament位置查找配置，不编译进源码绝对路径。
install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/../../../config/rosbag.yaml"
  DESTINATION share/${PROJECT_NAME}/config)
install(FILES README.md DESTINATION share/${PROJECT_NAME})
ament_package()
