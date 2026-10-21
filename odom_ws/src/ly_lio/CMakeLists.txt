cmake_minimum_required(VERSION 3.16)
project(ly_lio LANGUAGES C CXX)

# 固定维度矩阵与分支提示使用 C++20；默认 Release，保证逐点更新的运行效率。
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
option(LY_LIO_NATIVE_OPTIMIZATION "Optimize for the build machine CPU" OFF)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wno-unused-parameter)
  # liblzf 的展开复制分支有意贯穿；仅对原第三方 C 文件关闭此类警告。
  set_source_files_properties(3rdparty/liblzf/lzf_d.c
    PROPERTIES COMPILE_OPTIONS "-Wno-implicit-fallthrough")
  if(LY_LIO_NATIVE_OPTIMIZATION)
    add_compile_options(-march=native)
  endif()
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rcl REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcl_yaml_param_parser REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_srvs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)
# 标准 PointCloud2 模式不依赖 Livox；找到驱动时再启用自定义消息适配器。
find_package(livox_ros_driver2 QUIET)

rosidl_generate_interfaces(${PROJECT_NAME} "msg/ScanSession.msg" DEPENDENCIES std_msgs)
rosidl_get_typesupport_target(ly_lio_typesupport ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_library(ly_lio_core SHARED
  src/ly_lio/parameters.cpp
  src/ly_lio/preprocess.cpp
  src/ly_lio/estimator.cpp
  src/ly_lio/ly_lio.cpp
  src/util/voxelgrid_sampling.cpp
  src/util/pointcloud_mapping.cpp
  src/io/pcd_io.cpp
  3rdparty/liblzf/lzf_c.c
  3rdparty/liblzf/lzf_d.c)
target_compile_features(ly_lio_core PUBLIC cxx_std_20)
target_include_directories(ly_lio_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/3rdparty>
  $<INSTALL_INTERFACE:include/ly_lio>)
target_link_libraries(ly_lio_core Eigen3::Eigen OpenMP::OpenMP_CXX)
ament_target_dependencies(ly_lio_core rclcpp)
target_precompile_headers(ly_lio_core PRIVATE
  "$<$<COMPILE_LANGUAGE:CXX>:${CMAKE_CURRENT_SOURCE_DIR}/include/pch.h>")

add_library(ly_lio_component SHARED src/ly_lio_node.cpp src/terminal_dashboard.cpp)
target_link_libraries(ly_lio_component ly_lio_core yaml-cpp ${ly_lio_typesupport})
ament_target_dependencies(ly_lio_component
  ament_index_cpp rcl rclcpp rclcpp_components rcl_yaml_param_parser
  sensor_msgs geometry_msgs nav_msgs tf2_ros std_srvs)
if(livox_ros_driver2_FOUND)
  target_compile_definitions(ly_lio_component PRIVATE HAVE_LIVOX_DRIVER)
  ament_target_dependencies(ly_lio_component livox_ros_driver2)
endif()
rclcpp_components_register_node(ly_lio_component
  PLUGIN "ly_lio::LyLioNode"
  EXECUTABLE ly_lio_node
  EXECUTOR MultiThreadedExecutor)

install(TARGETS ly_lio_core ly_lio_component
  EXPORT ly_lioTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY src/ly_lio src/common src/util src/io
  DESTINATION include/${PROJECT_NAME} FILES_MATCHING PATTERN "*.h")
install(FILES include/pch.h DESTINATION include/${PROJECT_NAME})
install(DIRECTORY 3rdparty/ankerl 3rdparty/liblzf
  DESTINATION include/${PROJECT_NAME} FILES_MATCHING PATTERN "*.h")
install(DIRECTORY config launch docs DESTINATION share/${PROJECT_NAME})
install(FILES README.md LICENSE.txt THIRD_PARTY_NOTICES.md DESTINATION share/${PROJECT_NAME})
ament_export_targets(ly_lioTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp Eigen3 OpenMP)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
