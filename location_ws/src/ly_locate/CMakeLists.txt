cmake_minimum_required(VERSION 3.16)
project(ly_locate LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

# ROS 接口、点云配准、位姿图与持久化依赖。
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ly_lio REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters registration kdtree io)
find_package(GTSAM REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LoopConstraint.msg"
  "msg/Keyframe.msg"
  "msg/KeyframePose.msg"
  "msg/LoopUpdate.msg"
  "srv/GetKeyframe.srv"
  DEPENDENCIES
    std_msgs
    builtin_interfaces
    geometry_msgs
    sensor_msgs
)
rosidl_get_typesupport_target(typesupport ${PROJECT_NAME} "rosidl_typesupport_cpp")

# 单独编译 GTSAM，避免同一翻译单元混入 PCL 的第三方 Eigen 头。
add_library(ly_pose_graph STATIC src/pose_graph.cpp)
target_include_directories(ly_pose_graph PUBLIC include)
target_link_libraries(ly_pose_graph PUBLIC gtsam Eigen3::Eigen)

foreach(node IN ITEMS loop map)
  add_executable(${node}_node src/${node}.cpp)
  target_include_directories(${node}_node PRIVATE include ${PCL_INCLUDE_DIRS})
  target_link_libraries(${node}_node
    ${typesupport}
    ${PCL_LIBRARIES}
    Eigen3::Eigen
    yaml-cpp
    Threads::Threads
  )
  # 只链接会话消息接口，不引入里程计可选的 Livox 驱动依赖。
  target_link_libraries(${node}_node ly_lio::ly_lio__rosidl_typesupport_cpp)
  ament_target_dependencies(${node}_node
    rclcpp
    ament_index_cpp
    sensor_msgs
    std_msgs
    geometry_msgs
    nav_msgs
    std_srvs
    tf2_ros
  )
endforeach()
target_link_libraries(loop_node ly_pose_graph)

install(TARGETS loop_node map_node RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES README.md LICENSE DESTINATION share/${PROJECT_NAME})
ament_export_dependencies(rosidl_default_runtime)
ament_package()
