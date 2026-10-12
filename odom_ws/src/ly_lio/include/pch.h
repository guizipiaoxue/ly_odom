#pragma once

#ifndef PCH_H
#define PCH_H

// 统一编译头：Eigen、稠密哈希和 OpenMP；无需 PCL/IKFoM/Open3D 运行时依赖。
// STL
#define _USE_MATH_DEFINES
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <cstring>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <limits>
#include <list>
#include <memory>
#include <queue>
#include <vector>
// Eigen
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <Eigen/Sparse>
// omp
#include <omp.h>
// ankerl
#include <ankerl/unordered_dense.h>
// liblzf
#include <liblzf/lzf.h>
// ros2
#include <rclcpp/rclcpp.hpp>

#endif// PCH_H
