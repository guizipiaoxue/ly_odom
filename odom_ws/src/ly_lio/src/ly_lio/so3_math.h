/**
 * 衍生自 Small Point-LIO，移植至 ly_lio 并补充中文算法说明。
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See LICENSE.txt in the project root for license information.
 */


#pragma once
#include <pch.h>

namespace ly_lio {
// SO(3) 右扰动：旋转状态通过 R * Exp(delta_theta) 更新，不能直接加旋转矩阵。
template<class T>
Eigen::Matrix<T, 3, 3> hat(const Eigen::Matrix<T, 3, 1> &v) {
    Eigen::Matrix<T, 3, 3> result;
    result << 0, -v[2], v[1], v[2], 0, -v[0], -v[1], v[0], 0;
    return result;
}
template<class T>
Eigen::Matrix<T, 3, 3> exp(const Eigen::Matrix<T, 3, 1> &angle) {
    const T squared = angle.squaredNorm();
    const auto skew = hat<T>(angle);
    T a, b;
    // 小角度用级数，避免 1-cos(theta) 的相消误差与微小旋转被截断。
    if (squared < T(1e-8)) {
        a = T(1) - squared / T(6) + squared * squared / T(120);
        b = T(0.5) - squared / T(24) + squared * squared / T(720);
    } else {
        const T norm = std::sqrt(squared);
        a = std::sin(norm) / norm;
        b = (T(1) - std::cos(norm)) / squared;
    }
    return Eigen::Matrix<T, 3, 3>::Identity() + a * skew + b * skew * skew;
}
template<class T>
Eigen::Matrix<T, 3, 3> A_matrix(const Eigen::Matrix<T, 3, 1> &angle) {
    const T squared = angle.squaredNorm();
    const auto skew = hat<T>(angle);
    T a, b;
    if (squared < T(1e-8)) {
        a = T(0.5) - squared / T(24);
        b = T(1.0 / 6.0) - squared / T(120);
    } else {
        const T norm = std::sqrt(squared);
        a = (T(1) - std::cos(norm)) / squared;
        b = (T(1) - std::sin(norm) / norm) / squared;
    }
    return Eigen::Matrix<T, 3, 3>::Identity() + a * skew + b * skew * skew;
}
}  // namespace ly_lio
