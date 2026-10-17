#pragma once

#include "eskf.h"
#include "common/common.h"

namespace ly_lio {

// 安装外参 T_BI 将 IMU 坐标变到 baselink；滤波状态始终对应 IMU 原点。
// 返回 baselink 位姿、原点速度以及含杆臂/姿态相关项的协方差。
inline common::Odometry body_odometry(
    double timestamp, const state &x,
    const Eigen::Matrix<double, state::DIM, state::DIM> &covariance,
    const Eigen::Matrix3d &R_BI, const Eigen::Vector3d &t_BI) {
    const Eigen::Matrix3d R_IB = R_BI.transpose();
    const Eigen::Vector3d r_IB = -R_IB * t_BI;  // baselink 原点在 IMU 中的位置。
    const Eigen::Vector3d v_I = x.rotation.transpose() * x.velocity;
    common::Odometry result;
    result.timestamp = timestamp;
    result.position = x.position + x.rotation * r_IB;
    result.orientation = Eigen::Quaterniond(x.rotation * R_IB).normalized();
    // 刚体上不同原点的速度不同：v_B^W = v_I^W + R_WI(omega_I × r_IB)。
    result.velocity = x.velocity + x.rotation * x.omg.cross(r_IB);
    result.angular_velocity = R_BI * x.omg;

    // 输出 pose 的旋转误差在世界系表达；滤波姿态误差是 IMU 右扰动。
    Eigen::Matrix<double, 6, state::DIM> J_pose = Eigen::Matrix<double, 6, state::DIM>::Zero();
    J_pose.block<3, 3>(0, state::position_index).setIdentity();
    J_pose.block<3, 3>(0, state::rotation_index) = -x.rotation * hat<double>(r_IB);
    J_pose.block<3, 3>(3, state::rotation_index) = x.rotation;
    result.pose_covariance = J_pose * covariance * J_pose.transpose();

    // ROS twist 在 baselink 中表达，保留速度、姿态、角速度之间的交叉协方差。
    Eigen::Matrix<double, 6, state::DIM> J_twist = Eigen::Matrix<double, 6, state::DIM>::Zero();
    J_twist.block<3, 3>(0, state::rotation_index) = R_BI * hat<double>(v_I);
    J_twist.block<3, 3>(0, state::velocity_index) = R_BI * x.rotation.transpose();
    J_twist.block<3, 3>(0, state::omg_index) = -R_BI * hat<double>(r_IB);
    J_twist.block<3, 3>(3, state::omg_index) = R_BI;
    result.twist_covariance = J_twist * covariance * J_twist.transpose();
    return result;
}

}  // namespace ly_lio
