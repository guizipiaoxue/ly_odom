/**
 * 衍生自 Small Point-LIO，移植至 ly_lio 并补充中文算法说明。
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See LICENSE.txt in the project root for license information.
 */

#pragma once

#include "so3_math.h"
#include <pch.h>

namespace ly_lio {

    struct state {
        using value_type = double;

        // Point-LIO 的 IMU 作为输出观测模型：p,R,R_LI,t_LI,v,omega,a,g,bg,ba。
        constexpr static int DIM = 30;
        constexpr static int position_index = 0;
        constexpr static int rotation_index = 3;
        constexpr static int offset_R_L_I_index = 6;
        constexpr static int offset_T_L_I_index = 9;
        constexpr static int velocity_index = 12;
        constexpr static int omg_index = 15;
        constexpr static int acceleration_index = 18;
        constexpr static int gravity_index = 21;
        constexpr static int bg_index = 24;
        constexpr static int ba_index = 27;

        Eigen::Matrix<value_type, 3, 1> position = Eigen::Matrix<value_type, 3, 1>::Zero();        // 位置
        Eigen::Matrix<value_type, 3, 3> rotation = Eigen::Matrix<value_type, 3, 3>::Identity();    // 旋转
        Eigen::Matrix<value_type, 3, 3> offset_R_L_I = Eigen::Matrix<value_type, 3, 3>::Identity();// 雷达到 IMU 的 R
        Eigen::Matrix<value_type, 3, 1> offset_T_L_I = Eigen::Matrix<value_type, 3, 1>::Zero();    // 雷达到 IMU 的 T
        Eigen::Matrix<value_type, 3, 1> velocity = Eigen::Matrix<value_type, 3, 1>::Zero();        // 速度
        Eigen::Matrix<value_type, 3, 1> omg = Eigen::Matrix<value_type, 3, 1>::Zero();             // 角速度
        Eigen::Matrix<value_type, 3, 1> acceleration = Eigen::Matrix<value_type, 3, 1>::Zero();    // 加速度
        Eigen::Matrix<value_type, 3, 1> gravity = Eigen::Matrix<value_type, 3, 1>::Zero();         // 重力
        Eigen::Matrix<value_type, 3, 1> bg = Eigen::Matrix<value_type, 3, 1>::Zero();              // 陀螺仪零偏
        Eigen::Matrix<value_type, 3, 1> ba = Eigen::Matrix<value_type, 3, 1>::Zero();              // 加速度零偏

        state() = default;

        inline void plus(const Eigen::Matrix<value_type, DIM, 1> &vec) {
            position += vec.segment<3>(position_index);
            rotation *= exp<value_type>(vec.segment<3>(rotation_index));
            offset_R_L_I *= exp<value_type>(vec.segment<3>(offset_R_L_I_index));
            offset_T_L_I += vec.segment<3>(offset_T_L_I_index);
            velocity += vec.segment<3>(velocity_index);
            omg += vec.segment<3>(omg_index);
            acceleration += vec.segment<3>(acceleration_index);
            gravity += vec.segment<3>(gravity_index);
            bg += vec.segment<3>(bg_index);
            ba += vec.segment<3>(ba_index);
        }
    };

    struct point_measurement_result {
        bool valid = false;
        state::value_type z = 0.0;  // 观测减预测：负的点到地图平面有符号距离。
        Eigen::Matrix<state::value_type, 1, 12> H = Eigen::Matrix<state::value_type, 1, 12>::Zero();
        state::value_type laser_point_cov = 0.01;
    };

    struct imu_measurement_result {
        bool satu_check[6]{};
        Eigen::Matrix<state::value_type, 6, 1> z = Eigen::Matrix<state::value_type, 6, 1>::Zero();
        state::value_type imu_meas_omg_cov = 0.01;
        state::value_type imu_meas_acc_cov = 0.01;
    };

    class eskf {
    public:
        using measurement_model_point = std::function<void(const state &, point_measurement_result &)>;
        using measurement_model_imu = std::function<void(const state &, imu_measurement_result &)>;
        state x;
        Eigen::Matrix<double, state::DIM, state::DIM> P = Eigen::Matrix<double, state::DIM, state::DIM>::Identity();

    private:
        double time_predict_state_last = 0.0;
        double time_predict_cov_last = 0.0;
        measurement_model_point h_point;
        measurement_model_imu h_imu;

        inline bool inject_error(
            const Eigen::Matrix<double, state::DIM, 1> &delta,
            const Eigen::Matrix<double, state::DIM, state::DIM> &posterior) {
            if (!delta.allFinite() || !posterior.allFinite()) { return false; }
            // 名义R采用右扰动注入；注入后的误差切空间改变，P两侧须同时reset。
            // 在线旋转外参使用相同规则，固定外参的delta为零。
            const Eigen::Matrix3d J_rotation = A_matrix<double>(-delta.template segment<3>(state::rotation_index));
            const Eigen::Matrix3d J_extrinsic = A_matrix<double>(-delta.template segment<3>(state::offset_R_L_I_index));
            auto left = posterior.eval();
            left.template middleRows<3>(state::rotation_index) = J_rotation * posterior.template middleRows<3>(state::rotation_index);
            left.template middleRows<3>(state::offset_R_L_I_index) = J_extrinsic * posterior.template middleRows<3>(state::offset_R_L_I_index);
            auto reset = left.eval();
            reset.template middleCols<3>(state::rotation_index) = left.template middleCols<3>(state::rotation_index) * J_rotation.transpose();
            reset.template middleCols<3>(state::offset_R_L_I_index) = left.template middleCols<3>(state::offset_R_L_I_index) * J_extrinsic.transpose();
            if (!reset.allFinite()) { return false; }
            x.plus(delta);
            P = (0.5 * (reset + reset.transpose())).eval();
            return true;
        }

    public:
        eskf() = default;
        inline void init(const measurement_model_point &point_model, const measurement_model_imu &imu_model) {
            h_point = point_model;
            h_imu = imu_model;
        }
        inline void init_timestamp(double timestamp) {
            if (!std::isfinite(timestamp)) { throw std::invalid_argument("滤波起始时间必须有限"); }
            time_predict_state_last = time_predict_cov_last = timestamp;
        }

        // 状态在每个点时间预测；每次观测和发布前把协方差推进到同一时刻。
        inline void predict_state(double timestamp) {
            auto dt_state = static_cast<state::value_type>(timestamp - time_predict_state_last);
            if (dt_state > 0) [[likely]] {
                time_predict_state_last = timestamp;
                x.position += x.velocity * dt_state;
                // 与 Point-LIO 一阶模型一致，用区间起始姿态转换比力，再推进旋转。
                x.velocity += (x.rotation * x.acceleration + x.gravity) * dt_state;
                x.rotation *= exp<state::value_type>(x.omg * dt_state);
            }
        }

        inline void predict_cov(double timestamp, const Eigen::Matrix<state::value_type, state::DIM, state::DIM> &Q,
                                double noise_reference_dt = 0.005) {
            if (!std::isfinite(noise_reference_dt) || noise_reference_dt <= 0) {
                throw std::invalid_argument("过程噪声参考周期必须为正的有限秒数");
            }
            auto dt_cov = static_cast<state::value_type>(timestamp - time_predict_cov_last);
            if (dt_cov > 0) [[likely]] {
                time_predict_cov_last = timestamp;
                Eigen::Matrix<state::value_type, 3, 1> seg_SO3 = -x.omg * dt_cov;
                const Eigen::Matrix3d E = exp<state::value_type>(seg_SO3);
                const Eigen::Matrix3d A = A_matrix<state::value_type>(seg_SO3) * dt_cov;
                const Eigen::Matrix3d S = -x.rotation * hat<state::value_type>(x.acceleration) * dt_cov;
                const Eigen::Matrix3d B = x.rotation * dt_cov;
                // F 只有位置/姿态/速度的9行非单位块，分两步精确计算 F P Fᵀ。
                // 两个独立临时矩阵保留原始交叉块，不能原地先覆盖姿态再计算速度。
                auto left = P.eval();
                left.middleRows<3>(state::position_index) += dt_cov * P.middleRows<3>(state::velocity_index);
                left.middleRows<3>(state::rotation_index) = E * P.middleRows<3>(state::rotation_index) +
                    A * P.middleRows<3>(state::omg_index);
                left.middleRows<3>(state::velocity_index) += S * P.middleRows<3>(state::rotation_index) +
                    B * P.middleRows<3>(state::acceleration_index) + dt_cov * P.middleRows<3>(state::gravity_index);
                auto propagated = left.eval();
                propagated.middleCols<3>(state::position_index) += dt_cov * left.middleCols<3>(state::velocity_index);
                propagated.middleCols<3>(state::rotation_index) = left.middleCols<3>(state::rotation_index) * E.transpose() +
                    left.middleCols<3>(state::omg_index) * A.transpose();
                propagated.middleCols<3>(state::velocity_index) += left.middleCols<3>(state::rotation_index) * S.transpose() +
                    left.middleCols<3>(state::acceleration_index) * B.transpose() + dt_cov * left.middleCols<3>(state::gravity_index);
                // 旧 Q*dt² 随激光点分段变密而缩小。用参考IMU周期转换为方差率，
                // 默认5ms保留原200Hz尺度；同一时间内的累计随机游走不再取决于点数。
                propagated += Q * (dt_cov * noise_reference_dt);
                P = (0.5 * (propagated + propagated.transpose())).eval();

            }
        }

        // Point-LIO逐点点到面观测：每次只求一个标量创新，完整30维交叉协方差
        // 仍使位置/姿态观测间接修正速度、比力及IMU零偏；不是逐帧ICP。
        inline bool update_point() {
            if (!h_point || !P.allFinite()) { return false; }
            point_measurement_result measurement;
            h_point(x, measurement);
            if (!measurement.valid || !measurement.H.allFinite() || !std::isfinite(measurement.z) ||
                !std::isfinite(measurement.laser_point_cov) || measurement.laser_point_cov <= 0) { return false; }
            const Eigen::Matrix<double, state::DIM, 1> PHT =
                P.template leftCols<12>() * measurement.H.transpose();
            const double innovation = (measurement.H * PHT.template head<12>())(0) + measurement.laser_point_cov;
            if (!std::isfinite(innovation) || innovation <= 0) { return false; }
            const Eigen::Matrix<double, state::DIM, 1> K = PHT / innovation;
            // 最优标量增益下的Joseph式可化为P-PHT*PHTᵀ/S，避免30x30稠密乘法。
            const Eigen::Matrix<double, state::DIM, state::DIM> posterior = P - K * PHT.transpose();
            return inject_error(K * measurement.z, posterior);
        }

        inline bool update_imu() {
            if (!h_imu || !P.allFinite()) { return false; }
            imu_measurement_result measurement_result;
            h_imu(x, measurement_result);
            if (!std::isfinite(measurement_result.imu_meas_omg_cov) || measurement_result.imu_meas_omg_cov <= 0 ||
                !std::isfinite(measurement_result.imu_meas_acc_cov) || measurement_result.imu_meas_acc_cov <= 0) { return false; }
            Eigen::Matrix<state::value_type, 6, 1> z = measurement_result.z;
            Eigen::Matrix<state::value_type, state::DIM, 6> PHT = Eigen::Matrix<state::value_type, state::DIM, 6>::Zero();
            Eigen::Matrix<state::value_type, 6, state::DIM> HP = Eigen::Matrix<state::value_type, 6, state::DIM>::Zero();
            Eigen::Matrix<state::value_type, 6, 6> HPHT = Eigen::Matrix<state::value_type, 6, 6>::Zero();
            for (int i = 0; i < 3; i++) {
                if (!measurement_result.satu_check[i]) {
                    PHT.col(i) = P.col(state::omg_index + i) + P.col(state::bg_index + i);
                    HP.row(i) = P.row(state::omg_index + i) + P.row(state::bg_index + i);
                }
                if (!measurement_result.satu_check[i + 3]) {
                    PHT.col(i + 3) = P.col(state::acceleration_index + i) + P.col(state::ba_index + i);
                    HP.row(i + 3) = P.row(state::acceleration_index + i) + P.row(state::ba_index + i);
                }
            }
            for (int i = 0; i < 3; i++) {
                if (!measurement_result.satu_check[i]) {
                    HPHT.col(i) = HP.col(state::omg_index + i) + HP.col(state::bg_index + i);
                }
                if (!measurement_result.satu_check[i + 3]) {
                    HPHT.col(i + 3) = HP.col(state::acceleration_index + i) + HP.col(state::ba_index + i);
                }
                HPHT(i, i) += measurement_result.imu_meas_omg_cov;
                HPHT(i + 3, i + 3) += measurement_result.imu_meas_acc_cov;
            }
            Eigen::LDLT<Eigen::Matrix<state::value_type, 6, 6>> ldlt(HPHT);
            // IMU 观测只需求解固定 6x6 系统；饱和轴的列为零。
            if (ldlt.info() != Eigen::Success || !ldlt.isPositive() || !z.allFinite()) [[unlikely]] {
                return false;
            }
            Eigen::Matrix<state::value_type, state::DIM, 6> K = PHT * ldlt.solve(Eigen::Matrix<state::value_type, 6, 6>::Identity());
            // 展开的Joseph式保持观测噪声项，并使用独立临时量避免Eigen原地自别名。
            const Eigen::Matrix<double, state::DIM, state::DIM> KHP = K * HP;
            const Eigen::Matrix<double, state::DIM, state::DIM> posterior =
                P - KHP - KHP.transpose() + K * HPHT * K.transpose();
            return inject_error(K * z, posterior);
        }
    };

}  // namespace ly_lio
