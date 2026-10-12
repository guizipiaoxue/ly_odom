#pragma once

#include "ly_lio/ly_lio.h"
#include <chrono>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>

namespace ly_lio {
// 统计与滤波时间分离：steady_clock仅用于吞吐/新鲜度/耗时，绝不写入ROS数据stamp。
class TerminalDashboard {
public:
    static double steady_seconds();
    TerminalDashboard(bool clear_screen, double stale_timeout,
                      std::string world_frame, std::string body_frame);
    // 下面四个写入口只由节点的默认互斥回调组调用，不在逐点循环中做终端IO。
    void lidar_received(double sensor_time);
    void imu_received(double sensor_time);
    void odometry_published(const common::Odometry &body, bool tf_published,
                            const state &imu_state, const Eigen::Matrix3d &R_BI);
    void reset_pose();
    void update(const LioDiagnostics &diagnostics);
    // 独立wall timer调用，复制快照后立即释放锁；慢终端不能持有算法需要的锁。
    void print();
private:
    struct Stream {
        uint64_t count = 0;
        double first_stamp = -1, last_stamp = -1, last_arrival = -1;
        std::deque<double> stamps;
        void observe(double stamp);
    };
    struct Snapshot {
        LioDiagnostics core;
        Stream lidar, imu, odom, tf, processed;
        bool has_pose = false;
        double first_pose_stamp = -1;
        Eigen::Vector3d position = Eigen::Vector3d::Zero();
        // 固定显示系B0：首个有效body输出的完整位姿；reset后重新选取。
        Eigen::Vector3d origin = Eigen::Vector3d::Zero();
        Eigen::Quaterniond origin_orientation = Eigen::Quaterniond::Identity();
        Eigen::Vector3d distance_anchor = Eigen::Vector3d::Zero();
        double distance_traveled = 0;
        Eigen::Vector3d velocity_world = Eigen::Vector3d::Zero();
        Eigen::Vector3d angular_body = Eigen::Vector3d::Zero();
        Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
        Eigen::Vector3d gravity_world = Eigen::Vector3d::Zero();
        Eigen::Vector3d bg_body = Eigen::Vector3d::Zero(), ba_body = Eigen::Vector3d::Zero();
    };
    static double sensor_rate(const Stream &stream);
    bool clear_screen_;
    double stale_timeout_;
    std::string world_frame_, body_frame_;
    Snapshot working_, snapshot_;
    std::mutex mutex_;
    double previous_print_, previous_cpu_;
    std::deque<double> cpu_loads_;
};
}  // namespace ly_lio
