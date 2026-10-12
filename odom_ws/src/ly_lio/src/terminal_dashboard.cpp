#include "terminal_dashboard.h"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <sstream>
#include <sys/resource.h>
#include <unistd.h>

namespace ly_lio {
namespace {
std::string number(double value, int precision = 2) {
    if (!std::isfinite(value)) { return "n/a"; }
    std::ostringstream text;
    text << std::fixed << std::setprecision(precision) << value;
    return text.str();
}
std::string vector_text(const Eigen::Vector3d &v, int precision = 4) {
    return number(v.x(), precision) + " " + number(v.y(), precision) + " " + number(v.z(), precision);
}
double process_cpu_seconds() {
    rusage usage{};
    if (getrusage(RUSAGE_SELF, &usage) != 0) { return NAN; }
    return usage.ru_utime.tv_sec + usage.ru_stime.tv_sec +
        (usage.ru_utime.tv_usec + usage.ru_stime.tv_usec) * 1e-6;
}
double resident_mb() {
    std::ifstream file("/proc/self/statm");
    unsigned long total = 0, resident = 0;
    const long page_size = sysconf(_SC_PAGESIZE);
    if (!(file >> total >> resident) || page_size <= 0) { return NAN; }
    // 与参考PLIO一致，MB按十进制计；组件模式下统计整个进程。
    return static_cast<double>(resident) * page_size / 1e6;
}
}

double TerminalDashboard::steady_seconds() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
TerminalDashboard::TerminalDashboard(bool clear_screen, double stale_timeout,
    std::string world_frame, std::string body_frame)
    : clear_screen_(clear_screen), stale_timeout_(stale_timeout),
      world_frame_(std::move(world_frame)), body_frame_(std::move(body_frame)),
      previous_print_(steady_seconds()), previous_cpu_(process_cpu_seconds()) {
    const char *term = std::getenv("TERM");
    clear_screen_ = clear_screen_ && isatty(STDOUT_FILENO) && term && std::string(term) != "dumb";
}
void TerminalDashboard::Stream::observe(double stamp) {
    // 重复/倒退stamp既不计频率，也不能让断流状态继续显示GOOD。
    if (!std::isfinite(stamp) || stamp < 0 || stamp <= last_stamp) { return; }
    ++count;
    if (first_stamp < 0) { first_stamp = stamp; }
    last_stamp = stamp;
    last_arrival = steady_seconds();
    stamps.push_back(stamp);
    if (stamps.size() > 101) { stamps.pop_front(); }
}
void TerminalDashboard::lidar_received(double stamp) { working_.lidar.observe(stamp); }
void TerminalDashboard::imu_received(double stamp) { working_.imu.observe(stamp); }
void TerminalDashboard::odometry_published(const common::Odometry &body, bool tf_published,
    const state &imu_state, const Eigen::Matrix3d &R_BI) {
    working_.odom.observe(body.timestamp);
    if (tf_published) { working_.tf.observe(body.timestamp); }
    if (!working_.has_pose) {
        // 标定前无输出，不拿默认零状态作原点；异常首帧也不能永久污染参考变换。
        if (!std::isfinite(body.timestamp) || body.timestamp < 0 || !body.position.allFinite() ||
            !body.orientation.coeffs().allFinite() || body.orientation.norm() < 1e-9) { return; }
        working_.origin = working_.distance_anchor = body.position;
        working_.origin_orientation = body.orientation.normalized();
        working_.first_pose_stamp = body.timestamp;
    }
    // 沿用参考PLIO的10cm累计路程门限；每次发布计算，不漏掉两次终端刷新之间的运动。
    const double step = (body.position - working_.distance_anchor).norm();
    if (std::isfinite(step) && step >= 0.1) {
        working_.distance_traveled += step;
        working_.distance_anchor = body.position;
    }
    working_.has_pose = true;
    working_.position = body.position;
    working_.orientation = body.orientation;
    working_.velocity_world = body.velocity;
    working_.angular_body = body.angular_velocity;
    working_.gravity_world = imu_state.gravity;
    // 偏置是IMU的偏置向量，以baselink轴表达；不等同于机体原点的加速度测量。
    working_.bg_body = R_BI * imu_state.bg;
    working_.ba_body = R_BI * imu_state.ba;
}
void TerminalDashboard::reset_pose() { working_ = Snapshot{}; }
void TerminalDashboard::update(const LioDiagnostics &diagnostics) {
    working_.core = diagnostics;
    working_.processed.observe(diagnostics.last_scan_end);
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot_ = working_;
}
double TerminalDashboard::sensor_rate(const Stream &stream) {
    if (stream.stamps.size() < 2) { return 0; }
    const double span = stream.stamps.back() - stream.stamps.front();
    return span > 0 ? static_cast<double>(stream.stamps.size() - 1) / span : 0;
}
void TerminalDashboard::print() {
    Snapshot s;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        s = snapshot_;
    }
    const double now = steady_seconds(), dt = now - previous_print_;
    const auto fresh = [this, now](const Stream &stream) {
        return stream.last_arrival >= 0 && now - stream.last_arrival <= stale_timeout_;
    };
    const auto hz = [&fresh](const Stream &stream) {
        return fresh(stream) ? sensor_rate(stream) : 0.0;
    };
    const auto &d = s.core;
    const double match_ratio = d.last_tracking_points ?
        static_cast<double>(d.last_matched_points) / d.last_tracking_points : 0;
    std::string health, reason;
    if (s.lidar.last_stamp < 0) {
        health = "WAIT_LIDAR"; reason = "waiting for decoded LiDAR data";
    } else if (d.imu_enabled && s.imu.last_stamp < 0) {
        health = "WAIT_IMU"; reason = "waiting for valid IMU data";
    } else if (!fresh(s.lidar) || (d.imu_enabled && !fresh(s.imu))) {
        health = "STALE"; reason = "LiDAR/IMU sensor timestamps stopped advancing";
    } else if (!d.imu_calibrated) {
        health = "CALIBRATING"; reason = "keep stationary for IMU initialization";
    } else if (!d.initialized) {
        health = "BUILDING_MAP"; reason = "keep stationary while collecting initial scans";
    } else if (!s.has_pose || !fresh(s.odom)) {
        health = "WAIT_OUTPUT"; reason = "waiting for a scan covered by IMU measurements";
    } else if (!s.position.allFinite() || !s.orientation.coeffs().allFinite() ||
               !s.velocity_world.allFinite() || !s.angular_body.allFinite() ||
               !s.gravity_world.allFinite() || !s.bg_body.allFinite() || !s.ba_body.allFinite()) {
        health = "CHECK"; reason = "non-finite state";
    } else if (match_ratio < 0.01) {
        health = "CHECK"; reason = "low plane matches";
    } else {
        health = d.scans_processed > 5 ? "GOOD" : "WARMUP";
    }
    const double cpu = process_cpu_seconds();
    const double cores = dt > 0 ? (cpu - previous_cpu_) / dt : 0;
    const long processor_count = std::max(1L, sysconf(_SC_NPROCESSORS_ONLN));
    // 参考PLIO的CPU Load以全机逻辑核总容量为100%，Cores Utilized给等效核数。
    const double cpu_load = 100 * cores / processor_count;
    cpu_loads_.push_back(cpu_load);
    if (cpu_loads_.size() > 100) { cpu_loads_.pop_front(); }
    const double average_cpu = std::accumulate(cpu_loads_.begin(), cpu_loads_.end(), 0.0) / cpu_loads_.size();
    std::ostringstream date;
    if (s.has_pose) {
        const std::time_t stamp = static_cast<std::time_t>(s.odom.last_stamp);
        std::tm local{};
        localtime_r(&stamp, &local);
        date << std::put_time(&local, "%a %b %d %H:%M:%S %Y");
    } else {
        date << "Waiting for " << body_frame_ << " in " << world_frame_;
    }
    const auto state_vector = [&s](const Eigen::Vector3d &v, int precision = 4) {
        return s.has_pose ? vector_text(v, precision) : std::string("n/a");
    };
    const double elapsed = s.has_pose ? s.odom.last_stamp - s.first_pose_stamp : 0;
    const double start_offset_ms = s.has_pose ? (d.last_scan_start - s.odom.last_stamp) * 1e3 : NAN;
    const double end_offset_ms = s.has_pose ? (d.last_scan_end - s.odom.last_stamp) * 1e3 : NAN;
    const double imu_lead_ms = d.imu_enabled && d.last_scan_end >= 0 ?
        (d.latest_imu_time - d.last_scan_end) * 1e3 : NAN;
    std::ostringstream text;
    // 参考PLIO的边框、字段顺序、空行和精度。完整块一次写出，IO在独立回调组。
    if (clear_screen_) { text << "\033[2J\033[1;1H"; }
    const std::string border = "+-------------------------------------------------------------------+\n";
    const auto line = [&text](const std::string &value) {
        text << "| " << std::left << std::setw(66) << value.substr(0, 66) << "|\n";
    };
    text << '\n' << border
         << "|                    Point-LIO Odometry                            |\n" << border;
    line(date.str() + "   Elapsed: " + number(elapsed, 2) + " seconds");
    line("Sensor Rates: LiDAR recv @ " + number(hz(s.lidar), 2) +
         " Hz, IMU @ " + (d.imu_enabled ? number(hz(s.imu), 2) : "disabled") + " Hz");
    const double odom_hz = hz(s.odom), tf_hz = hz(s.tf);
    const std::string output_rates = std::abs(odom_hz - tf_hz) < 0.05 ? number(odom_hz, 1) :
        number(odom_hz, 1) + "/" + number(tf_hz, 1);
    line("Processing Rate     :: " + number(hz(s.processed), 2) +
         " Hz; odom/TF @ " + output_rates + " Hz");
    line("Clouds recv/accept/proc :: " + std::to_string(d.scans_received) + "/" +
         std::to_string(d.scans_accepted) + "/" + std::to_string(d.scans_processed));
    line("Cloud reject/pending    :: " + std::to_string(d.scans_rejected) + "/" + std::to_string(d.scans_dropped));
    // 目前核心没有DDS message_lost事件，不能用队列丢帧冒充DDS统计或伪造0。
    line("DDS message drops       :: n/a");
    line("Input / tracked    :: " + std::to_string(d.last_input_points) + " / " + std::to_string(d.last_tracking_points));
    text << "|===================================================================|\n";
    line("Health             :: " + health);
    for (size_t offset = 0; offset < reason.size(); offset += 56) {
        line("Reason :: " + reason.substr(offset, 56));
    }
    // 保留PLIO标签：这里{W}专指启动时的固定机体系B0，{B}仍是当前baselink。
    // T_B0B = inverse(T_WB0) * T_WB，同时去掉初始平移和朝向；只改显示，不写回滤波/TF。
    const Eigen::Quaterniond q_B0W = s.origin_orientation.conjugate();
    const Eigen::Vector3d position_initial = q_B0W * (s.position - s.origin);
    const Eigen::Quaterniond orientation_initial = (q_B0W * s.orientation).normalized();
    line("Position     {W}   :: " + state_vector(position_initial));
    line("Orientation {W} wxyz :: " + (s.has_pose ? number(orientation_initial.w(), 4) + " " +
         number(orientation_initial.x(), 4) + " " + number(orientation_initial.y(), 4) + " " + number(orientation_initial.z(), 4) : "n/a"));
    line("Lin Velocity {W}   :: " + state_vector(q_B0W * s.velocity_world));
    // 当前B系速度使用原始q_WB和v_W，避免将相对姿态与原始世界速度混用。
    line("Lin Velocity {B}   :: " + state_vector(s.orientation.conjugate() * s.velocity_world));
    line("Ang Velocity {B}   :: " + state_vector(s.angular_body));
    line("Gravity      {W}   :: " + state_vector(q_B0W * s.gravity_world, 5));
    line("Accel Bias         :: " + state_vector(s.ba_body, 8));
    line("Gyro Bias          :: " + state_vector(s.bg_body, 8));
    text << "|                                                                   |\n";
    line("Distance Traveled  :: " + (s.has_pose ? number(s.distance_traveled, 4) : "n/a") + " meters");
    line("Distance to Origin :: " + (s.has_pose ? number(position_initial.norm(), 4) : "n/a") + " meters");
    line("Registration       :: matches " + std::to_string(d.last_matched_points) + "/" +
         std::to_string(d.last_tracking_points) + " (" + number(match_ratio * 100, 1) +
         "%), map voxels " + std::to_string(d.map_voxels));
    line("Point Time [ms]    :: " + number(start_offset_ms, 3) + " .. " + number(end_offset_ms, 3) +
         ", IMU lead " + number(imu_lead_ms, 3));
    text << "|                                                                   |\n";
    line("Computation Time   :: " + number(d.last_processing_ms, 2) + " ms // Avg: " +
         number(d.mean_processing_ms, 2) + " / Max: " + number(d.max_processing_ms, 2));
    line("Cores Utilized     :: " + number(cores, 2) + " / " + std::to_string(processor_count) + " cores");
    line("CPU Load           :: " + number(cpu_load, 2) + "% // Avg: " + number(average_cpu, 2) + "%");
    line("RAM Allocation     :: " + number(resident_mb(), 2) + " MB");
    text << border;
    std::cout << text.str() << std::flush;
    previous_print_ = now;
    previous_cpu_ = cpu;
}
}  // namespace ly_lio
