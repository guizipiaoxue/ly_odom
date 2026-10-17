// 原生 LY-LIO 诊断采集器。只向 /tmp 中的源码副本插入只读探针，不改生产源码。
// 由 vision_odom.py 构建及调用；全部点仍使用原生预处理、地图、H、P 和更新。
// 每点 contribution 是冻结同一 Batch 先验下 K_j*r_j 的线性分解，不是因果概率。
#ifdef VISION_ODOM_PREPARE
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
namespace fs = std::filesystem;
static std::string read(const fs::path &p) { std::ifstream f(p); if (!f) throw std::runtime_error("cannot read "+p.string()); return {std::istreambuf_iterator<char>(f),{}}; }
static void write(const fs::path &p,const std::string &s) { std::ofstream f(p); f<<s; if(!f) throw std::runtime_error("cannot write "+p.string()); }
static void replace(std::string &s,const std::string &a,const std::string &b,bool all=false) { size_t pos=0,n=0; while((pos=s.find(a,pos))!=std::string::npos){s.replace(pos,a.size(),b);pos+=b.size();++n;if(!all)break;} if(!n)throw std::runtime_error("source contract changed: "+a); }
static const char *trace_header = R"TRACE(
#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fstream>
#include <iomanip>
#include <vector>
#include <array>
#include <cmath>
#include <cstdint>
namespace vision_trace {
// points.bin: uint32 scan,batch,status + float32[42]，小端、无文件头、每点180字节。
// float列: dt,lidar[3],world[3],normal[3],residual,R,contribution_body[3],
// contribution_imu_position[3],contribution_velocity[3],eigenvalues[3],centroid[3],support[5,3].
struct PointRecord { uint32_t scan,batch,status; float f[42]{}; };
static_assert(sizeof(PointRecord)==180);
struct Pending { PointRecord disk{}; Eigen::Matrix<double,1,18> H=Eigen::Matrix<double,1,18>::Zero(); double z=0,R=1; bool accepted=false; };
inline bool enabled=false;
inline std::ofstream point_file, scan_file, batch_file, imu_file, snapshot_file;
inline uint32_t scan=0,batch=0;
inline double batch_time=0;
inline Eigen::Matrix3d R_BL=Eigen::Matrix3d::Identity();
inline Eigen::Vector3d t_BL=Eigen::Vector3d::Zero();
inline std::vector<Pending> pending;
inline Pending *active=nullptr;
inline Eigen::Matrix<double,3,30> Jbody=Eigen::Matrix<double,3,30>::Zero();
inline Eigen::Vector3d sums[3], lengths=Eigen::Vector3d::Zero();
inline double scan_start=0,scan_end=0;
inline size_t scan_candidates=0,scan_accepted=0,point_count=0,update_failures=0,nonfinite=0;
inline double max_contribution_error=0;
inline Eigen::Vector3d gain_sum=Eigen::Vector3d::Zero();
template<class S> Eigen::Vector3d body(const S &s) { return s.position+s.rotation*(s.offset_T_L_I-s.offset_R_L_I*R_BL.transpose()*t_BL); }
template<class S> Eigen::Vector3d body_velocity(const S &s) { return s.velocity+s.rotation*s.omg.cross(Eigen::Vector3d(s.offset_T_L_I-s.offset_R_L_I*R_BL.transpose()*t_BL)); }
inline Eigen::Matrix3d skew(const Eigen::Vector3d &v) { Eigen::Matrix3d m; m<<0,-v.z(),v.y(),v.z(),0,-v.x(),-v.y(),v.x(),0;return m; }
inline void init(const std::string &dir,bool trace,const Eigen::Matrix3d &r,const Eigen::Vector3d &t) {
 enabled=trace;R_BL=r;t_BL=t;
 snapshot_file.open(dir+"/state_covariance.bin",std::ios::binary);
 scan_file.open(dir+"/scans.csv");scan_file<<std::setprecision(17)<<"scan,t,x,y,z,qx,qy,qz,qw,vx,vy,vz,bgx,bgy,bgz,bax,bay,baz,gx,gy,gz,p_trace,matched,tracking,map_voxels,predict_dx,predict_dy,predict_dz,imu_dx,imu_dy,imu_dz,laser_dx,laser_dy,laser_dz,predict_path,imu_path,laser_path\n";
 if(enabled){ point_file.open(dir+"/points.bin",std::ios::binary);batch_file.open(dir+"/batches.csv");batch_file<<std::setprecision(17)<<"scan,batch,t,candidates,accepted,success,dx,dy,dz,sum_cx,sum_cy,sum_cz,linearization_error,information_min,information_mid,information_max\n";imu_file.open(dir+"/imu_updates.csv");imu_file<<std::setprecision(17)<<"scan,t,dx,dy,dz,dvx,dvy,dvz,bgx,bgy,bgz,bax,bay,baz\n"; }
 if(!snapshot_file||!scan_file||(enabled&&(!point_file||!batch_file||!imu_file)))throw std::runtime_error("cannot create trace files");
}
inline void begin_scan(double start,double end){++scan;scan_start=start;scan_end=end;scan_candidates=scan_accepted=0;for(auto &s:sums)s.setZero();lengths.setZero();}
template<class S> void delta(int kind,double t,const S &before,const S &after) {
 if(!enabled)return; const Eigen::Vector3d d=body(after)-body(before);sums[kind]+=d;lengths[kind]+=d.norm();
 if(kind==1){const Eigen::Vector3d dv=after.velocity-before.velocity;imu_file<<scan<<','<<t;for(double x:d)imu_file<<','<<x;for(double x:dv)imu_file<<','<<x;for(double x:after.bg)imu_file<<','<<x;for(double x:after.ba)imu_file<<','<<x;imu_file<<'\n';}
}
template<class S> void begin_batch(double t,const S &s) {
 if(!enabled)return; ++batch;batch_time=t;pending.clear();active=nullptr;gain_sum.setZero();
 Jbody.setZero();Jbody.template block<3,3>(0,0).setIdentity();const Eigen::Vector3d u=R_BL.transpose()*t_BL;
 const Eigen::Vector3d r=s.offset_T_L_I-s.offset_R_L_I*u;
 Jbody.template block<3,3>(0,3)=-s.rotation*skew(r);
 Jbody.template block<3,3>(0,6)=s.rotation*s.offset_R_L_I*skew(u);
 Jbody.template block<3,3>(0,9)=s.rotation;
}
template<class S> void begin_point(const S &s,const Eigen::Vector3d &lidar,double dt) {
 if(!enabled)return;pending.emplace_back();active=&pending.back();active->disk.scan=scan;active->disk.batch=batch;active->disk.status=1;active->disk.f[0]=dt;
 // world坐标随后由实际h_point计算结果写入；不重复运动补偿。
 for(int i=0;i<3;++i){active->disk.f[1+i]=lidar[i];}
}
inline void world(const Eigen::Vector3d &p){if(enabled&&active)for(int i=0;i<3;++i)active->disk.f[4+i]=p[i];}
inline void reject(uint32_t code){if(enabled&&active)active->disk.status=code;}
template<class V> void supports(const V &near){if(!enabled||!active)return;Eigen::Vector3d c=Eigen::Vector3d::Zero();for(size_t j=0;j<near.size()&&j<5;++j){c+=near[j].template cast<double>();for(int i=0;i<3;++i)active->disk.f[27+j*3+i]=near[j][i];}if(!near.empty())c/=double(near.size());for(int i=0;i<3;++i)active->disk.f[24+i]=c[i];}
template<class V> void eigenvalues(const V &v){if(enabled&&active)for(int i=0;i<3;++i)active->disk.f[21+i]=v[i];}
template<class V> void plane(const V &n,double distance){if(enabled&&active){for(int i=0;i<3;++i)active->disk.f[7+i]=n[i];active->disk.f[10]=-distance;}}
template<class M> void point_result(const M &m){if(!enabled||!active)return;active->accepted=m.valid&&m.H.allFinite()&&std::isfinite(m.z);if(active->accepted){active->H=m.H;active->z=m.z;active->R=m.variance;active->disk.status=0;active->disk.f[10]=m.z;active->disk.f[11]=m.variance;}}
template<class C,class T,class D> void gain(const C &cross,const T &solved_lt,const D &delta){
 if(!enabled)return;const auto transform=(cross*solved_lt).eval();constexpr int N=T::RowsAtCompileTime;
 Eigen::Matrix<double,30,1> sum=Eigen::Matrix<double,30,1>::Zero();
 for(auto &p:pending){if(!p.accepted)continue;Eigen::Matrix<double,N,1> h;for(int i=0;i<N;++i){int k=N==12?(i<6?i:i+6):i;h[i]=p.H[k];}const Eigen::Matrix<double,30,1> c=transform*h*(p.z/p.R);const Eigen::Vector3d cb=Jbody*c;for(int i=0;i<3;++i){p.disk.f[12+i]=cb[i];p.disk.f[15+i]=c[i];p.disk.f[18+i]=c[12+i];}sum+=c;gain_sum+=cb;}
 const double error=(sum-delta).norm();if(std::isfinite(error))max_contribution_error=std::max(max_contribution_error,error);else ++nonfinite;
}
template<class S,class B> void end_batch(const S &before,const S &after,const B &measurement,bool success){
 if(!enabled)return;const Eigen::Vector3d d=body(after)-body(before);Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(measurement.information.template block<3,3>(0,0));
 batch_file<<scan<<','<<batch<<','<<batch_time<<','<<pending.size()<<','<<measurement.matched_points<<','<<success;for(double x:d)batch_file<<','<<x;for(double x:gain_sum)batch_file<<','<<x;batch_file<<','<<(d-gain_sum).norm();for(double x:solver.eigenvalues())batch_file<<','<<x;batch_file<<'\n';
 for(auto &p:pending){if(p.accepted&&!success){p.disk.status=8;for(int i=12;i<21;++i)p.disk.f[i]=0;}point_file.write(reinterpret_cast<const char*>(&p.disk),sizeof(PointRecord));}
 scan_candidates+=pending.size();scan_accepted+=success?measurement.matched_points:0;point_count+=pending.size();if(measurement.matched_points&&!success)++update_failures;active=nullptr;
}
template<class S,class P,class D> void snapshot(double t,const S &s,const P &cov,const D &d){
 const Eigen::Vector3d p=body(s),v=body_velocity(s);const Eigen::Quaterniond q(s.rotation*s.offset_R_L_I*R_BL.transpose());
 scan_file<<scan<<','<<t;for(double x:p)scan_file<<','<<x;scan_file<<','<<q.x()<<','<<q.y()<<','<<q.z()<<','<<q.w();for(double x:v)scan_file<<','<<x;for(double x:s.bg)scan_file<<','<<x;for(double x:s.ba)scan_file<<','<<x;for(double x:s.gravity)scan_file<<','<<x;scan_file<<','<<cov.trace()<<','<<d.last_matched_points<<','<<d.last_tracking_points<<','<<d.map_voxels;for(auto &part:sums)for(double x:part)scan_file<<','<<x;for(double x:lengths)scan_file<<','<<x;scan_file<<'\n';
 snapshot_file.write(reinterpret_cast<const char*>(&t),8);
 for(const auto *vec:{&s.position,&s.offset_T_L_I,&s.velocity,&s.omg,&s.acceleration,&s.gravity,&s.bg,&s.ba})snapshot_file.write(reinterpret_cast<const char*>(vec->data()),24);
 snapshot_file.write(reinterpret_cast<const char*>(s.rotation.data()),72);snapshot_file.write(reinterpret_cast<const char*>(s.offset_R_L_I.data()),72);snapshot_file.write(reinterpret_cast<const char*>(cov.data()),30*30*8);
 if(!p.allFinite()||!s.rotation.allFinite()||!v.allFinite()||!cov.allFinite())++nonfinite;
}
inline void finish(const std::string &dir){std::ofstream f(dir+"/trace_summary.json");f<<std::setprecision(17)<<"{\"scans\":"<<scan<<",\"batches\":"<<batch<<",\"candidate_points\":"<<point_count<<",\"update_failures\":"<<update_failures<<",\"nonfinite\":"<<nonfinite<<",\"max_sum_delta_error\":"<<max_contribution_error<<"}\n";}
}
)TRACE";
int main(int argc,char **argv){try{
 if(argc!=3)throw std::runtime_error("prepare SOURCE_PACKAGE TEMP_PACKAGE");const fs::path src=argv[1],dst=argv[2];if(fs::exists(dst))throw std::runtime_error("destination exists");fs::create_directories(dst);for(const auto &name:{"src","include","3rdparty"})fs::copy(src/name,dst/name,fs::copy_options::recursive);write(dst/"include/vision_trace.hpp",trace_header);
 auto e=read(dst/"src/ly_lio/estimator.cpp");replace(e,"#include \"estimator.h\"","#include \"estimator.h\"\n#include <vision_trace.hpp>");
 replace(e,"!s.omg.allFinite() || !s.offset_R_L_I.allFinite() || !s.offset_T_L_I.allFinite()) { return; }","!s.omg.allFinite() || !s.offset_R_L_I.allFinite() || !s.offset_T_L_I.allFinite()) { vision_trace::reject(1); return; }");
 replace(e,"const Eigen::Vector3d point_world = point.point_world(s);","const Eigen::Vector3d point_world = point.point_world(s);\n        vision_trace::world(point_world);");
 replace(e,"if (!point_imu.allFinite() || !point_world.allFinite()) { return; }","if (!point_imu.allFinite() || !point_world.allFinite()) { vision_trace::reject(1); return; }");
 replace(e,"if (nearest_points.size() != 5) { return; }","vision_trace::supports(nearest_points);\n        if (nearest_points.size() != 5) { vision_trace::reject(2); return; }");
 replace(e,"if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite()) { return; }","if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite()) { vision_trace::reject(3); return; }");
 replace(e,"const auto eigenvalues = solver.eigenvalues();","const auto eigenvalues = solver.eigenvalues();\n        vision_trace::eigenvalues(eigenvalues);");
 replace(e,"eigenvalues[0] > 0.1 * eigenvalues[1]) { return; }","eigenvalues[0] > 0.1 * eigenvalues[1]) { vision_trace::reject(4); return; }");
 replace(e,"if (std::abs(normal.dot(point.cast<double>()) + offset) > parameters->plane_threshold) { return; }","if (std::abs(normal.dot(point.cast<double>()) + offset) > parameters->plane_threshold) { vision_trace::reject(5); return; }");
 replace(e,"const double distance = normal.dot(point_world) + offset;","const double distance = normal.dot(point_world) + offset;\n        vision_trace::plane(normal, distance);");
 replace(e,"if (!std::isfinite(distance) || point_imu.norm() <= parameters->match_sqaured * distance * distance) { return; }","if (!std::isfinite(distance) || point_imu.norm() <= parameters->match_sqaured * distance * distance) { vision_trace::reject(6); return; }");
 replace(e,"result.valid = std::isfinite(result.variance) && result.variance > 0;","result.valid = std::isfinite(result.variance) && result.variance > 0;\n        if (!result.valid) { vision_trace::reject(7); }");
 replace(e,"h_point(prior, point, measurement);","vision_trace::begin_point(prior, point.point_lidar, point.time_offset);\n            h_point(prior, point, measurement);\n            vision_trace::point_result(measurement);");
 replace(e,"return kf.update_lidar_batch(batch) ? batch.matched_points : 0;","const bool success=kf.update_lidar_batch(batch);\n        vision_trace::end_batch(prior,kf.x,batch,success);\n        return success ? batch.matched_points : 0;");write(dst/"src/ly_lio/estimator.cpp",e);
 auto k=read(dst/"src/ly_lio/eskf.h");replace(k,"#include <pch.h>","#include <pch.h>\n#include <vision_trace.hpp>");replace(k,"return inject_error(delta, posterior);","if (vision_trace::enabled) vision_trace::gain(cross_whitened, factor.solve(L.transpose()).eval(), delta);\n            return inject_error(delta, posterior);");write(dst/"src/ly_lio/eskf.h",k);
 auto l=read(dst/"src/ly_lio/ly_lio.cpp");replace(l,"const auto processing_start = std::chrono::steady_clock::now();","const auto processing_start = std::chrono::steady_clock::now();\n    vision_trace::begin_scan(scan.timestamp_start,scan.timestamp_end);");
 for(const std::string t:{"imu.timestamp","batch_end","scan.timestamp_end"})replace(l,"estimator.kf.predict_state("+t+");","{ const auto trace_before=estimator.kf.x; estimator.kf.predict_state("+t+"); vision_trace::delta(0,"+t+",trace_before,estimator.kf.x); }");
 replace(l,"estimator.kf.update_imu();","{ const auto trace_before=estimator.kf.x; estimator.kf.update_imu(); vision_trace::delta(1,imu.timestamp,trace_before,estimator.kf.x); }");
 replace(l,"matched_points += estimator.update_lidar_batch(batch_points);","{ const auto trace_before=estimator.kf.x; vision_trace::begin_batch(batch_end,trace_before); matched_points += estimator.update_lidar_batch(batch_points); vision_trace::delta(2,batch_end,trace_before,estimator.kf.x); }");
 // snapshot回调在生产diagnostics刷新前；debug在process_scan尾部单独保存精确已更新计数。
 replace(l,"diagnostic_state.last_scan_end = scan.timestamp_end;","diagnostic_state.last_scan_end = scan.timestamp_end;\n        vision_trace::snapshot(scan.timestamp_end,estimator.kf.x,estimator.kf.P,diagnostics());");write(dst/"src/ly_lio/ly_lio.cpp",l);
 const std::string cmake=R"CMAKE(cmake_minimum_required(VERSION 3.16)
project(vision_odom_native)
set(CMAKE_CXX_STANDARD 20)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(yaml-cpp REQUIRED)
add_executable(vision_odom_native vision_odom_trace.cpp src/ly_lio/parameters.cpp src/ly_lio/preprocess.cpp src/ly_lio/estimator.cpp src/ly_lio/ly_lio.cpp src/util/voxelgrid_sampling.cpp)
target_include_directories(vision_odom_native PRIVATE src include 3rdparty)
target_link_libraries(vision_odom_native Eigen3::Eigen OpenMP::OpenMP_CXX yaml-cpp)
ament_target_dependencies(vision_odom_native rclcpp)
target_compile_options(vision_odom_native PRIVATE -O3 -DNDEBUG)
)CMAKE";
 write(dst/"CMakeLists.txt",cmake);fs::copy_file(fs::path(__FILE__),dst/"vision_odom_trace.cpp");std::cout<<dst<<'\n';return 0;
 }catch(const std::exception &e){std::cerr<<e.what()<<'\n';return 1;}}
#else
#include "ly_lio/ly_lio.h"
#include <vision_trace.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <chrono>
#include <iostream>
#include <string>
// VODOMEV1: 8字节magic。随后I+7xf64(t,a3,w3)，或C+u32 n+n*(f64 t,f32 xyz)。
static bool read_exact(std::ifstream &f,void *p,size_t n){f.read(static_cast<char*>(p),n);if(size_t(f.gcount())!=n)throw std::runtime_error("truncated events.bin");return true;}
int main(int argc,char **argv){try{
 if(argc<6){std::cerr<<"vision_odom_native EVENTS_BIN OUTPUT_DIR PARAM_YAML MOUNT_YAML TRACE_0_OR_1 [MAX_EVENTS] [CONSTANT_IMU_0_OR_1]\n";return 2;}
 const std::string events=argv[1],out=argv[2],config=argv[3],mount=argv[4];const bool trace=std::stoi(argv[5])!=0;const size_t max_events=argc>6?std::stoull(argv[6]):size_t(-1);const bool constant_imu=argc>7&&std::stoi(argv[7])!=0;
 std::filesystem::create_directories(out);
 int rosargc=1; rclcpp::init(rosargc,argv);rclcpp::NodeOptions options;
 const auto params=YAML::LoadFile(config)["ly_lio"]["ros__parameters"];
 for(const auto &p:params){const std::string key=p.first.as<std::string>();const auto value=p.second;if(value.IsSequence()){std::vector<double> v;for(auto x:value)v.push_back(x.as<double>());options.append_parameter_override(key,v);}else if(value.IsScalar()){const std::string text=value.Scalar();if(text=="true"||text=="false")options.append_parameter_override(key,value.as<bool>());else{char *end=nullptr;const double d=std::strtod(text.c_str(),&end);if(end&&*end=='\0'&&!text.empty()){if(text.find_first_of(".eE")==std::string::npos)options.append_parameter_override(key,value.as<int64_t>());else options.append_parameter_override(key,d);}else options.append_parameter_override(key,text);}}}
 rclcpp::Node node("ly_lio",options);ly_lio::LyLio lio(node);
 const auto m=YAML::LoadFile(mount);Eigen::Matrix3d R=Eigen::Matrix3d::Identity();Eigen::Vector3d t=Eigen::Vector3d::Zero();
 // 安装文件支持顶层参数和pcl_publisher.ros__parameters；序列R为行优先。
 auto mp=m;if(m["pcl_publisher"])mp=m["pcl_publisher"]["ros__parameters"];if(m["ros__parameters"])mp=m["ros__parameters"];
 std::vector<double> rv;for(auto a:mp["lidar_to_gimbal_rotation"]){if(a.IsSequence())for(auto b:a)rv.push_back(b.as<double>());else rv.push_back(a.as<double>());}auto tv=mp["lidar_to_gimbal_translation"].as<std::vector<double>>();if(rv.size()!=9||tv.size()!=3)throw std::runtime_error("invalid mount");R=Eigen::Map<Eigen::Matrix<double,3,3,Eigen::RowMajor>>(rv.data());t=Eigen::Map<Eigen::Vector3d>(tv.data());
 vision_trace::init(out,trace,R,t);
 std::ifstream f(events,std::ios::binary);char magic[8];read_exact(f,magic,8);if(std::string(magic,8)!="VODOMEV1")throw std::runtime_error("bad input magic");
 bool constant_ready=false;Eigen::Vector3d constant_acc,constant_gyro;double accel_scale=9.81;if(params["gravity"]){double g2=0;for(auto a:params["gravity"])g2+=std::pow(a.as<double>(),2);accel_scale=std::sqrt(g2);}accel_scale/=params["acc_norm"]?params["acc_norm"].as<double>():1.;
 size_t count=0,ni=0,nc=0,np=0;const auto start=std::chrono::steady_clock::now();char kind;
 while(count<max_events&&f.read(&kind,1)){++count;if(kind=='I'){double data[7];read_exact(f,data,sizeof(data));ly_lio::common::ImuMsg m;m.timestamp=data[0];m.linear_acceleration=Eigen::Map<Eigen::Vector3d>(data+1);m.angular_velocity=Eigen::Map<Eigen::Vector3d>(data+4);if(constant_imu&&constant_ready){m.linear_acceleration=constant_acc;m.angular_velocity=constant_gyro;}lio.on_imu_callback(m);++ni;}else if(kind=='C'){uint32_t n;read_exact(f,&n,4);if(n>10000000)throw std::runtime_error("invalid cloud count");std::vector<ly_lio::common::Point> points;points.reserve(n);for(uint32_t j=0;j<n;++j){double stamp;float xyz[3];read_exact(f,&stamp,8);read_exact(f,xyz,12);points.push_back({stamp,Eigen::Map<Eigen::Vector3f>(xyz)});}np+=n;lio.on_point_cloud_callback(points);++nc;}else throw std::runtime_error("unknown event type");lio.handle_once();if(constant_imu&&!constant_ready&&lio.initialized()){const auto &s=lio.current_state();constant_acc=(s.acceleration+s.ba)/accel_scale;constant_gyro=s.omg+s.bg;constant_ready=true;std::ofstream ci(out+"/constant_imu.json");ci<<std::setprecision(17)<<"{\"acc_raw\":["<<constant_acc.x()<<','<<constant_acc.y()<<','<<constant_acc.z()<<"],\"gyro_rad_s\":["<<constant_gyro.x()<<','<<constant_gyro.y()<<','<<constant_gyro.z()<<"]}\n";}if(count%5000==0)std::cerr<<"events="<<count<<" scans="<<lio.diagnostics().scans_processed<<'\n';}
 const auto d=lio.diagnostics();vision_trace::finish(out);std::ofstream result(out+"/run_summary.json");result<<"{\"input_events\":"<<count<<",\"input_imu\":"<<ni<<",\"input_clouds\":"<<nc<<",\"input_adapter_points\":"<<np<<",\"scans_processed\":"<<d.scans_processed<<",\"scans_rejected\":"<<d.scans_rejected<<",\"scans_dropped\":"<<d.scans_dropped<<",\"pending_scans\":"<<d.pending_scans<<",\"resets\":"<<d.resets<<",\"wall_seconds\":"<<std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count()<<"}\n";rclcpp::shutdown();return 0;
 }catch(const std::exception &e){std::cerr<<e.what()<<'\n';return 1;}}
#endif
