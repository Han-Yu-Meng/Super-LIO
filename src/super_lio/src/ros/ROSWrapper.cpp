#include "ros/ROSWrapper.h"
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>

using namespace BASIC;

namespace LI2Sup{

void LoadParamFromFins()
{
  fins::ParamLoader map("super_lio.map");
  fins::ParamLoader sensor("super_lio.sensor");
  fins::ParamLoader extrinsic("super_lio.extrinsic");
  fins::ParamLoader hash_map("super_lio.hash_map");
  fins::ParamLoader kf("super_lio.kf");
  fins::ParamLoader submap("super_lio.submap");
  fins::ParamLoader output("super_lio.output");

  // Map params
  g_if_filter = map.get("if_filter", false);
  g_map_ds_size = map.get("ds_size", 0.5);

  // Sensor params
  g_lidar_type = sensor.get("lidar_type", 0);
  double temp_range_dis = sensor.get("blind", 0.0);
  g_blind2 = temp_range_dis * temp_range_dis;
  temp_range_dis = sensor.get("maxrange", 100.0);
  g_maxrange2 = temp_range_dis * temp_range_dis;
  g_filter_rate = sensor.get("filter_rate", 1);
  g_enable_downsample = sensor.get("enable_downsample", false);
  g_voxel_fliter_size = sensor.get("voxel_fliter_size", 0.2);
  g_gravity_norm = sensor.get("gravity_norm", 9.81);
  g_imu_type = sensor.get("imu_type", 0);
  g_imu_na = sensor.get("imu_na", 0.0);
  g_imu_ng = sensor.get("imu_ng", 0.0);
  g_imu_nba = sensor.get("imu_nba", 0.0);
  g_imu_nbg = sensor.get("imu_nbg", 0.0);

  // Extrinsic params
  std::vector<double> extrinsic_lidar_imu = extrinsic.get("lidar_imu", std::vector<double>(12, 0.0));
  V3 __t(extrinsic_lidar_imu[0], extrinsic_lidar_imu[1], extrinsic_lidar_imu[2]);
  std::vector<scalar> r_data(9);
  for (int i = 0; i < 9; ++i) r_data[i] = static_cast<scalar>(extrinsic_lidar_imu[3 + i]);
  M3 __R(r_data.data());
  g_lidar_imu = SE3(__R, __t);

  std::vector<double> extrinsic_odom_robo = extrinsic.get("odom_robo", std::vector<double>(6, 0.0));
  __t = V3(extrinsic_odom_robo[0], extrinsic_odom_robo[1], extrinsic_odom_robo[2]);
  auto temp_R = Eigen::AngleAxisd(extrinsic_odom_robo[5] * M_PI / 180.0, Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(extrinsic_odom_robo[4] * M_PI / 180.0, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(extrinsic_odom_robo[3] * M_PI / 180.0, Eigen::Vector3d::UnitX());
  g_odom_robo.R_ = temp_R.toRotationMatrix().cast<scalar>().transpose().eval();
  g_odom_robo = SE3(g_odom_robo.R_, __t);
  g_lidar_robo_yaw = Eigen::AngleAxisd(extrinsic_odom_robo[5] * M_PI / 180.0, Eigen::Vector3d::UnitZ()).toRotationMatrix().cast<scalar>();

  // Hash map params
  g_ivox_capacity = hash_map.get("hash_capacity", 100000);
  g_ivox_resolution = hash_map.get("vox_resolution", 0.5);

  // KF params
  g_kf_type = kf.get("kf_type", 1);
  g_kf_max_iterations = kf.get("kf_max_iterations", 4);
  g_kf_align_gravity = kf.get("kf_align_gravity", true);
  g_kf_quit_eps = kf.get("kf_quit_eps", 0.0);

  // Output params
  g_2_robot = output.get("robot", false);
  g_planner_enable = output.get("planner", false);
  g_2_plan_env_world = output.get("plan_env_world", false);
  g_2_plan_env_body = output.get("plan_env_body", false);
  g_visual_map = output.get("map", true);
  g_visual_dense = output.get("dense", false);
  g_pub_step = output.get("pub_step", 0);

  LOG(INFO) << GREEN << " ---> [Params]: Load from Fins parameter server using ParamLoader structure." << RESET;
}

void livox2pcl(const livox_driver2::msg::CustomMsg::SharedPtr& msg, CloudPtr& point_cloud){
  point_cloud->clear();
  CloudPtr cloud_full(new PointCloudType());
  int plsize = msg->point_num;
  cloud_full->resize(plsize);
  point_cloud->reserve(plsize);
  std::vector<bool> is_valid_pt(plsize, false);
  std::vector<std::size_t> index(plsize - 1);
  std::iota(std::begin(index), std::end(index), 1);
#if defined(__cpp_lib_execution) || (defined(__cplusplus) && __cplusplus >= 201603L && __has_include(<execution>))
  std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const std::size_t &i) {
#else
  std::for_each(index.begin(), index.end(), [&](const std::size_t &i) {
#endif
    if((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00) {
      cloud_full->at(i).x = msg->points[i].x;
      cloud_full->at(i).y = msg->points[i].y;
      cloud_full->at(i).z = msg->points[i].z;
      cloud_full->at(i).intensity = msg->points[i].reflectivity;
      if ((abs(cloud_full->at(i).x - cloud_full->at(i - 1).x) > 1e-7) ||
          (abs(cloud_full->at(i).y - cloud_full->at(i - 1).y) > 1e-7) ||
          (abs(cloud_full->at(i).z - cloud_full->at(i - 1).z) > 1e-7)) {
        double normal_dis = cloud_full->at(i).x * cloud_full->at(i).x + 
                            cloud_full->at(i).y * cloud_full->at(i).y +
                            cloud_full->at(i).z * cloud_full->at(i).z;
        if(normal_dis > g_blind2 and normal_dis < g_maxrange2) is_valid_pt[i] = true;
      }
    }
  });
  for (int i = 1; i < plsize; i++) if (is_valid_pt[i]) point_cloud->points.push_back(cloud_full->at(i));
}

std::string lidarTypeToString(int type) {
  if (type <= 0 || type >= static_cast<int>(LID_TYPE_NAMES.size())) return "UNKNOWN";
  return LID_TYPE_NAMES[type];
}

inline bool validPoint(double x, double y, double z) {
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) return false;
  double d2 = x * x + y * y + z * z;
  return (d2 > g_blind2 && d2 < g_maxrange2);
}

inline double stampToSec(const builtin_interfaces::msg::Time& t) {
  return static_cast<double>(t.sec) + static_cast<double>(t.nanosec) * 1e-9;
}

ROSWrapper::ROSWrapper(fins::Node* node) : node_(node) {
  LoadParamFromFins();
  path_.header.frame_id = "odom";
  setupIO();
}

void ROSWrapper::setupIO(){ }

void ROSWrapper::imuHandler(const fins::Msg<sensor_msgs::msg::Imu>& msg){
  IMUData data;
  data.secs = stampToSec(msg->header.stamp);
  data.acc = V3(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
  data.gyr = V3(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
  if (data.secs < last_timestamp_imu_) {
    imu_buffer_.clear(); imu_buffer_.push_back(data); last_timestamp_imu_ = data.secs; return;
  }
  imu_buffer_.push_back(data);
  last_timestamp_imu_ = data.secs;
  DynamicState imu_state, robo_state;
  if(eskf_ && eskf_->Predict(data, imu_state, robo_state)){
    nav_msgs::msg::Odometry odom_robo;
    odom_robo.pose.pose.position.x = robo_state.p(0); odom_robo.pose.pose.position.y = robo_state.p(1); odom_robo.pose.pose.position.z = robo_state.p(2);
    Quat q = Quat(robo_state.R); q.normalize();
    odom_robo.pose.pose.orientation.x = q.x(); odom_robo.pose.pose.orientation.y = q.y(); odom_robo.pose.pose.orientation.z = q.z(); odom_robo.pose.pose.orientation.w = q.w();
    odom_robo.twist.twist.linear.x = imu_state.v(0); odom_robo.twist.twist.linear.y = imu_state.v(1); odom_robo.twist.twist.linear.z = imu_state.v(2);
    odom_robo.twist.twist.angular.x = imu_state.w(0); odom_robo.twist.twist.angular.y = imu_state.w(1); odom_robo.twist.twist.angular.z = imu_state.w(2);
    odom_robo.header.stamp = msg->header.stamp;
    odom_robo.header.frame_id = "odom";
    node_->send("robo_odom", odom_robo, fins::from_ros_time(msg->header.stamp));

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = msg->header.stamp;
    tf_msg.header.frame_id = "odom";
    tf_msg.child_frame_id = "imu";
    tf_msg.transform.translation.x = imu_state.p[0];
    tf_msg.transform.translation.y = imu_state.p[1];
    tf_msg.transform.translation.z = imu_state.p[2];
    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();
    tf_msg.transform.rotation.w = q.w();
    node_->send("tf", tf_msg, fins::from_ros_time(msg->header.stamp));
  }
}

void ROSWrapper::livoxHandler(const fins::Msg<livox_driver2::msg::CustomMsg>& msg){
  if(msg->point_num < 10) return;
  LidarData lidar_data;
  lidar_data.pc.reset(new pcl::PointCloud<LI2Sup::PointXTZIT>());
  lidar_data.pc->reserve(msg->point_num / g_filter_rate + 1);
  double offset_time = 0.0;
  for(std::size_t _i = 0; _i < msg->point_num; _i += g_filter_rate){
    auto& pt = msg->points[_i];
    if ((pt.tag & 0x30) == 0x10 || (pt.tag & 0x30) == 0x00){
      auto dis = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
      if(dis > g_blind2 && dis < g_maxrange2){
        offset_time = pt.offset_time * 1e-9;
        lidar_data.pc->emplace_back(pt.x, pt.y, pt.z, pt.reflectivity, offset_time);
      }
    }
  }
  lidar_data.start_time = stampToSec(msg->header.stamp);
  lidar_data.end_time = lidar_data.start_time + offset_time;
  lidar_buffer_.push_back(lidar_data);
}

void ROSWrapper::stdMsgHandler(const fins::Msg<sensor_msgs::msg::PointCloud2>& msg){
  if(msg->data.size() < 10) return;
  LidarData lidar_data; lidar_data.pc.reset(new pcl::PointCloud<LI2Sup::PointXTZIT>());
  double offset_time = 0.0;
  switch (g_lidar_type) {
  case LID_TYPE::HESAI16: {
    pcl::PointCloud<hesai_ros::Point> pl_orig; pcl::fromROSMsg(*msg, pl_orig);
    lidar_data.pc->reserve(pl_orig.size() / g_filter_rate + 1);
    const double time_begin = pl_orig.points[0].timestamp; lidar_data.start_time = time_begin;
    for(std::size_t i = 0; i < pl_orig.size(); i += g_filter_rate) {
      auto& pt = pl_orig.points[i]; if (!validPoint(pt.x, pt.y, pt.z)) continue;
      offset_time = pt.timestamp - time_begin; lidar_data.pc->emplace_back(pt.x, pt.y, pt.z, pt.intensity, offset_time);
    }
    lidar_data.end_time = time_begin + offset_time; break;
  }
  case LID_TYPE::VEL_NCLT: {
    pcl::PointCloud<NCLT::Point> pl_orig; pcl::fromROSMsg(*msg, pl_orig);
    lidar_data.pc->reserve(pl_orig.size() / g_filter_rate + 1);
    lidar_data.start_time = stampToSec(msg->header.stamp);
    for(std::size_t i = 0; i < pl_orig.size(); i += g_filter_rate){
      auto& pt = pl_orig.points[i]; if (!validPoint(pt.x, pt.y, pt.z)) continue;
      offset_time = pt.time * 1e-6; lidar_data.pc->emplace_back(pt.x, pt.y, pt.z, 1.0, offset_time);
    }
    lidar_data.end_time = lidar_data.start_time + offset_time; break;
  }
  case LID_TYPE::VELO16:
  case LID_TYPE::VELO32: {
    pcl::PointCloud<velodyne_ros::Point> pl_orig; pcl::fromROSMsg(*msg, pl_orig);
    lidar_data.pc->reserve(pl_orig.size() / g_filter_rate + 1);
    lidar_data.start_time = stampToSec(msg->header.stamp);
    for(std::size_t i = 0; i < pl_orig.size(); i += g_filter_rate){
      auto& pt = pl_orig.points[i]; if (!validPoint(pt.x, pt.y, pt.z)) continue;
      lidar_data.pc->emplace_back(pt.x, pt.y, pt.z, pt.intensity, pt.time);
    }
    lidar_data.end_time = lidar_data.start_time + lidar_data.pc->points.back().offset_time; break;
  }
  case OUSTER: {
    pcl::PointCloud<ouster_ros::Point> pl_orig; pcl::fromROSMsg(*msg, pl_orig);
    lidar_data.pc->reserve(pl_orig.size() / g_filter_rate + 1);
    lidar_data.start_time = stampToSec(msg->header.stamp);
    for(std::size_t i = 0; i < pl_orig.size(); i += g_filter_rate){
      auto& pt = pl_orig.points[i]; if (!validPoint(pt.x, pt.y, pt.z)) continue;
      offset_time = pt.t * 1e-9; lidar_data.pc->emplace_back(pt.x, pt.y, pt.z, pt.intensity, offset_time);
    }
    lidar_data.end_time = lidar_data.start_time + offset_time; break;
  }
  default: return;
  }
  lidar_buffer_.push_back(lidar_data);
}

bool ROSWrapper::sync_measure(MeasureGroup& meas){
  if (lidar_buffer_.empty() || imu_buffer_.empty()) return false;
  if (!lidar_pushed_) { meas.lidar = lidar_buffer_.front(); lidar_pushed_ = true; }
  if(last_timestamp_lidar_ > meas.lidar.end_time){ lidar_buffer_.pop_front(); lidar_pushed_ = false; return false; }
  if (last_timestamp_imu_ < meas.lidar.end_time) return false;
  double imu_time = imu_buffer_.front().secs; meas.imu.clear();
  while ((!imu_buffer_.empty()) && (imu_time < meas.lidar.end_time)) {
    imu_time = imu_buffer_.front().secs; if (imu_time > meas.lidar.end_time) break;
    meas.imu.push_back(imu_buffer_.front()); imu_buffer_.pop_front();
  }
  last_timestamp_lidar_ = meas.lidar.end_time; lidar_buffer_.pop_front(); lidar_pushed_ = false; return true;
}

void ROSWrapper::pub_odom(const NavState& state){
  V4 temp_q = state.R.coeffs();
  V3 robo_position = state.R.R_ * ( - g_odom_robo.R_ * g_odom_robo.t_) + state.p;
  if((last_path_point_ - robo_position).norm() > 0.1) {
    path_.header.stamp = fins::to_ros_time(fins::from_seconds(state.timestamp));
    geometry_msgs::msg::PoseStamped point;
    point.pose.position.x = state.p[0]; point.pose.position.y = state.p[1]; point.pose.position.z = state.p[2];
    point.pose.orientation.x = temp_q[0]; point.pose.orientation.y = temp_q[1]; point.pose.orientation.z = temp_q[2]; point.pose.orientation.w = temp_q[3];
    path_.poses.push_back(point); node_->send("path", path_, fins::from_seconds(state.timestamp));
    last_path_point_ = robo_position;
  }
}

void ROSWrapper::pub_cloud_world(const CloudPtr& pc, double time){
  sensor_msgs::msg::PointCloud2 cloud;
  pcl::toROSMsg(*pc, cloud);
  cloud.header.frame_id = "odom";
  cloud.header.stamp = fins::to_ros_time(fins::from_seconds(time));
  node_->send("cloud_registered", cloud, fins::from_seconds(time));
}

} // namespace LI2Sup
