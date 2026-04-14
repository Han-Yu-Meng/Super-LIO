#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <fins/node.hpp>
#include "ros/ROSWrapper.h"
#include "lio/super_lio.h"

using namespace LI2Sup;

class SuperLIONode : public fins::Node {
public:
  void define() override {
    set_name("SuperLIO");
    set_description("Fast LiDAR-Inertial Odometry with Ground Condition Awareness");
    set_category("SLAM");

    register_input<sensor_msgs::msg::Imu>("imu", &SuperLIONode::on_imu);
    register_input<livox_driver2::msg::CustomMsg>("lidar", &SuperLIONode::on_livox);
    register_input<sensor_msgs::msg::PointCloud2>("lidar", &SuperLIONode::on_lidar);

    register_output<nav_msgs::msg::Odometry>("robo_odom");
    register_output<nav_msgs::msg::Path>("path");
    register_output<sensor_msgs::msg::PointCloud2>("cloud_registered");
    register_output<geometry_msgs::msg::TransformStamped>("tf");
  }

  void initialize() override {
    data_wrapper_ = std::make_shared<ROSWrapper>(this);
    lio_ = std::make_shared<SuperLIO>();
    lio_->setROSWrapper(data_wrapper_);
    lio_->init();

    is_running_ = true;
    has_new_data_ = false;
    mapping_thread_ = std::thread(&SuperLIONode::mapping_worker_loop, this);
  }

  void deinitialize() {
    is_running_ = false;
    trigger_cv_.notify_all();
    if (mapping_thread_.joinable()) {
      mapping_thread_.join();
    }
  }

  ~SuperLIONode() { deinitialize(); }

  void run() override {} // 重点关注：必须定义 run 方法，可以为空
  void pause() override {} // 重点关注：必须定义 pause 方法，可以为空
  void reset() override {} // 重点关注：必须定义 reset方法，可以为空

  void on_imu(const fins::Msg<sensor_msgs::msg::Imu>& msg) { 
    data_wrapper_->imuHandler(msg);
    notify_backend();
  }

  void on_livox(const fins::Msg<livox_driver2::msg::CustomMsg>& msg) {
    data_wrapper_->livoxHandler(msg);
    notify_backend();
  }

  void on_lidar(const fins::Msg<sensor_msgs::msg::PointCloud2>& msg) {
    data_wrapper_->stdMsgHandler(msg);
    notify_backend();
  }

private:
  void notify_backend() {
    {
      std::lock_guard<std::mutex> lock(trigger_mtx_);
      has_new_data_ = true;
    }
    trigger_cv_.notify_one();
  }

  void mapping_worker_loop() {
    while (is_running_) {
      {
        std::unique_lock<std::mutex> lock(trigger_mtx_);
        trigger_cv_.wait(lock, [this] { return !is_running_ || has_new_data_; });
        has_new_data_ = false;
      }

      if (!is_running_)
        break;

      if (lio_) {
        lio_->process();
      }
    }
  }

  ROSWrapper::Ptr data_wrapper_;
  std::shared_ptr<SuperLIO> lio_;

  std::thread mapping_thread_;
  std::mutex trigger_mtx_;
  std::condition_variable trigger_cv_;
  std::atomic<bool> is_running_{false};
  bool has_new_data_{false};
};

EXPORT_NODE(SuperLIONode)
DEFINE_PLUGIN_ENTRY()