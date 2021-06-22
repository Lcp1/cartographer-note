/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


// 这个函数负责开启订阅各个传感器的消息，从而在消息回调函数中开启主要处理工作！注意：
// 所有传感器的回调函数中会通过map_builder_bridge_.sensor_bridges_调用相关库函数。
#include "cartographer_ros/sensor_bridge.h"

#include "absl/memory/memory.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/time_conversion.h"

namespace cartographer_ros {

namespace carto = ::cartographer;

using carto::transform::Rigid3d;

namespace {

const std::string& CheckNoLeadingSlash(const std::string& frame_id) {
  if (frame_id.size() > 0) {
    CHECK_NE(frame_id[0], '/') << "The frame_id " << frame_id
                               << " should not start with a /. See 1.7 in "
                                  "http://wiki.ros.org/tf2/Migration.";
  }
  return frame_id;
}

}  // namespace

SensorBridge::SensorBridge(
    const int num_subdivisions_per_laser_scan,       //每次激光扫描的细分数
    const std::string& tracking_frame,               //跟踪骨架
    const double lookup_transform_timeout_sec,       //查找转换超时时间 单位秒
    const double tf2_ros::Buffer* const tf_buffer,   //tf侦听器：一旦创建了侦听器，它就开始通过连接接收tf2转换，并对它们进行长达10秒的缓冲
    carto::mapping::TrajectoryBuilderInterface* const trajectory_builder)  //轨迹构建器接口
    : num_subdivisions_per_laser_scan_(num_subdivisions_per_laser_scan),     //初始化全局变量
      tf_bridge_(tracking_frame, lookup_transform_timeout_sec, tf_buffer),   //初始化全局变量
      trajectory_builder_(trajectory_builder) {}                             //初始化全局变量
//  ----------------------------------ToOdometryData-----------------------------------
// 获取里程计信息 time pose
std::unique_ptr<carto::sensor::OdometryData> SensorBridge::ToOdometryData(
                      const nav_msgs::Odometry::ConstPtr& msg) {
  const carto::common::Time time = FromRos(msg->header.stamp);                //读取时间戳
  const auto sensor_to_tracking = tf_bridge_.LookupToTracking(                // 寻找位姿
      time, CheckNoLeadingSlash(msg->child_frame_id));
  if (sensor_to_tracking == nullptr) {                                        //叛变是否找到位姿，否则退出
    return nullptr;
  }
  return absl::make_unique<carto::sensor::OdometryData>(                      //返回一个含有时间和位姿的结构体
      carto::sensor::OdometryData{
          time, ToRigid3d(msg->pose.pose) * sensor_to_tracking->inverse()});
          // OdometryData 结构体  时间和位姿
}
//  ----------------------------------ToOdometryData-----------------------------------
//  ----------------------------------HandleOdometryMessage-----------------------------------
// HandleOdometryMessage 进入函数 ToOdometryData 获取（位姿、时间)共享指针
// 处理里程计信息  进入 AddSensorData
// TrajectoryBuilderInterface 中的虚函数AddSensorData实例化
void SensorBridge::HandleOdometryMessage(
    const std::string& sensor_id, const nav_msgs::Odometry::ConstPtr& msg) {
  std::unique_ptr<carto::sensor::OdometryData> odometry_data =
      ToOdometryData(msg);
  if (odometry_data != nullptr) {
    trajectory_builder_->AddSensorData(                  
        sensor_id,
        carto::sensor::OdometryData{odometry_data->time, odometry_data->pose});
  }
}
//  ----------------------------------HandleOdometryMessage-----------------------------------
//  ----------------------------------HandleNavSatFixMessage-----------------------------------
// 处理GPS信息
void SensorBridge::HandleNavSatFixMessage(
    const std::string& sensor_id,                                          //传感器ID
     const sensor_msgs::NavSatFix::ConstPtr& msg) {                        //GPS信息
  const carto::common::Time time = FromRos(msg->header.stamp);             //从ROS读取GPS时间戳
  if (msg->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX) {    //判断信息状态 如果不是固定的
    trajectory_builder_->AddSensorData(
        sensor_id,
        carto::sensor::FixedFramePoseData{time, absl::optional<Rigid3d>()});//  结构体 时间和位姿
    return;
  }

  if (!ecef_to_local_frame_.has_value()) {                                  //地心地固坐标系到局部框架如果没数据就赋值
    ecef_to_local_frame_ =                                        
        ComputeLocalFrameFromLatLong(msg->latitude, msg->longitude);
    LOG(INFO) << "Using NavSatFix. Setting ecef_to_local_frame with lat = "
              << msg->latitude << ", long = " << msg->longitude << ".";
  }

  trajectory_builder_->AddSensorData(
      sensor_id, carto::sensor::FixedFramePoseData{
                     time,                                                  //时间
                    //  经纬度到地心转换
                     absl::optional<Rigid3d>(Rigid3d::Translation(          //坐标 
                               ecef_to_local_frame_.value() *
                               LatLongAltToEcef(msg->latitude,              //纬度[度]
                                                msg->longitude,             //经度[度]
                                                msg->altitude)))});         //海拔
}

//  ----------------------------------HandleNavSatFixMessage-----------------------------------
//  ----------------------------------HandleLandmarkMessage-----------------------------------
// 出来路标信息  也通过AddSensorData 添加到轨迹跟踪器上
void SensorBridge::HandleLandmarkMessage(
    const std::string& sensor_id,
    const cartographer_ros_msgs::LandmarkList::ConstPtr& msg) {
  auto landmark_data = ToLandmarkData(*msg);//消息转成数据 时间和 结构体 LandmarkObservation
                                                            // struct LandmarkObservation {
                                                            //   std::string id;
                                                            //   transform::Rigid3d landmark_to_tracking_transform;
                                                            //   double translation_weight;
                                                            //   double rotation_weight;
                                                            // };

  auto tracking_from_landmark_sensor = tf_bridge_.LookupToTracking(      //返回空指针 或 旋转矩阵
      landmark_data.time, CheckNoLeadingSlash(msg->header.frame_id));
      // CheckNoLeadingSlash(msg->header.frame_id) 检查frame_id.size()是否不为零，并返回id
  
  if (tracking_from_landmark_sensor != nullptr) {                         //如果旋转矩阵不为空指针
                                                                          //把路标数据乘以旋转矩阵
    for (auto& observation : landmark_data.landmark_observations) {
      observation.landmark_to_tracking_transform =
          *tracking_from_landmark_sensor *
          observation.landmark_to_tracking_transform;
    }
  }
  // 加入到pose_graph_？？？？？？？？？？？？？？
  trajectory_builder_->AddSensorData(sensor_id, landmark_data);
}
//  ----------------------------------HandleLandmarkMessage-----------------------------------

//  ----------------------------------ToImuData-----------------------------------
//通过查找当前位姿 转换iMU数据的 线速度和加速度 
std::unique_ptr<carto::sensor::ImuData> SensorBridge::ToImuData(
    const sensor_msgs::Imu::ConstPtr& msg) {
  CHECK_NE(msg->linear_acceleration_covariance[0], -1)
      << "Your IMU data claims to not contain linear acceleration measurements "
         "by setting linear_acceleration_covariance[0] to -1. Cartographer "
         "requires this data to work. See "
         "http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html.";
  CHECK_NE(msg->angular_velocity_covariance[0], -1)
      << "Your IMU data claims to not contain angular velocity measurements "
         "by setting angular_velocity_covariance[0] to -1. Cartographer "
         "requires this data to work. See "
         "http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html.";

  const carto::common::Time time = FromRos(msg->header.stamp);
  const auto sensor_to_tracking = tf_bridge_.LookupToTracking(
      time, 
      CheckNoLeadingSlash(msg->header.frame_id));
  if (sensor_to_tracking == nullptr) {
    return nullptr;
  }
  CHECK(sensor_to_tracking->translation().norm() < 1e-5)
      << "The IMU frame must be colocated with the tracking frame. "
         "Transforming linear acceleration into the tracking frame will "
         "otherwise be imprecise.";
  //通过查找当前位姿 转换iMU数据的 线速度和加速度 
  return absl::make_unique<carto::sensor::ImuData>(carto::sensor::ImuData{
      time,                                                                  //时间
       sensor_to_tracking->rotation() * ToEigen(msg->linear_acceleration),   //线速度
      sensor_to_tracking->rotation() * ToEigen(msg->angular_velocity)});     //加速度
}
//  ----------------------------------ToImuData-----------------------------------

//  ----------------------------------HandleImuMessage-----------------------------------
// 通过ToImuData转换后 在通过AddSensorData存储数据
void SensorBridge::HandleImuMessage(const std::string& sensor_id,
                                    const sensor_msgs::Imu::ConstPtr& msg) {
  std::unique_ptr<carto::sensor::ImuData> imu_data = ToImuData(msg);         //转换IMU信息后
  if (imu_data != nullptr) {
    trajectory_builder_->AddSensorData(
        sensor_id,
        carto::sensor::ImuData{imu_data->time,                               //存取IMU数据
                               imu_data->linear_acceleration,  
                               imu_data->angular_velocity});
  }
}
//  ----------------------------------HandleImuMessage-----------------------------------

//  ----------------------------------HandleLaserScanMessage-----------------------------------
// 获取有效点云信息和时间 并通过HandleLaserScan 来处理
void SensorBridge::HandleLaserScanMessage(
    const std::string& sensor_id, const sensor_msgs::LaserScan::ConstPtr& msg) {
  carto::sensor::PointCloudWithIntensities point_cloud;
  carto::common::Time time;
  // 结构体  点云、时间
  std::tie(point_cloud, time) = ToPointCloudWithIntensities(*msg);           //获取 有效点云信息和时间戳
                                                                             //剔除异常点云
                                                                             //std::tie：创建左值引用的 tuple，
                                                                             //或将 tuple 解包为独立对象
                                                                             //返回值
                                                                             //含左值引用的 std::tuple 对象。
                                     // std::tie 可用于解包 std::pair ，因为 std::tuple 拥有从 pair 的转换赋值
                                     //ToPointCloudWithIntensities 返回的是 std::tuple 对象
  HandleLaserScan(sensor_id, time, msg->header.frame_id, point_cloud);
}
//  ----------------------------------HandleLaserScanMessage-----------------------------------

//  ----------------------------------HandleMultiEchoLaserScanMessage-----------------------------------
void SensorBridge::HandleMultiEchoLaserScanMessage(
    const std::string& sensor_id,
    const sensor_msgs::MultiEchoLaserScan::ConstPtr& msg) {
  carto::sensor::PointCloudWithIntensities point_cloud;
  carto::common::Time time;
  std::tie(point_cloud, time) = ToPointCloudWithIntensities(*msg);
  HandleLaserScan(sensor_id, time, msg->header.frame_id, point_cloud);
}
//  ----------------------------------HandleMultiEchoLaserScanMessage-----------------------------------

//  ----------------------------------HandlePointCloud2Message-----------------------------------
void SensorBridge::HandlePointCloud2Message(
    const std::string& sensor_id,
    const sensor_msgs::PointCloud2::ConstPtr& msg) {
  carto::sensor::PointCloudWithIntensities point_cloud;
  carto::common::Time time;
  std::tie(point_cloud, time) = ToPointCloudWithIntensities(*msg);               //同上
  HandleRangefinder(sensor_id, time, msg->header.frame_id, point_cloud.points);  
}
//  ---------------------------------HandlePointCloud2Message----------------------------------

//  --------------------------------tf_bridge()-----------------------------------
const TfBridge& SensorBridge::tf_bridge() const { return tf_bridge_; }
//  ----------------------------------tf_bridge()-----------------------------------

//  ----------------------------------HandleLaserScan-----------------------------------
// 按照时间增量管理点云，并且传送到HandleRangefinder处理 变量包括 具有时间点云容器、当前时间、frame_id、sensor_id
void SensorBridge::HandleLaserScan(
    const std::string& sensor_id, const carto::common::Time time,
    const std::string& frame_id,
    const carto::sensor::PointCloudWithIntensities& points) {
  if (points.points.empty()) {                                         //如果为空直接返回
    return;
  }
  CHECK_LE(points.points.back().time, 0.f);
  // TODO(gaschler): Use per-point time instead of subdivisions.
  //利用时间增量来管理激光点云 而不是 利用当前时间 点来管理点云
  for (int i = 0; i != num_subdivisions_per_laser_scan_; ++i) {        //直到循环次数达到一阵激光细分数
    const size_t start_index =
        points.points.size() * i / num_subdivisions_per_laser_scan_;
    const size_t end_index =
        points.points.size() * (i + 1) / num_subdivisions_per_laser_scan_;
        // 数组 的 时间和点云位姿 结构体
    carto::sensor::TimedPointCloud subdivision( 
        points.points.begin() + start_index,
        points.points.begin() + end_index);
                                                                        //TimedPointCloud表示为具有时间的点云
                                                                        //   struct TimedRangefinderPoint {
                                                                        //      Eigen::Vector3f position;
                                                                        //      loat time;
                                                                        //    };
    if (start_index == end_index) {                                     //如果时间相同则返回for
      continue;
    }
    const double time_to_subdivision_end = subdivision.back().time;
    // `subdivision_time` is the end of the measurement so sensor::Collator will
    // send all other sensor data first.
    const carto::common::Time subdivision_time =
        time + carto::common::FromSeconds(time_to_subdivision_end);      //表示当前shi
    auto it = sensor_to_previous_subdivision_time_.find(sensor_id);
    if (it != sensor_to_previous_subdivision_time_.end() &&
        it->second >= subdivision_time) {       //it->second表示时间  如果大于subdivision_time 表示 时间已经超出范围
      LOG(WARNING) << "Ignored subdivision of a LaserScan message from sensor "
                   << sensor_id << " because previous subdivision time "
                   << it->second << " is not before current subdivision time "
                   << subdivision_time;
      continue;
    }
    sensor_to_previous_subdivision_time_[sensor_id] = subdivision_time;
    for (auto& point : subdivision) {
      point.time -= time_to_subdivision_end;                            //时间从后往前推
    } 
    CHECK_EQ(subdivision.back().time, 0.f);                             //检查最后的时间是否刚好为0
    HandleRangefinder(sensor_id,                                        //调用AddSensorData 添加点云数据
                      subdivision_time,                                 
                      frame_id, 
                      subdivision);                                      //vector容器
  }
}
//  ----------------------------------HandleLaserScan-----------------------------------

//  --------------------------------HandleRangefinder----------------------------------
void SensorBridge::HandleRangefinder(
    const std::string& sensor_id, const carto::common::Time time,
    const std::string& frame_id, const carto::sensor::TimedPointCloud& ranges) {
  if (!ranges.empty()) {
    CHECK_LE(ranges.back().time, 0.f);
  }
  // 调用监听器的求变换矩阵函数 lookupTransform 
  const auto sensor_to_tracking = tf_bridge_.LookupToTracking(time, 
                                                              CheckNoLeadingSlash(frame_id));  //检查frame_id数量是否大于0
  if (sensor_to_tracking != nullptr) {                                     //如果数据不为空
    trajectory_builder_->AddSensorData(
        sensor_id, carto::sensor::TimedPointCloudData{
                       time, 
                       sensor_to_tracking->translation().cast<float>(),    //转换为float型
                       carto::sensor::TransformTimedPointCloud( ranges,sensor_to_tracking->cast<float>())});
  }
}

}  // namespace cartographer_ros
