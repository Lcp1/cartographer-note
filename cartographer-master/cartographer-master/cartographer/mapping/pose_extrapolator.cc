/*
 * Copyright 2017 The Cartographer Authors
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

#include "cartographer/mapping/pose_extrapolator.h"

#include <algorithm>

#include "absl/memory/memory.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

PoseExtrapolator::PoseExtrapolator(const common::Duration pose_queue_duration,
                                   double imu_gravity_time_constant)
      // 初始化
    : pose_queue_duration_(pose_queue_duration),
      gravity_time_constant_(imu_gravity_time_constant),
      cached_extrapolated_pose_{common::Time::min(),
                                transform::Rigid3d::Identity()} {}
// ---------------------------------------------------------------------------------------------
// 根据IMU数据来初始化一个PoseExtrapolator
std::unique_ptr<PoseExtrapolator> PoseExtrapolator::InitializeWithImu(
    const common::Duration pose_queue_duration,
    const double imu_gravity_time_constant, 
    const sensor::ImuData& imu_data) 
{
  //创建插值器
  auto extrapolator = absl::make_unique<PoseExtrapolator>(
                                              pose_queue_duration, 
                                              imu_gravity_time_constant);
  extrapolator->AddImuData(imu_data);//添加IMU数据
  extrapolator->imu_tracker_ =//IMU轨迹指针
                        absl::make_unique<ImuTracker>(imu_gravity_time_constant, 
                                                      imu_data.time);
  //添加imu线性加速度
  extrapolator->imu_tracker_->AddImuLinearAccelerationObservation(
                                                 imu_data.linear_acceleration);
  //添加imu角速度
  extrapolator->imu_tracker_->AddImuAngularVelocityObservation(
                                                 imu_data.angular_velocity);
  //添加时间
  extrapolator->imu_tracker_->Advance(imu_data.time);
  extrapolator->AddPose(imu_data.time,
                        //时间戳
                        transform::Rigid3d::Rotation(extrapolator->imu_tracker_->orientation()));
                        //插值器推算旋转量
  return extrapolator;//返回插值器
}
// ---------------------------------------------------------------------------------------------
//获取最新位姿时间
common::Time PoseExtrapolator::GetLastPoseTime() const
{
  if (timed_pose_queue_.empty()) 
  {
    return common::Time::min();
  }
  return timed_pose_queue_.back().time;
}
// ---------------------------------------------------------------------------------------------
//获取IMU轨迹插值器最新时间
common::Time PoseExtrapolator::GetLastExtrapolatedTime() const {
  if (!extrapolation_imu_tracker_) {
    return common::Time::min();
  }
  return extrapolation_imu_tracker_->time();
}
// --------------------------------------------------------------------------
//添加对应时间的位姿
void PoseExtrapolator::AddPose(const common::Time time,
                               const transform::Rigid3d& pose) 
{
  // 如果ImuTracker还没有建立，则需要建立一个ImuTracker
  if (imu_tracker_ == nullptr) {
    common::Time tracker_start = time;
    // 如果IMU数据队列不为空，则以当前时间和IMU数据中的最早时刻的较小值为初始时刻建立一个ImuTracker
    if (!imu_data_.empty()) {
      tracker_start = std::min(tracker_start, 
                               imu_data_.front().time);
    }
    imu_tracker_ =
        absl::make_unique<ImuTracker>(gravity_time_constant_, tracker_start);
  }
  // timed_pose_queue_队列加入时间和位姿
  timed_pose_queue_.push_back(TimedPose{time, pose});
  while (timed_pose_queue_.size() > 2 &&
         timed_pose_queue_[1].time <= time - pose_queue_duration_) 
  {
    timed_pose_queue_.pop_front();
  }
  UpdateVelocitiesFromPoses();
  // 通过时间获取imu_tracker_数据
  AdvanceImuTracker(time, imu_tracker_.get());
  //清楚旧数据,保持数据数量
  TrimImuData();
  TrimOdometryData();
  odometry_imu_tracker_ = absl::make_unique<ImuTracker>(*imu_tracker_);
  extrapolation_imu_tracker_ = absl::make_unique<ImuTracker>(*imu_tracker_);
}
// -----------------------------------------------------------------------------
// 添加IMU数据
void PoseExtrapolator::AddImuData(const sensor::ImuData& imu_data) {
  CHECK(timed_pose_queue_.empty() ||
        imu_data.time >= timed_pose_queue_.back().time);
  imu_data_.push_back(imu_data);
  //imu_data_数据足够的情况下清楚队列前一个数据,保持数据数量
  TrimImuData();
}
// ---------------------------------------------------------------------------------------------
void PoseExtrapolator::AddOdometryData(
    const sensor::OdometryData& odometry_data) 
{
  CHECK(timed_pose_queue_.empty() ||
        odometry_data.time >= timed_pose_queue_.back().time);
        //里程计数据时间是否晚于Pose队列的最新时间
  odometry_data_.push_back(odometry_data);//压入里程计数据队列
  // 释放多余信息
  TrimOdometryData();
  //信息量不够则返回
  if (odometry_data_.size() < 2) {
    return;
  }
  // TODO(whess): Improve by using more than just the last two odometry poses.
  // Compute extrapolation in the tracking frame.
  // 获取一前一后里程计数据来计算速率
  const sensor::OdometryData& odometry_data_oldest = odometry_data_.front();
  const sensor::OdometryData& odometry_data_newest = odometry_data_.back();
  // 时间差
  const double odometry_time_delta =
      common::ToSeconds(odometry_data_oldest.time - odometry_data_newest.time);
  // 位姿变化量
  const transform::Rigid3d odometry_pose_delta =
      odometry_data_newest.pose.inverse() * odometry_data_oldest.pose;
  // 计算旋转速率
  angular_velocity_from_odometry_ =
      transform::RotationQuaternionToAngleAxisVector(odometry_pose_delta.rotation()) /
      odometry_time_delta;
  if (timed_pose_queue_.empty()) {
    return;
  }
  // 计算平移速率
  const Eigen::Vector3d
      linear_velocity_in_tracking_frame_at_newest_odometry_time =
          odometry_pose_delta.translation() / odometry_time_delta;
          //在里程计坐标系中的线速度
  const Eigen::Quaterniond orientation_at_newest_odometry_time =
      timed_pose_queue_.back().pose.rotation() *
      ExtrapolateRotation(odometry_data_newest.time,
                          odometry_imu_tracker_.get());
                          //计算里程计坐标系与基准坐标系的变化姿态
  linear_velocity_from_odometry_ =
      orientation_at_newest_odometry_time *
      linear_velocity_in_tracking_frame_at_newest_odometry_time;
}
// ---------------------------------------------------------------------------------------------
// 根据当前时间推算出平移和旋转矩阵
transform::Rigid3d PoseExtrapolator::ExtrapolatePose(const common::Time time) 
{
  const TimedPose& newest_timed_pose = timed_pose_queue_.back();
  CHECK_GE(time, newest_timed_pose.time);
  //如果位姿的时间不是最新的 则根据当前时间推算出平移和旋转矩阵
  // 意思是查不到的时候需要插值b
  if (cached_extrapolated_pose_.time != time) {
    const Eigen::Vector3d translation =
        ExtrapolateTranslation(time) + newest_timed_pose.pose.translation();
    const Eigen::Quaterniond rotation =
        newest_timed_pose.pose.rotation() *
        ExtrapolateRotation(time, extrapolation_imu_tracker_.get());
    cached_extrapolated_pose_ =
    //平移 旋转 结构体
        TimedPose{time, 
                  transform::Rigid3d{translation, rotation}};//转换成位姿
  }
  return cached_extrapolated_pose_.pose;
}
// ---------------------------------------------------------------------------------------------
// 估计重力方向
Eigen::Quaterniond PoseExtrapolator::EstimateGravityOrientation(
                                               const common::Time time) 
{
  ImuTracker imu_tracker = *imu_tracker_;
  AdvanceImuTracker(time, &imu_tracker);
  return imu_tracker.orientation();
}
// ---------------------------------------------------------------------------------------------
//更新位姿的线速度
// 取出timed_pose_queue_这个队列中最早和最新的两个Pose做差，然后除以时间得到机器人的速度。
void PoseExtrapolator::UpdateVelocitiesFromPoses() 
{
  if (timed_pose_queue_.size() < 2) {//检查位姿输入是否超过2,否则退出
    // We need two poses to estimate velocities.
    return;
  }
  //读取时间点前后位姿和时间间隔
  CHECK(!timed_pose_queue_.empty());
  const TimedPose& newest_timed_pose = timed_pose_queue_.back();
  const auto newest_time = newest_timed_pose.time;
  const TimedPose& oldest_timed_pose = timed_pose_queue_.front();
  const auto oldest_time = oldest_timed_pose.time;
  const double queue_delta = common::ToSeconds(newest_time - oldest_time);//时间间隔
  //判断位姿时间间隔是否在符合范围内
  if (queue_delta < common::ToSeconds(pose_queue_duration_)) {
    LOG(WARNING) << "Queue too short for velocity estimation. Queue duration: "
                 << queue_delta << " s";
    return;
  }
  const transform::Rigid3d& newest_pose = newest_timed_pose.pose;
  const transform::Rigid3d& oldest_pose = oldest_timed_pose.pose;
  //线性速度
  linear_velocity_from_poses_ =
      (newest_pose.translation() - oldest_pose.translation()) / queue_delta;
  //角速度
  angular_velocity_from_poses_ =
      transform::RotationQuaternionToAngleAxisVector(
          oldest_pose.rotation().inverse() * newest_pose.rotation()) /
      queue_delta;
}
// ---------------------------------------------------------------------------------------------
// 释放上一个IMU数据，保持只有两个数据
void PoseExtrapolator::TrimImuData() {
  while (imu_data_.size() > 1 && 
         !timed_pose_queue_.empty() &&
         imu_data_[1].time <= timed_pose_queue_.back().time) 
  {
    imu_data_.pop_front();
  }
}
// ---------------------------------------------------------------------------------------------
// 释放前一个里程计数据
void PoseExtrapolator::TrimOdometryData() {
  while (odometry_data_.size() > 2 && 
        !timed_pose_queue_.empty() &&
         odometry_data_[1].time <= timed_pose_queue_.back().time) 
  {
    odometry_data_.pop_front();
  }
}
// ---------------------------------------------------------------------------------------------
// 通过时间获取IMU数据
void PoseExtrapolator::AdvanceImuTracker(const common::Time time,
                                         ImuTracker* const imu_tracker) const 
{
  CHECK_GE(time, imu_tracker->time());
  // 如果IMU数据为空 或者IMU数据时间比当前时间还要新,就添加当前IMU数据
  if (imu_data_.empty() || time < imu_data_.front().time) 
  {
    // There is no IMU data until 'time', so we advance the ImuTracker and use
    // the angular velocities from poses and fake gravity to help 2D stability.
    imu_tracker->Advance(time);//添加时间
    imu_tracker->AddImuLinearAccelerationObservation(Eigen::Vector3d::UnitZ());
    // 添加IMU线性加速度观测值
    imu_tracker->AddImuAngularVelocityObservation(
        odometry_data_.size() < 2 ? angular_velocity_from_poses_
                                  : angular_velocity_from_odometry_);
    // 添加IMU角速度观测值
    return;
  }
  // 如果当前IMU时间比IMU数据集时间最前面一个还要早,则IMU轨迹时间太晚了,用IMU数据集时间作为IMU轨迹的第一时间
  if (imu_tracker->time() < imu_data_.front().time) {
    // Advance to the beginning of 'imu_data_'.
    imu_tracker->Advance(imu_data_.front().time);
  }
  //然后依次取出IMU数据队列中的数据，更新ImuTracker，直到IMU数据的时间比指定时间time要晚。
  auto it = std::lower_bound(
      imu_data_.begin(), 
      imu_data_.end(),
      imu_tracker->time(),
      [](const sensor::ImuData& imu_data, const common::Time& time) 
                                 { return imu_data.time < time;}
      );
  // 一个一个时间段 读取IMU数据
  while (it != imu_data_.end() && it->time < time) {
    imu_tracker->Advance(it->time);
    imu_tracker->AddImuLinearAccelerationObservation(it->linear_acceleration);
    imu_tracker->AddImuAngularVelocityObservation(it->angular_velocity);
    ++it;
  }
  imu_tracker->Advance(time);
}
// ---------------------------------------------------------------------------------------------
// 通过位姿插值器获取旋转方向
Eigen::Quaterniond PoseExtrapolator::ExtrapolateRotation(
                                        const common::Time time, 
                                        ImuTracker* const imu_tracker) const 
{
  CHECK_GE(time, imu_tracker->time());
  AdvanceImuTracker(time, imu_tracker);
    // 上一时刻的姿态
  const Eigen::Quaterniond last_orientation = imu_tracker_->orientation();//获取全局变量IMU旋转方向
    // 求取姿态变化量：上一时刻姿态的逆乘以当前的姿态
  return last_orientation.inverse() * imu_tracker->orientation();//全局乘以局部 进行更新
}
// ---------------------------------------------------------------------------------------------
// 解算位移的增长量
Eigen::Vector3d PoseExtrapolator::ExtrapolateTranslation(common::Time time) {
    // 取出Pose队列中最新的时刻
  const TimedPose& newest_timed_pose = timed_pose_queue_.back();
  const double extrapolation_delta =  // 算时间差
      common::ToSeconds(time - newest_timed_pose.time);
  if (odometry_data_.size() < 2) {
    return extrapolation_delta * linear_velocity_from_poses_;//没有里程计信息时间差乘以位姿估计速度得到平移量
  }
  return extrapolation_delta * linear_velocity_from_odometry_;
  //有里程计信息 就将时间差乘以里程计估计速度得到平移量
}
// ---------------------------------------------------------------------------------------------
PoseExtrapolator::ExtrapolationResult PoseExtrapolator::ExtrapolatePosesWithGravity(
    const std::vector<common::Time>& times) 
{
  std::vector<transform::Rigid3f> poses;
  //读取插值器存储对应时间位姿
  for (auto it = times.begin(); it != std::prev(times.end()); ++it) {
    poses.push_back(ExtrapolatePose(*it).cast<float>());
  }
  //根据里程计节点数量选择估计速度
  const Eigen::Vector3d current_velocity = odometry_data_.size() < 2
                                               ? linear_velocity_from_poses_
                                               : linear_velocity_from_odometry_;
  return ExtrapolationResult{poses, 
                             ExtrapolatePose(times.back()),
                             current_velocity,
                             EstimateGravityOrientation(times.back())};
}

}  // namespace mapping
}  // namespace cartographer
