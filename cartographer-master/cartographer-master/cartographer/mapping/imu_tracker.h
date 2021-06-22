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

#ifndef CARTOGRAPHER_MAPPING_IMU_TRACKER_H_
#define CARTOGRAPHER_MAPPING_IMU_TRACKER_H_

#include "Eigen/Geometry"
#include "cartographer/common/time.h"

namespace cartographer {
namespace mapping {
// ImuTracker的主要作用就是根据IMU的读数维护传感器当前的姿态、
// 线加速度(经过重力校正的)、当前姿态、重力方向、角速度等量。
// 这些量都是以ImuTracker刚建立时的那一时刻IMU本身的坐标系为基准坐标系。
// Keeps track of the orientation using angular velocities and linear
// accelerations from an IMU. Because averaged linear acceleration (assuming
// slow movement) is a direct measurement of gravity, roll/pitch does not drift,
// though yaw does.
// 使用IMU的角速度和线加速度跟踪方向。
// 因为平均线加速度（假设缓慢移动）是重力的直接测量，
// 横滚/俯仰不会漂移，尽管偏航会漂移。
class ImuTracker {
 public:
  ImuTracker(double imu_gravity_time_constant, common::Time time);

  // Advances to the given 'time' and updates the orientation to reflect this.
//   根据给定的时间 更新方向
  void Advance(common::Time time);

  // Updates from an IMU reading (in the IMU frame).
//   添加IMU线性加速度
  void AddImuLinearAccelerationObservation(
      const Eigen::Vector3d& imu_linear_acceleration);
// 添加IMU角速度
  void AddImuAngularVelocityObservation(
      const Eigen::Vector3d& imu_angular_velocity);

  // Query the current time.
  //访问当前时间
  common::Time time() const { return time_; }

  // Query the current orientation estimate.
//   访问当前方向估计值
  Eigen::Quaterniond orientation() const { return orientation_; }

 private:
  const double imu_gravity_time_constant_;
  common::Time time_;
  common::Time last_linear_acceleration_time_;  //上一个线性加速度时刻
  Eigen::Quaterniond orientation_;              //当前姿态 
  Eigen::Vector3d gravity_vector_;              //当前重力方向
  Eigen::Vector3d imu_angular_velocity_;        //角速度
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_IMU_TRACKER_H_
