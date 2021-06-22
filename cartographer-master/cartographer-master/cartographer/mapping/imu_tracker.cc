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

#include "cartographer/mapping/imu_tracker.h"

#include <cmath>
#include <limits>

#include "cartographer/common/math.h"
#include "cartographer/mapping/internal/eigen_quaterniond_from_two_vectors.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"
// 更新参数是将局部参数赋值到全局参数
namespace cartographer {
namespace mapping {
//初始化参数
ImuTracker::ImuTracker(const double imu_gravity_time_constant,
                       const common::Time time)
    : imu_gravity_time_constant_(imu_gravity_time_constant),
      time_(time),
      last_linear_acceleration_time_(common::Time::min()),
      orientation_(Eigen::Quaterniond::Identity()),
      gravity_vector_(Eigen::Vector3d::UnitZ()),
      imu_angular_velocity_(Eigen::Vector3d::Zero()) {}
//更新全局变量参数
void ImuTracker::Advance(const common::Time time) 
{
  CHECK_LE(time_, time);
  const double delta_t = common::ToSeconds(time - time_);//时间间隔
  const Eigen::Quaterniond rotation =//计算旋转角度
                    transform::AngleAxisVectorToRotationQuaternion(
                                   Eigen::Vector3d(imu_angular_velocity_ * delta_t));
  orientation_ = (orientation_ * rotation).normalized();//计算方向
  gravity_vector_ = rotation.conjugate() * gravity_vector_;//计算重力方向
  time_ = time;//更新全局时间
}
// 根据传感器读数更新传感器的最新状态，得到经过重力校正的线加速度、角速度等。
void ImuTracker::AddImuLinearAccelerationObservation(
    const Eigen::Vector3d& imu_linear_acceleration) 
{
  // Update the 'gravity_vector_' with an exponential moving average using the
  // 'imu_gravity_time_constant'.
  //加速时间间隔足够下,计算时间间隔
  const double delta_t =
      last_linear_acceleration_time_ > common::Time::min()
                                        ? common::ToSeconds(time_ - last_linear_acceleration_time_)
                                        : std::numeric_limits<double>::infinity();
  last_linear_acceleration_time_ = time_;//更新线性加速度时间
  // 计算权重    即 时间间隔越久,exp接近零  alpha 越接近1  那么 重力越倾向于 最新测量的结果 
  const double alpha = 1. - std::exp(-delta_t / imu_gravity_time_constant_);
  // Y(n)=αX(n) + (1-α)Y(n-1)
  gravity_vector_ = (1. - alpha) * gravity_vector_ + alpha * imu_linear_acceleration;
  // Change the 'orientation_' so that it agrees with the current
  // 'gravity_vector_'.
  // 根据新的重力向量,更新对应的旋转矩阵.
  const Eigen::Quaterniond rotation = FromTwoVectors(
                    gravity_vector_, orientation_.conjugate() * Eigen::Vector3d::UnitZ());
  orientation_ = (orientation_ * rotation).normalized();
  CHECK_GT((orientation_ * gravity_vector_).z(), 0.);
  CHECK_GT((orientation_ * gravity_vector_).normalized().z(), 0.99);
}
//更新全局角速度
void ImuTracker::AddImuAngularVelocityObservation(
    const Eigen::Vector3d& imu_angular_velocity) {
  imu_angular_velocity_ = imu_angular_velocity;//可直接测得角度
}

}  // namespace mapping
}  // namespace cartographer
