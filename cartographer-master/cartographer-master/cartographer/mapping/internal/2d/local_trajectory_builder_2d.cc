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

#include "cartographer/mapping/internal/2d/local_trajectory_builder_2d.h"

#include <limits>
#include <memory>

#include "absl/memory/memory.h"
#include "cartographer/metrics/family_factory.h"
#include "cartographer/sensor/range_data.h"

namespace cartographer {
namespace mapping {

static auto* kLocalSlamLatencyMetric = metrics::Gauge::Null();
static auto* kLocalSlamRealTimeRatio = metrics::Gauge::Null();
static auto* kLocalSlamCpuRealTimeRatio = metrics::Gauge::Null();
static auto* kRealTimeCorrelativeScanMatcherScoreMetric =metrics::Histogram::Null();
static auto* kCeresScanMatcherCostMetric = metrics::Histogram::Null();
static auto* kScanMatcherResidualDistanceMetric = metrics::Histogram::Null();
static auto* kScanMatcherResidualAngleMetric = metrics::Histogram::Null();

//参数初始化
LocalTrajectoryBuilder2D::LocalTrajectoryBuilder2D(
    const proto::LocalTrajectoryBuilderOptions2D& options,
    const std::vector<std::string>& expected_range_sensor_ids)
    : options_(options),
      active_submaps_(options.submaps_options()),
      motion_filter_(options_.motion_filter_options()),
      real_time_correlative_scan_matcher_(
                     options_.real_time_correlative_scan_matcher_options()),
      ceres_scan_matcher_(options_.ceres_scan_matcher_options()),
      range_data_collator_(expected_range_sensor_ids) 
      {}

LocalTrajectoryBuilder2D::~LocalTrajectoryBuilder2D() 
{}
// ----------------TransformToGravityAlignedFrameAndFilter------------------
// 根据重力加速度方向进行旋转投影，仅保留一定高度，并进行采样滤波
sensor::RangeData
LocalTrajectoryBuilder2D::TransformToGravityAlignedFrameAndFilter(
    const transform::Rigid3f& transform_to_gravity_aligned_frame,
    const sensor::RangeData& range_data) const 
{
  const sensor::RangeData cropped =
      sensor::CropRangeData(sensor::TransformRangeData(
                                                  range_data, 
                                                  transform_to_gravity_aligned_frame),
                                                  options_.min_z(), 
                                                  options_.max_z());
  return sensor::RangeData
  {
      cropped.origin,
      sensor::VoxelFilter(cropped.returns, options_.voxel_filter_size()),
      sensor::VoxelFilter(cropped.misses, options_.voxel_filter_size())
  };
}
// -----------------------------------------------------------------
//扫描匹配
std::unique_ptr<transform::Rigid2d> LocalTrajectoryBuilder2D::ScanMatch(
                                      const common::Time time, 
                                      const transform::Rigid2d& pose_prediction,
                                      const sensor::PointCloud& filtered_gravity_aligned_point_cloud) 
{//判断数据是否为空,如果是则直接返回估计位置
  if (active_submaps_.submaps().empty()) 
  {
    return absl::make_unique<transform::Rigid2d>(pose_prediction);
  }
  // 获取old submap
  std::shared_ptr<const Submap2D> matching_submap =
                                    active_submaps_.submaps().front();
  // The online correlative scan matcher will refine the initial estimate for
  // the Ceres scan matcher.
  //将估计位姿作为初始位姿
  transform::Rigid2d initial_ceres_pose = pose_prediction;
//如果相关匹配使能,则进行相关匹配,并作为优化的初始值
  if (options_.use_online_correlative_scan_matching()) 
  {
    //计算匹配分数
    const double score = real_time_correlative_scan_matcher_.Match(
                                                                pose_prediction, 
                                                                filtered_gravity_aligned_point_cloud,
                                                                *matching_submap->grid(), 
                                                                &initial_ceres_pose);
    kRealTimeCorrelativeScanMatcherScoreMetric->Observe(score);
  }
//进行ceres优化匹配
 // 调用Ceres库来实现匹配。匹配结果放到pose_observation中
  auto pose_observation = absl::make_unique<transform::Rigid2d>();
  ceres::Solver::Summary summary;
  ceres_scan_matcher_.Match(pose_prediction.translation(), 
                            initial_ceres_pose,
                            filtered_gravity_aligned_point_cloud,
                            *matching_submap->grid(), 
                            pose_observation.get(),
                            &summary);
// 计算估计值和观测值误差  // 统计残差                          
  if (pose_observation) 
  {
    kCeresScanMatcherCostMetric->Observe(summary.final_cost);
    const double residual_distance =  (pose_observation->translation() - 
                                       pose_prediction.translation()).norm();
    kScanMatcherResidualDistanceMetric->Observe(residual_distance);
    const double residual_angle =   std::abs(pose_observation->rotation().angle() -
                                            pose_prediction.rotation().angle());
    kScanMatcherResidualAngleMetric->Observe(residual_angle);
  }
  return pose_observation;
}

// ----------------------AddRangeData--------------------------------
//插入顶层的新的激光数据
std::unique_ptr<LocalTrajectoryBuilder2D::MatchingResult> 
 //MatchingResult 时间戳 节点位置对应的点云 子图位置 子图序列  节点信息
LocalTrajectoryBuilder2D::AddRangeData(
    const std::string& sensor_id,
    const sensor::TimedPointCloudData& unsynchronized_data) 
      /* TimedPointCloudData定义如下
       struct TimedPointCloudData {
          common::Time time;//时间
          Eigen::Vector3f origin;//原点
          sensor::TimedPointCloud ranges;//点云
      };
      */
{
  //添加到多种激光融合集合器
  auto synchronized_data =
         //表示同步后的数据
         range_data_collator_.AddRangeData(sensor_id, 
                                           unsynchronized_data);
  //检查同步数据是否为空
  if (synchronized_data.ranges.empty())
  {
    LOG(INFO) << "Range data collator filling buffer.";
    return nullptr;
  }
//获取同步时间
// 取点云获取的时间为基准为PoseExtrapolator初始化
  const common::Time& time = synchronized_data.time;
  // Initialize extrapolator now if we do not ever use an IMU.
    // 如果没有imu数据，直接初始化推算器
  if (!options_.use_imu_data()) 
    InitializeExtrapolator(time);
//如果插值器为空 就报警插值器还没初始化,并返回空指针
  if (extrapolator_ == nullptr) 
  {
    // Until we've initialized the extrapolator with our first IMU message, we
    // cannot compute the orientation of the rangefinder.
    LOG(INFO) << "Extrapolator not yet initialized.";
    return nullptr;
  }
//同步数据集 激光数据为空
  CHECK(!synchronized_data.ranges.empty());
  // TODO(gaschler): Check if this can strictly be 0.
// 检测时间是否为零
  CHECK_LE(synchronized_data.ranges.back().point_time.time, 0.f);
// 第一个点的时间就等于点云集获取的时间加上第一个点记录的相对时间
  const common::Time time_first_point =
      time +
      common::FromSeconds(synchronized_data.ranges.front().point_time.time);
      // 同步时间
  if (time_first_point < extrapolator_->GetLastPoseTime()) 
  {
    LOG(INFO) << "Extrapolator is still initializing.";
    return nullptr;
  }

  std::vector<transform::Rigid3f> range_data_poses;
  // 对集合进行分配内存
  range_data_poses.reserve(synchronized_data.ranges.size());
  bool warned = false;
  for (const auto& range : synchronized_data.ranges) 
  {
       // 遍历每一个点云点的时间戳，理论上应晚于估计器上刻位置时间戳，否则说明传感器采集时间错误
    common::Time time_point = time + common::FromSeconds(range.point_time.time);
    if (time_point < extrapolator_->GetLastExtrapolatedTime()) 
    {
      if (!warned) {
        LOG(ERROR)
            << "Timestamp of individual range data point jumps backwards from "
            << extrapolator_->GetLastExtrapolatedTime() << " to " << time_point;
        warned = true;
      }
      time_point = extrapolator_->GetLastExtrapolatedTime();
    }
    // 根据每个点的时间戳估计点云点对应的位置并进行缓存
    range_data_poses.push_back(
        extrapolator_->ExtrapolatePose(time_point).cast<float>());
  }
//没有初始化,就需要初始化
  if (num_accumulated_ == 0) {
    // 'accumulated_range_data_.origin' is uninitialized until the last
    // accumulation.
    accumulated_range_data_ = sensor::RangeData{{}, {}, {}};
  }

  // Drop any returns below the minimum range and convert returns beyond the
  // maximum range into misses.
  //通过障碍点相对雷达坐标与雷达到原点的坐标 计算出障碍点的坐标,并计算出模长
  for (size_t i = 0; i < synchronized_data.ranges.size(); ++i) 
  {
    // 提取每个点云的位置 包括时间
    const sensor::TimedRangefinderPoint& hit =
        synchronized_data.ranges[i].point_time;
    //提取点云对应原点的坐标,对每个点云进行去畸变
    const Eigen::Vector3f origin_in_local =
        range_data_poses[i] *          //每个点的时间戳估计点云点对应的位置
        synchronized_data.origins.at(synchronized_data.ranges[i].origin_index);
    // 对此点进行畸变矫正，并转换为pose，不包含时间戳,计算障碍点在submap位姿
    sensor::RangefinderPoint hit_in_local =
        range_data_poses[i] * sensor::ToRangefinderPoint(hit);
    // 计算障碍点到原点距离
    const Eigen::Vector3f delta = hit_in_local.position - origin_in_local;
    const float range = delta.norm();//计算直线距离
    //这主要是增加传感器信息的通用性,在其他SLAM算法中直接利用数据的模长,本算法是重复进行了一次运算

    if (range >= options_.min_range()) //存储有效范围点
    {
      if (range <= options_.max_range()) 
      {
        accumulated_range_data_.returns.push_back(hit_in_local);
      } 
      else 
      {//超出距离就将数据放进miss队列,距离值设为配置值
        hit_in_local.position =
            origin_in_local +
            options_.missing_data_ray_length() / range * delta;
        accumulated_range_data_.misses.push_back(hit_in_local);
      }
    }
  }
  //激光点云累积个数
  ++num_accumulated_;
  //如果点云个数大于配置设定值,进行处理
  if (num_accumulated_ >= options_.num_accumulated_range_data()) 
  {
    //读取最新时间戳
    const common::Time current_sensor_time = synchronized_data.time;
    //定义两次的时间间隔
    absl::optional<common::Duration> sensor_duration;
    if (last_sensor_time_.has_value()) 
    {
      //与上一次的时间间隔
      sensor_duration = current_sensor_time - last_sensor_time_.value();
    }
    //更新上一次时间,将当前时间设为下次使用的上一次时间
    last_sensor_time_ = current_sensor_time;
    // 初始化计数器
    num_accumulated_ = 0;
    //从插值器获取重力方向的向量
    const transform::Rigid3d gravity_alignment = transform::Rigid3d::Rotation(
        extrapolator_->EstimateGravityOrientation(time));
    // TODO(gaschler): This assumes that 'range_data_poses.back()' is at time
    // 'time'.
    //估计最后的预测点位置作为校正后的点云原点坐标
    accumulated_range_data_.origin = range_data_poses.back().translation();
    return AddAccumulatedRangeData(
        time,
        //进行降采样滤波
        TransformToGravityAlignedFrameAndFilter(
            //计算重力方向的位姿
            gravity_alignment.cast<float>() * range_data_poses.back().inverse(),
            accumulated_range_data_),
        gravity_alignment, 
        sensor_duration);
  }
  return nullptr;
  //由上可知,激光匹配个数不是一帧,而是配置参数设定值
}
// ---------------------------------------------------------------------------------
// -------------------------AddAccumulatedRangeData--------------------------------
/*
input:  
1.时间戳
2.经水平投影后的点云数据
3.重力加速度旋转向量
4.与上次处理的时间间隔
 */
std::unique_ptr<LocalTrajectoryBuilder2D::MatchingResult>
 //MatchingResult 时间戳 节点位置对应的点云 子图位置 子图序列  节点信息
LocalTrajectoryBuilder2D::AddAccumulatedRangeData(
    const common::Time time,
    const sensor::RangeData& gravity_aligned_range_data,
    const transform::Rigid3d& gravity_alignment,
    const absl::optional<common::Duration>& sensor_duration) 
{
  if (gravity_aligned_range_data.returns.empty()) 
  //重力方向数据为空则返回null
  {
    LOG(WARNING) << "Dropped empty horizontal range data.";
    return nullptr;
  }
  // Computes a gravity aligned pose prediction.
  // 根据时间采用插值器获取推算出大约位置
  const transform::Rigid3d non_gravity_aligned_pose_prediction =
          extrapolator_->ExtrapolatePose(time);
  // 在重力方向投影到2d平面
  const transform::Rigid2d pose_prediction = transform::Project2D(
          non_gravity_aligned_pose_prediction * gravity_alignment.inverse());

  // 经过立体像素滤波获取点云
  // 默认size为0.5m， 最小个数200个，最远距离50m
  const sensor::PointCloud& filtered_gravity_aligned_point_cloud =
      sensor::AdaptiveVoxelFilter(gravity_aligned_range_data.returns,
                                  options_.adaptive_voxel_filter_options());
  if (filtered_gravity_aligned_point_cloud.empty()) 
  {
    return nullptr;
  }

  // local map frame <- gravity-aligned frame
  // 采用预测位置作为初始位置和滤波后的点云进行相关匹配获得的位置
  std::unique_ptr<transform::Rigid2d> pose_estimate_2d =
                                            ScanMatch(time, 
                                                      pose_prediction, 
                                                      filtered_gravity_aligned_point_cloud);
  if (pose_estimate_2d == nullptr) 
  {
    LOG(WARNING) << "Scan matching failed.";
    return nullptr;
  }
  //Rigid2d转换成 Rigid3d
  const transform::Rigid3d pose_estimate =
                                 transform::Embed3D(*pose_estimate_2d) * gravity_alignment;
  // 将此刻匹配后的准确位置加入估计值， 即更新估计器   
  extrapolator_->AddPose(time, pose_estimate);
  // 将点云转换至当前估计位置坐标下
  sensor::RangeData range_data_in_local =
      TransformRangeData(gravity_aligned_range_data,
                         transform::Embed3D(pose_estimate_2d->cast<float>()));
  //点云插入 更新子图 并读取结果
  std::unique_ptr<InsertionResult> insertion_result = InsertIntoSubmap(
                                                        time, 
                                                        range_data_in_local, 
                                                        filtered_gravity_aligned_point_cloud,
                                                        pose_estimate, 
                                                        gravity_alignment.rotation());

  const auto wall_time = std::chrono::steady_clock::now();
  if (last_wall_time_.has_value()) 
  {
    //获取当前时间
    const auto wall_time_duration = wall_time - last_wall_time_.value();
    //先转成秒,后设置成延迟度量
    kLocalSlamLatencyMetric->Set(common::ToSeconds(wall_time_duration));
    //如果有时间间隔值,计算传感器间隔与容器时间间隔比值
    if (sensor_duration.has_value()) 
    {
      kLocalSlamRealTimeRatio->Set(common::ToSeconds(sensor_duration.value()) /
                                   common::ToSeconds(wall_time_duration));
    }
  }
  const double thread_cpu_time_seconds = common::GetThreadCpuTimeSeconds();
  if (last_thread_cpu_time_seconds_.has_value()) 
  {
    const double thread_cpu_duration_seconds =
        thread_cpu_time_seconds - last_thread_cpu_time_seconds_.value();
    if (sensor_duration.has_value()) 
    {
      kLocalSlamCpuRealTimeRatio->Set(         //计算slam匹配周期是cpu频率时间的多少倍
          common::ToSeconds(sensor_duration.value()) /
          thread_cpu_duration_seconds);         
    }
  }
  last_wall_time_ = wall_time;
  last_thread_cpu_time_seconds_ = thread_cpu_time_seconds;
  //返回匹配结果
  return absl::make_unique<MatchingResult>(
        MatchingResult{   time,
                          pose_estimate, 
                          std::move(range_data_in_local),
                          std::move(insertion_result)});
}
// -----------------------------------------------------------
//将点云数据根据点云origin位置插入到submap中
/*input:
1.时间戳
2.转换至世界坐标系的点云
3.滤波后的原始点云
4.当前世界坐标
5.重力加速度旋转向量
 */
std::unique_ptr<LocalTrajectoryBuilder2D::InsertionResult>
LocalTrajectoryBuilder2D::InsertIntoSubmap(
    const common::Time time, 
    const sensor::RangeData& range_data_in_local,
    const sensor::PointCloud& filtered_gravity_aligned_point_cloud,
    const transform::Rigid3d& pose_estimate,
    const Eigen::Quaterniond& gravity_alignment) 
    {
  if (motion_filter_.IsSimilar(time, pose_estimate)) 
  {
    return nullptr;
  }
  std::vector<std::shared_ptr<const Submap2D>> insertion_submaps =
      active_submaps_.InsertRangeData(range_data_in_local);
  return absl::make_unique<InsertionResult>(InsertionResult
  {
      std::make_shared<const TrajectoryNode::Data>(TrajectoryNode::Data
      {
          time,
          gravity_alignment,
          filtered_gravity_aligned_point_cloud,
          {},  // 'high_resolution_point_cloud' is only used in 3D.
          {},  // 'low_resolution_point_cloud' is only used in 3D.
          {},  // 'rotational_scan_matcher_histogram' is only used in 3D.
          pose_estimate
      }),
      std::move(insertion_submaps)});
}
// -----------------------------------------------------------------
//如果有IMU数据,就添加IMU时间初始化优化器
void LocalTrajectoryBuilder2D::AddImuData(const sensor::ImuData& imu_data) 
{
  CHECK(options_.use_imu_data()) << "An unexpected IMU packet was added.";
  InitializeExtrapolator(imu_data.time);
  extrapolator_->AddImuData(imu_data);
}
//插值器加入里程计数
void LocalTrajectoryBuilder2D::AddOdometryData(
    const sensor::OdometryData& odometry_data) 
{
  if (extrapolator_ == nullptr) 
  {
    // Until we've initialized the extrapolator we cannot add odometry data.
    LOG(INFO) << "Extrapolator not yet initialized.";
    return;
  }
  extrapolator_->AddOdometryData(odometry_data);
}
//初始化插值器 
void LocalTrajectoryBuilder2D::InitializeExtrapolator(const common::Time time) 
{
  //如果插值器不是空的 就停止退出
  if (extrapolator_ != nullptr) 
  {
    return;
  }
  CHECK(!options_.pose_extrapolator_options().use_imu_based());
  // TODO(gaschler): Consider using InitializeWithImu as 3D does.
  //如果插值器为空的 就进行定义插值器 时间间隔 IMU重力时间常数 
  extrapolator_ = absl::make_unique<PoseExtrapolator>(
      ::cartographer::common::FromSeconds(options_.pose_extrapolator_options()
                                                    .constant_velocity()
                                                       .pose_queue_duration()),
      options_.pose_extrapolator_options()
                .constant_velocity()
                   .imu_gravity_time_constant());
  //并且 定义初始位姿的时间和位姿
  extrapolator_->AddPose(time, transform::Rigid3d::Identity());
}

void LocalTrajectoryBuilder2D::RegisterMetrics(
    metrics::FamilyFactory* family_factory) 
{
  auto* latency = family_factory->NewGaugeFamily(
                                  "mapping_2d_local_trajectory_builder_latency",
                                  "Duration from first incoming point cloud in accumulation to local slam "
                                  "result");
  kLocalSlamLatencyMetric = latency->Add({});
  auto* real_time_ratio = family_factory->NewGaugeFamily(
                              "mapping_2d_local_trajectory_builder_real_time_ratio",
                              "sensor duration / wall clock duration.");
  kLocalSlamRealTimeRatio = real_time_ratio->Add({});

  auto* cpu_real_time_ratio = family_factory->NewGaugeFamily(
                                                "mapping_2d_local_trajectory_builder_cpu_real_time_ratio",
                                                "sensor duration / cpu duration.");
  kLocalSlamCpuRealTimeRatio = cpu_real_time_ratio->Add({});
  auto score_boundaries = metrics::Histogram::FixedWidth(0.05, 20);
  auto* scores = family_factory->NewHistogramFamily(
                                          "mapping_2d_local_trajectory_builder_scores",
                                          "Local scan matcher scores",
                                          score_boundaries);
  kRealTimeCorrelativeScanMatcherScoreMetric =
                                      scores->Add({{"scan_matcher", 
                                                    "real_time_correlative"}});
  auto cost_boundaries = metrics::Histogram::ScaledPowersOf(2, 0.01, 100);
  auto* costs = family_factory->NewHistogramFamily(
                                        "mapping_2d_local_trajectory_builder_costs", 
                                        "Local scan matcher costs",
                                        cost_boundaries);
  kCeresScanMatcherCostMetric = costs->Add({{"scan_matcher", 
                                             "ceres"}});
  auto distance_boundaries = metrics::Histogram::ScaledPowersOf(2, 0.01, 10);
  auto* residuals = family_factory->NewHistogramFamily(
                                          "mapping_2d_local_trajectory_builder_residuals",
                                          "Local scan matcher residuals", 
                                          distance_boundaries);
  kScanMatcherResidualDistanceMetric =
      residuals->Add({{"component", 
                       "distance"}});
  kScanMatcherResidualAngleMetric = residuals->Add({{"component", 
                                                    "angle"}});
}

}  // namespace mapping
}  // namespace cartographer
