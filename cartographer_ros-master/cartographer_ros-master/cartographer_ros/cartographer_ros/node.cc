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

#include "cartographer_ros/node.h"

#include <chrono>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "absl/memory/memory.h"
#include "absl/strings/str_cat.h"
#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/pose_graph_interface.h"
#include "cartographer/mapping/proto/submap_visualization.pb.h"
#include "cartographer/metrics/register.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "cartographer_ros/metrics/family_factory.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/sensor_bridge.h"
#include "cartographer_ros/tf_bridge.h"
#include "cartographer_ros/time_conversion.h"
#include "cartographer_ros_msgs/StatusCode.h"
#include "cartographer_ros_msgs/StatusResponse.h"
#include "geometry_msgs/PoseStamped.h"
#include "glog/logging.h"
#include "nav_msgs/Odometry.h"
#include "ros/serialization.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf2_eigen/tf2_eigen.h"
#include "visualization_msgs/MarkerArray.h"

namespace cartographer_ros {

namespace carto = ::cartographer;

using carto::transform::Rigid3d;
using TrajectoryState = ::cartographer::mapping::PoseGraphInterface::TrajectoryState;

namespace {
// Subscribes to the 'topic' for 'trajectory_id' using the 'node_handle' and
// calls 'handler' on the 'node' to handle messages. Returns the subscriber.

// ------------------------- SubscribeWithHandler------------------------------------
// 订阅话题函数
template <typename MessageType> ::ros::Subscriber SubscribeWithHandler(
            void (Node::*handler)(int, const std::string&,
                                  const typename MessageType::ConstPtr&),
            const int trajectory_id,
            const std::string& topic,
            ::ros::NodeHandle* const node_handle,
            Node* const node)//主要是订阅话题
 {         //返回不同ros句柄订阅的信息
           return node_handle->subscribe<MessageType>(
                  topic, kInfiniteSubscriberQueueSize,
                  boost::function<void(const typename MessageType::ConstPtr&)>(
                    [node, handler, trajectory_id, topic](const typename MessageType::ConstPtr& msg) {
                      (node->*handler)(trajectory_id, topic, msg); }
         ));
 }
// ------------------------- SubscribeWithHandler------------------------------------

// -----------------------------------TrajectoryStateToString--------------------------
// 轨迹不同状态字符设定
std::string TrajectoryStateToString(const TrajectoryState trajectory_state) {//轨迹状态字符
  switch (trajectory_state) {
    case TrajectoryState::ACTIVE:
      return "ACTIVE";
    case TrajectoryState::FINISHED:
      return "FINISHED";
    case TrajectoryState::FROZEN:
      return "FROZEN";
    case TrajectoryState::DELETED:
      return "DELETED";
  }
  return "";
}
// -----------------------------------TrajectoryStateToString--------------------------
}  // namespace

// 构造函数体
Node::Node(
        const NodeOptions& node_options,                                           //参数发布周期 是否发布 等等
        std::unique_ptr<cartographer::mapping::MapBuilderInterface> map_builder,   //虚函数 类对象 实例化
        tf2_ros::Buffer* const tf_buffer,                                          //监听器存储空间
        const bool collect_metrics)
    : node_options_(node_options),                                                 //初始化变量node_options_
      map_builder_bridge_(node_options_,                                           //初始化变量 map_builder_bridge_
                          std::move(map_builder), 
                          tf_buffer) 
{ 
  absl::MutexLock lock(&mutex_);//创建互斥锁
  if (collect_metrics) {                                                           // 收集指标  FLAGS_collect_metrics
    metrics_registry_ = absl::make_unique<metrics::FamilyFactory>();
    carto::metrics::RegisterAllMetrics(metrics_registry_.get());                    //注册指标
  }
// 定义发布话题
  submap_list_publisher_ =
      node_handle_.advertise<::cartographer_ros_msgs::SubmapList>(//发布kSubmapListTopic话题，子图
          kSubmapListTopic, kLatestOnlyPublisherQueueSize);//数据类型为 ::cartographer_ros_msgs::SubmapList ，
                                                           //内存大小为 kLatestOnlyPublisherQueueSize
  trajectory_node_list_publisher_ =
      node_handle_.advertise<::visualization_msgs::MarkerArray>(//发布kTrajectoryNodeListTopic话题，轨迹列表
          kTrajectoryNodeListTopic, kLatestOnlyPublisherQueueSize);
  landmark_poses_list_publisher_ =
      node_handle_.advertise<::visualization_msgs::MarkerArray>(//发布kLandmarkPosesListTopic话题，路标
          kLandmarkPosesListTopic, kLatestOnlyPublisherQueueSize);
  constraint_list_publisher_ =
      node_handle_.advertise<::visualization_msgs::MarkerArray>(//发布 kConstraintListTopic话题，约束
          kConstraintListTopic, kLatestOnlyPublisherQueueSize);
  if (node_options_.publish_tracked_pose) {
    tracked_pose_publisher_ =
        node_handle_.advertise<::geometry_msgs::PoseStamped>(//发布kTrackedPoseTopic话题，位姿
            kTrackedPoseTopic, kLatestOnlyPublisherQueueSize);
  }
  // 定义服务器
  service_servers_.push_back(node_handle_.advertiseService(//注册服务器，名字为 kSubmapQueryServiceName 变量内容      
                                                           //查询Submap
      kSubmapQueryServiceName, &Node::HandleSubmapQuery, this)); //第二个 参数为句柄，有请求，该函数会回应
  service_servers_.push_back(node_handle_.advertiseService(
      kTrajectoryQueryServiceName, &Node::HandleTrajectoryQuery, this));  //Trajectory查询
  service_servers_.push_back(node_handle_.advertiseService(
      kStartTrajectoryServiceName, &Node::HandleStartTrajectory, this));   //开始一段Trajectory
  service_servers_.push_back(node_handle_.advertiseService(
      kFinishTrajectoryServiceName, &Node::HandleFinishTrajectory, this));   //结束一段Trajectory
  service_servers_.push_back(node_handle_.advertiseService(
      kWriteStateServiceName, &Node::HandleWriteState, this));              //写状态
  service_servers_.push_back(node_handle_.advertiseService(
      kGetTrajectoryStatesServiceName, &Node::HandleGetTrajectoryStates, this));    //获取轨迹状态
  service_servers_.push_back(node_handle_.advertiseService(
      kReadMetricsServiceName, &Node::HandleReadMetrics, this));                    //读取服务器名字

  scan_matched_point_cloud_publisher_ =                 //发布点云扫描匹配的话题
      node_handle_.advertise<sensor_msgs::PointCloud2>(
          kScanMatchedPointCloudTopic, kLatestOnlyPublisherQueueSize);
//ros::WallTime::now()为当前的真实时间，也就是墙上的挂钟时间，一直在走。
//ros::Time::now()为rosbag当时的时间，是由bag中/clock获取的。是仿真时间。
//wall_timers_  : vecter 容器  给参数设定定时器
  wall_timers_.push_back(node_handle_.createWallTimer(
      ::ros::WallDuration(node_options_.submap_publish_period_sec),
      &Node::PublishSubmapList, this));                     //子图
  if (node_options_.pose_publish_period_sec > 0) {
    publish_local_trajectory_data_timer_ = node_handle_.createTimer(
        ::ros::Duration(node_options_.pose_publish_period_sec),
        &Node::PublishLocalTrajectoryData, this);           //局部轨迹
  }
  wall_timers_.push_back(node_handle_.createWallTimer(
      ::ros::WallDuration(node_options_.trajectory_publish_period_sec),
      &Node::PublishTrajectoryNodeList, this));                          //轨迹节点
  wall_timers_.push_back(node_handle_.createWallTimer(
      ::ros::WallDuration(node_options_.trajectory_publish_period_sec),
      &Node::PublishLandmarkPosesList, this));                            //路标
  wall_timers_.push_back(node_handle_.createWallTimer(
      ::ros::WallDuration(kConstraintPublishPeriodSec),
      &Node::PublishConstraintList, this));                 
}

Node::~Node() { FinishAllTrajectories(); }                                //主函数析构函数

::ros::NodeHandle* Node::node_handle() { return &node_handle_; }


//节点句柄都是最终调用map_builder_bridge_内的查询句柄函数
//map_builder_bridge_调用MapBuilderBridge   ,
//MapBuilderBridged调用cartographer::mapping::MapBuilderInterface
//MapBuilderInterface map_builder.h 实例化
bool Node::HandleSubmapQuery(
    ::cartographer_ros_msgs::SubmapQuery::Request& request,
    ::cartographer_ros_msgs::SubmapQuery::Response& response) {
  absl::MutexLock lock(&mutex_);
  map_builder_bridge_.HandleSubmapQuery(request, response);               //MapBuilderBridge的一个变量类型  子图访问
  return true;
}
// ---------------------------------HandleTrajectoryQuery---------------------------------------

// ---------------------------------HandleTrajectoryQuery---------------------------------------
// 轨迹队列访问
// 由Node 进入到 map_builder_bridge_.HandleTrajectoryQuery 
bool Node::HandleTrajectoryQuery(
    ::cartographer_ros_msgs::TrajectoryQuery::Request& request,
    ::cartographer_ros_msgs::TrajectoryQuery::Response& response) {
  absl::MutexLock lock(&mutex_);
  response.status = TrajectoryStateToStatus(                       // TrajectoryState
                                            request.trajectory_id,     
                                            { TrajectoryState::ACTIVE, 
                                              TrajectoryState::FINISHED,
                                              TrajectoryState::FROZEN }  /* valid states */);
  if (response.status.code != cartographer_ros_msgs::StatusCode::OK) {
    LOG(ERROR) << "Can't query trajectory from pose graph: "
               << response.status.message;
    return true;
  }
  map_builder_bridge_.HandleTrajectoryQuery(request, response);   //MapBuilderBridge的一个变量类型  轨迹访问
  return true;
}
// ---------------------------------HandleTrajectoryQuery---------------------------------------

// ----------------------------------PublishSubmapList----------------------------------------
// 发布子图队列 进入到 map_builder_bridge_ 获取子图队列
void Node::PublishSubmapList(const ::ros::WallTimerEvent& unused_timer_event) {
  absl::MutexLock lock(&mutex_);
  submap_list_publisher_.publish(map_builder_bridge_.GetSubmapList());
}
// ----------------------------------PublishSubmapList----------------------------------------

// ----------------------------------AddExtrapolator----------------------------------------
// 创建插值器 并配置插值器的一些参数
void Node::AddExtrapolator(const int trajectory_id,             
                           const TrajectoryOptions& options) {  //配置参数
                                                      //   struct TrajectoryOptions {//定义轨迹结构
                                                      //   ::cartographer::mapping::proto::TrajectoryBuilderOptions
                                                      //       trajectory_builder_options;
                                                      //   std::string tracking_frame;
                                                      //   std::string published_frame;
                                                      //   std::string odom_frame;
                                                      //   bool provide_odom_frame;
                                                      //   bool use_odometry;
                                                      //   bool use_nav_sat;
                                                      //   bool use_landmarks;
                                                      //   bool publish_frame_projected_to_2d;
                                                      //   int num_laser_scans;
                                                      //   int num_multi_echo_laser_scans;
                                                      //   int num_subdivisions_per_laser_scan;
                                                      //   int num_point_clouds;
                                                      //   double rangefinder_sampling_ratio;
                                                      //   double odometry_sampling_ratio;
                                                      //   double fixed_frame_pose_sampling_ratio;
                                                      //   double imu_sampling_ratio;
                                                      //   double landmarks_sampling_ratio;
                                                      // };
  constexpr double kExtrapolationEstimationTimeSec = 0.001;  // 1 ms
  CHECK(extrapolators_.count(trajectory_id) == 0);
  const double gravity_time_constant =                                            //判别是否用3d或者2d轨迹参数 设置常数
      node_options_.map_builder_options.use_trajectory_builder_3d()  
          ? options.trajectory_builder_options.trajectory_builder_3d_options().imu_gravity_time_constant()
          : options.trajectory_builder_options.trajectory_builder_2d_options().imu_gravity_time_constant();
// 有待深入了解
  extrapolators_.emplace(                                                         //插值器插入信息                          
                std::piecewise_construct,         
                std::forward_as_tuple(trajectory_id),                             // 返回值
                                                                                  // 如同以 std::tuple<Types&&...>(std::forward<Types>(args)...) 
                                                                                  // 创建的 std::tuple 对象。
                std::forward_as_tuple( ::cartographer::common::FromSeconds(kExtrapolationEstimationTimeSec),
                                       gravity_time_constant));
}
// ----------------------------------AddExtrapolator----------------------------------------

// ----------------------------------AddSensorSamplers----------------------------------------
// 添加传感器采样器 设置采样器频率参数、轨迹id 
void Node::AddSensorSamplers(const int trajectory_id,
                             const TrajectoryOptions& options) 
{
  CHECK(sensor_samplers_.count(trajectory_id) == 0);
  sensor_samplers_.emplace(
      std::piecewise_construct, 
      std::forward_as_tuple(trajectory_id),
      std::forward_as_tuple(                                                    //构建采样器的频率 结构体
          options.rangefinder_sampling_ratio,
          options.odometry_sampling_ratio,
          options.fixed_frame_pose_sampling_ratio, 
          options.imu_sampling_ratio,
          options.landmarks_sampling_ratio));
}
// ----------------------------------AddSensorSamplers----------------------------------------

// ----------------------------------PublishLocalTrajectoryData----------------------------------------
void Node::PublishLocalTrajectoryData(const ::ros::TimerEvent& timer_event) 
{
  absl::MutexLock lock(&mutex_);                                                 //加锁
  for (const auto& entry : map_builder_bridge_.GetLocalTrajectoryData())         //循环读取轨迹容器参数
  {
    const auto& trajectory_data = entry.second;                                  //读取轨迹local_trajectory_data数据，
                                                                                 //first是map容器一个int型序号
    auto& extrapolator = extrapolators_.at(entry.first);                         //通过序号读取插值器？？？？？？？？？？？                 
    // We only publish a point cloud if it has changed. It is not needed at high
    // frequency, and republishing it would be computationally wasteful.
    if (trajectory_data.local_slam_data->time != extrapolator.GetLastPoseTime()) 
    {
      if (scan_matched_point_cloud_publisher_.getNumSubscribers() > 0) {           //判断 是否接收到扫描匹配点云发布器信息
        // TODO(gaschler): Consider using other message without time
        // information.
        carto::sensor::TimedPointCloud point_cloud;                                //具有时间信息的点云容器
        point_cloud.reserve(trajectory_data.local_slam_data->range_data_in_local.returns.size());
                                                                                   // reserve() 为点云容器预留足够的空间
        for (const cartographer::sensor::RangefinderPoint point :
             trajectory_data.local_slam_data->range_data_in_local.returns)         //读取障碍物位置信息，相当有效点云信息
        //                                                                     struct RangeData {
        //                                                                           Eigen::Vector3f origin;   //{x0,y0,z0},sensor坐标。
        //                                                                           PointCloud returns;       //反射位置{x,y,z}，表征有物体反射。
        //                                                                           PointCloud misses;        //无反射,自由空间
        {
          point_cloud.push_back(cartographer::sensor::ToTimedRangefinderPoint(
                                                                    point, 
                                                                    0.f /* time */));
        }
        //发布 时间 位姿话题
        scan_matched_point_cloud_publisher_.publish(ToPointCloud2Message(
            carto::common::ToUniversal(trajectory_data.local_slam_data->time),      //全局时间？？？？？
            node_options_.map_frame,                                                //frame
            carto::sensor::TransformTimedPointCloud( point_cloud,                   //转换局部点云位姿到全局地图位姿
                                                     trajectory_data.local_to_map.cast<float>())));                                                                         
      }
      extrapolator.AddPose(trajectory_data.local_slam_data->time,                    //给插值器添加具有时间位姿
                           trajectory_data.local_slam_data->local_pose);
    }
    geometry_msgs::TransformStamped stamped_transform;//定义转换矩阵
                                                    // Header header
                                                    // string child_frame_id  
                                                    // Transform transform

    // If we do not publish a new point cloud, we still allow time of the
    // published poses to advance. If we already know a newer pose, we use its
    // time instead. Since tf knows how to interpolate, providing newer
    // information is better.
    const ::cartographer::common::Time now = std::max(                               //选取最新时间
                                            FromRos(ros::Time::now()),               //ROS当前时间
                                            extrapolator.GetLastExtrapolatedTime()); //插值器最新时间
    stamped_transform.header.stamp =                                                 //判断是否用插值器，是就用当前ROS的时间 ，
                                                                                     //否则用轨迹SLAM系统时间
        node_options_.use_pose_extrapolator? ToRos(now): ToRos(trajectory_data.local_slam_data->time);

    // Suppress publishing if we already published a transform at this time.
    // Due to 2020-07 changes to geometry2, tf buffer will issue warnings for
    // repeated transforms with the same timestamp.
    if (last_published_tf_stamps_.count(entry.first) &&
        last_published_tf_stamps_[entry.first] == stamped_transform.header.stamp)
      continue;
    last_published_tf_stamps_[entry.first] = stamped_transform.header.stamp;      //这里有点奇怪 ，上面已经判断tf时间戳 等于 转换时间戳 
                                                                                  //又进行了一次赋值？？？？？？？？？？？？？？？？？

    const Rigid3d tracking_to_local_3d =                                           //如果用位姿插值器，
        node_options_.use_pose_extrapolator ? extrapolator.ExtrapolatePose(now)    //就用当前时间的差值位姿
        : trajectory_data.local_slam_data->local_pose;                             //否则使用轨迹，局部SLAM位姿
    const Rigid3d tracking_to_local = [&]{                                         //返回3d或则2d位姿
      if (trajectory_data.trajectory_options.publish_frame_projected_to_2d)        //判断是否使用2d位姿
      {
        return carto::transform::Embed3D(
            carto::transform::Project2D(tracking_to_local_3d));                    //将3d转为2d位姿          
      }
      return tracking_to_local_3d;
    }();

    const Rigid3d tracking_to_map =                                                 //将本地位姿转换到世界map坐标系
        trajectory_data.local_to_map * tracking_to_local;

    if (trajectory_data.published_to_tracking != nullptr) {                         //发布到跟踪器？？？？？
      if (node_options_.publish_to_tf) {                                            //发布到tf坐标系
        if (trajectory_data.trajectory_options.provide_odom_frame) { 
          std::vector<geometry_msgs::TransformStamped> stamped_transforms;          //创建转换矩阵信息
          //存储里程计信息到map坐标变换信息
          stamped_transform.header.frame_id = node_options_.map_frame;              //父节点为map
          stamped_transform.child_frame_id = trajectory_data.trajectory_options.odom_frame;
                                                                                    //子节点为里程计信息
          stamped_transform.transform =
              ToGeometryMsgTransform(trajectory_data.local_to_map);                  //局部到全局坐标变换 转化为 信息类型
          stamped_transforms.push_back(stamped_transform);                           //存储该转换矩阵
          // -----------------------------------------------------------------
          //又重新存储里程计跟踪器信息？？？？？？？？？？？？？？？？
          stamped_transform.header.frame_id =                                       //父节点为odom           
              trajectory_data.trajectory_options.odom_frame;
          stamped_transform.child_frame_id =                                         //子节点为里程计信息
              trajectory_data.trajectory_options.published_frame;                    //发布odom作为frame的坐标tf变换
          stamped_transform.transform = ToGeometryMsgTransform(
              tracking_to_local * (*trajectory_data.published_to_tracking));
          stamped_transforms.push_back(stamped_transform);
          tf_broadcaster_.sendTransform(stamped_transforms);
          // -----------------------------------------------------------------------------------------------

        } else {                                                                     //如果不提供odom信息，就直接发布
          stamped_transform.header.frame_id = node_options_.map_frame;               //发布published_frame到map_frame 坐标tf变换
          stamped_transform.child_frame_id =
              trajectory_data.trajectory_options.published_frame;
          stamped_transform.transform = ToGeometryMsgTransform(
              tracking_to_map * (*trajectory_data.published_to_tracking));
          tf_broadcaster_.sendTransform(stamped_transform);
        }
      }
      if (node_options_.publish_tracked_pose) {                                       //判断是否发布跟踪位姿
        ::geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.frame_id = node_options_.map_frame;
        pose_msg.header.stamp = stamped_transform.header.stamp;
        pose_msg.pose = ToGeometryMsgPose(tracking_to_map);                           //转化为消息
        tracked_pose_publisher_.publish(pose_msg);                                    //发布位姿
      }
    }
  }
}
// -----------------------------------------------------------------------------------

// -------------------------------------PublishTrajectoryNodeList----------------------------------------------
// 发布轨迹列表  通过 map_builder_bridge_ 获取
void Node::PublishTrajectoryNodeList(
    const ::ros::WallTimerEvent& unused_timer_event) {
  if (trajectory_node_list_publisher_.getNumSubscribers() > 0) {
    absl::MutexLock lock(&mutex_);
    trajectory_node_list_publisher_.publish(
        map_builder_bridge_.GetTrajectoryNodeList());
  }
}
// -------------------------------------PublishTrajectoryNodeList----------------------------------------------

// -------------------------------------PublishLandmarkPosesList----------------------------------------------
// 发布路标列表  通过 map_builder_bridge_ 获取
void Node::PublishLandmarkPosesList(
    const ::ros::WallTimerEvent& unused_timer_event) {
  if (landmark_poses_list_publisher_.getNumSubscribers() > 0) {
    absl::MutexLock lock(&mutex_);
    landmark_poses_list_publisher_.publish(
        map_builder_bridge_.GetLandmarkPosesList());
  }
}
// -------------------------------------PublishLandmarkPosesList----------------------------------------------


// -------------------------------------PublishConstraintList----------------------------------------------
// 发布约束列表  通过 map_builder_bridge_ 获取
void Node::PublishConstraintList(
    const ::ros::WallTimerEvent& unused_timer_event) {
  if (constraint_list_publisher_.getNumSubscribers() > 0) {
    absl::MutexLock lock(&mutex_);
    constraint_list_publisher_.publish(map_builder_bridge_.GetConstraintList());
  }
}
// -------------------------------------PublishConstraintList----------------------------------------------

// -------------------------------------ComputeExpectedSensorIds----------------------------------------------
// 计算传感器类别id
              // struct SensorId {
              //   enum class SensorType {                        枚举类
              //     RANGE = 0,
              //     IMU,                                         1
              //     ODOMETRY,                                    2
              //     FIXED_FRAME_POSE,                            3
              //     LANDMARK,                                    4
              //     LOCAL_SLAM_RESULT                            5
              //   };
std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>
Node::ComputeExpectedSensorIds(const TrajectoryOptions& options) const 
{
  using SensorId = cartographer::mapping::TrajectoryBuilderInterface::SensorId;
  using SensorType = SensorId::SensorType;
  std::set<SensorId> expected_topics;
  // Subscribe to all laser scan, multi echo laser scan, and point cloud topics.
  for (const std::string& topic :                                                //如果传感器数据为激光 插入激光id号“0”
       ComputeRepeatedTopicNames( kLaserScanTopic, 
                                  options.num_laser_scans)) 
  {
    expected_topics.insert(SensorId{SensorType::RANGE, topic});                      
  }

  for (const std::string& topic :                                                //如果传感器数据为其他激光 插入激光id号“0”
       ComputeRepeatedTopicNames( kMultiEchoLaserScanTopic,
                                  options.num_multi_echo_laser_scans)) 
  {
    expected_topics.insert(SensorId{SensorType::RANGE, topic});
  }

  for (const std::string& topic :                                               //如果传感器数据为点云 插入激光id号“0”
       ComputeRepeatedTopicNames( kPointCloud2Topic, 
                                  options.num_point_clouds))
  {
    expected_topics.insert(SensorId{SensorType::RANGE, topic});
  }
  // For 2D SLAM, subscribe to the IMU if we expect it. For 3D SLAM, the IMU is
  // required.
  if (node_options_.map_builder_options.use_trajectory_builder_3d() ||          //在2d或者3d激光雷达信息下，如果传感器数据IMU 插入IMU id号“1”
      (node_options_.map_builder_options.use_trajectory_builder_2d() &&
       options.trajectory_builder_options.trajectory_builder_2d_options().use_imu_data()))
 {
    expected_topics.insert(SensorId{SensorType::IMU, kImuTopic});
  }
  // Odometry is optional.
  if (options.use_odometry) {                                                  //如果传感器数据为里程计信息 插入激光id号“2”
    expected_topics.insert(SensorId{SensorType::ODOMETRY, kOdometryTopic});
  }
  // NavSatFix is optional.
  if (options.use_nav_sat) {
    expected_topics.insert(                                                    //如果传感器数据为GPS信息 插入激光id号“3”
        SensorId{SensorType::FIXED_FRAME_POSE, kNavSatFixTopic});
  }
  // Landmark is optional.
  if (options.use_landmarks) {                                                 //如果传感器数据为路标信息 插入激光id号“4”
    expected_topics.insert(SensorId{SensorType::LANDMARK, kLandmarkTopic});
  }
  return expected_topics;                                                      //返回所有传感器信息 和 对的id
}
// ---------------------------------------------ComputeExpectedSensorIds------------------------------------------

// ----------------------------------------:AddTrajectory-----------------------------------------------
//给对应id轨迹         创建插值器、传感器容器、map_builder_,设置订阅其他传感器话题信息
// 并返回id
int Node::AddTrajectory(const TrajectoryOptions& options) {
  const std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId> //设置传感器信息对应id
                            expected_sensor_ids = ComputeExpectedSensorIds(options);  
  const int trajectory_id =
      map_builder_bridge_.AddTrajectory(expected_sensor_ids, options);         //又进入map_builder_ 轨迹
  AddExtrapolator(trajectory_id, options);                                     //添加到插值器
  AddSensorSamplers(trajectory_id, options);                                   //添加到传感器容器sensor_samplers_
  //重点关注这个函数，可以额外订阅其他传感器信息？？？？？？
  LaunchSubscribers(options, trajectory_id);                                   //调用node_handle->subscribe订阅话题信息
  wall_timers_.push_back( node_handle_.createWallTimer(::ros::WallDuration(kTopicMismatchCheckDelaySec),
                                                       &Node::MaybeWarnAboutTopicMismatch, 
                                                       this,
                                                      /*oneshot=*/true));      //给Mismatch不匹配设定定时器
  for (const auto& sensor_id : expected_sensor_ids) {                          //订阅话题存储数据id
       subscribed_topics_.insert(sensor_id.id);
  }
  return trajectory_id;
}
// ----------------------------------------:AddTrajectory-----------------------------------------------

// ----------------------------------------LaunchSubscribers-----------------------------------------------
  //设置订阅话题信息，并存放到subscribers_容器 option类型不同存储轨迹信息不同
void Node::LaunchSubscribers(const TrajectoryOptions& options,
                             const int trajectory_id) {
  for (const std::string& topic : ComputeRepeatedTopicNames( kLaserScanTopic, 
                                                             options.num_laser_scans)) 
  {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::LaserScan>(                          //订阅激光雷达
             &Node::HandleLaserScanMessage, trajectory_id, topic, &node_handle_,//在HandleLaserScanMessage中处理数据
             this),                                     
         topic});
  }

  for (const std::string& topic : ComputeRepeatedTopicNames(                    //订阅其他激光雷达
           kMultiEchoLaserScanTopic,                                       
           options.num_multi_echo_laser_scans)) 
  {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::MultiEchoLaserScan>(
             &Node::HandleMultiEchoLaserScanMessage, trajectory_id, topic,       //在HandleMultiEchoLaserScanMessage函数处理
             &node_handle_, this),
         topic});
  }
  for (const std::string& topic :                                                 //订阅点云话题
       ComputeRepeatedTopicNames(kPointCloud2Topic, options.num_point_clouds)) {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::PointCloud2>(
             &Node::HandlePointCloud2Message, trajectory_id, topic,               //在HandlePointCloud2Message函数处理
             &node_handle_, this),
         topic});
  }

  // For 2D SLAM, subscribe to the IMU if we expect it. For 3D SLAM, the IMU is
  // required.

// 给轨迹对应不同options.的传感器信息 存储含有不同数据的轨迹
  if (node_options_.map_builder_options.use_trajectory_builder_3d() ||            // 使用3d或者2d
      (node_options_.map_builder_options.use_trajectory_builder_2d() &&
       options.trajectory_builder_options.trajectory_builder_2d_options().use_imu_data()))       
                                                                                  //且使用IMU数据
  {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::Imu>(  &Node::HandleImuMessage,
                                                  trajectory_id, 
                                                  kImuTopic,
                                                  &node_handle_, 
                                                  this),
         kImuTopic});
  }

  if (options.use_odometry) {                                                     //使用了里程计信息
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<nav_msgs::Odometry>(&Node::HandleOdometryMessage,
                                                  trajectory_id,
                                                  kOdometryTopic,
                                                  &node_handle_, 
                                                  this),
         kOdometryTopic});
  }
  if (options.use_nav_sat) {                                                      //使用了GPS信息
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::NavSatFix>(
                                                  &Node::HandleNavSatFixMessage, 
                                                  trajectory_id, 
                                                  kNavSatFixTopic,
                                                  &node_handle_, 
                                                  this),
         kNavSatFixTopic});
  }
  if (options.use_landmarks) {                                                    //使用了路标信息
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<cartographer_ros_msgs::LandmarkList>(
                                                  &Node::HandleLandmarkMessage, 
                                                  trajectory_id, 
                                                  kLandmarkTopic,
                                                  &node_handle_, 
                                                  this),
         kLandmarkTopic});
  }
}
// ----------------------------------------LaunchSubscribers-----------------------------------------------

// ----------------------------------------ValidateTrajectoryOptions-----------------------------------------------
//判断选择2d或者3d轨迹构建器
bool Node::ValidateTrajectoryOptions(const TrajectoryOptions& options) {
  if (node_options_.map_builder_options.use_trajectory_builder_2d()) {
    return options.trajectory_builder_options.has_trajectory_builder_2d_options();
  }
  if (node_options_.map_builder_options.use_trajectory_builder_3d()) {
    return options.trajectory_builder_options.has_trajectory_builder_3d_options();
  }
  return false;
}
// ----------------------------------------ValidateTrajectoryOptions-----------------------------
// 判断话题是否已经被占用
bool Node::ValidateTopicNames(const TrajectoryOptions& options) {
  for (const auto& sensor_id : ComputeExpectedSensorIds(options)) {
    const std::string& topic = sensor_id.id;
    if (subscribed_topics_.count(topic) > 0) {
      LOG(ERROR) << "Topic name [" << topic << "] is already used.";
      return false;
    }
  }
  return true;
}
// ----------------------------------------ValidateTrajectoryOptions-----------------------------

// ----------------------------------------TrajectoryStateToStatus-----------------------------
// 通过map_builder_bridge_.GetTrajectoryStates()获取轨迹状态 并且 将查找对应轨迹id 与 当前轨迹序号比较 是否一致
// 返回轨迹状态
cartographer_ros_msgs::StatusResponse Node::TrajectoryStateToStatus(const int trajectory_id, 
                                                                    const std::set<TrajectoryState>& valid_states) 
{
  const auto trajectory_states = map_builder_bridge_.GetTrajectoryStates();
  cartographer_ros_msgs::StatusResponse status_response;
  const auto it = trajectory_states.find(trajectory_id);
  if (it == trajectory_states.end()) {
    status_response.message = absl::StrCat( "Trajectory ", 
                                            trajectory_id, 
                                            " doesn't exist.");
    status_response.code = cartographer_ros_msgs::StatusCode::NOT_FOUND;
    return status_response;
  }

  status_response.message = absl::StrCat(  "Trajectory ", 
                                            trajectory_id, 
                                            " is in '",
                                            TrajectoryStateToString(it->second), 
                                            "' state.");
  status_response.code = valid_states.count(it->second)
          ? cartographer_ros_msgs::StatusCode::OK : cartographer_ros_msgs::StatusCode::INVALID_ARGUMENT;
  return status_response;
}
// ----------------------------------------TrajectoryStateToStatus-----------------------------

// ----------------------------------------FinishTrajectoryUnderLock-----------------------------
// 判定是否完成轨迹生成，并返回状态
cartographer_ros_msgs::StatusResponse Node::FinishTrajectoryUnderLock(const int trajectory_id) 
{
  cartographer_ros_msgs::StatusResponse status_response;                       //定义轨迹状态            
  if (trajectories_scheduled_for_finish_.count(trajectory_id))                 //如果当前轨迹已经有完成的 就声明一下OK
  {
    status_response.message = absl::StrCat( "Trajectory ", 
                                            trajectory_id,
                                            " already pending to finish.");
    status_response.code = cartographer_ros_msgs::StatusCode::OK;              //并且返回状态OK
    LOG(INFO) << status_response.message;
    return status_response;
  }

  // First, check if we can actually finish the trajectory.
  status_response = TrajectoryStateToStatus( trajectory_id,                     //读取当前轨迹状态
                                             {TrajectoryState::ACTIVE} /* valid states */);
  //首先检测是否真的完成轨迹
  if (status_response.code != cartographer_ros_msgs::StatusCode::OK) {          //如果当前轨迹不OK 就声明一下 未完成轨迹                    
     LOG(ERROR) << "Can't finish trajectory: " << status_response.message;
     return status_response;
  }

  // Shutdown the subscribers of this trajectory.
  // A valid case with no subscribers is e.g. if we just visualize states.
  if (subscribers_.count(trajectory_id)) {
    for (auto& entry : subscribers_[trajectory_id]) {
      entry.subscriber.shutdown();
      subscribed_topics_.erase(entry.topic);//删除订阅话题某处信息
                                        //  （1）erase(pos,n); 删除从pos开始的n个字符，比如erase(0,1)就是删除第一个字符
                                        //  （2）erase(position);删除position处的一个字符(position是个string类型的迭代器)
                                        //  （3）erase(first,last);删除从first到last之间的字符（first和last都是迭代器）

      LOG(INFO) << "Shutdown the subscriber of [" << entry.topic << "]";
    }
    CHECK_EQ(subscribers_.erase(trajectory_id), 1);
  }
  map_builder_bridge_.FinishTrajectory(trajectory_id);
                                                                              //从这里进入轨迹生成处理  map_builder_bridge_
                                                                              // 再进入到map_builder_->FinishTrajectory
  trajectories_scheduled_for_finish_.emplace(trajectory_id);
  status_response.message =
      absl::StrCat("Finished trajectory ", trajectory_id, ".");
  status_response.code = cartographer_ros_msgs::StatusCode::OK;              //返回ok状态
  return status_response;                                                     
}
// ----------------------------------------FinishTrajectoryUnderLock-----------------------------

// ----------------------------------------HandleStartTrajectory-----------------------------
bool Node::HandleStartTrajectory(
    ::cartographer_ros_msgs::StartTrajectory::Request& request,
    ::cartographer_ros_msgs::StartTrajectory::Response& response) {
  TrajectoryOptions trajectory_options;
  std::tie(std::ignore, trajectory_options) = LoadOptions(
      request.configuration_directory, request.configuration_basename);      //加载参数设置 返回结构体
  ////////////////////////////////////////////////                          //下面很大部分是判断异常情况
  if (request.use_initial_pose) {
    const auto pose = ToRigid3d(request.initial_pose);
    if (!pose.IsValid()) {                                                  //位姿无效 声明无效 并返回true
      response.status.message = "Invalid pose argument. Orientation quaternion must be normalized.";
      LOG(ERROR) << response.status.message;
      response.status.code = cartographer_ros_msgs::StatusCode::INVALID_ARGUMENT;
      return true;
    }
    // Check if the requested trajectory for the relative initial pose exists.
    response.status = TrajectoryStateToStatus(                              //读取轨迹状态 如果 轨迹存在 并有以下三种状态 返回OK状态
                          request.relative_to_trajectory_id,
                          {  TrajectoryState::ACTIVE, 
                             TrajectoryState::FROZEN,
                             TrajectoryState::FINISHED  } /* valid states */);
                   
    if (response.status.code != cartographer_ros_msgs::StatusCode::OK) {    //状态不正常
      LOG(ERROR) << "Can't start a trajectory with initial pose: "
                 << response.status.message;
      return true;
    }
    ::cartographer::mapping::proto::InitialTrajectoryPose initial_trajectory_pose;
                                                                            //创建初始化位姿
    initial_trajectory_pose.set_to_trajectory_id( request.relative_to_trajectory_id);
                                                                            //初始化id
    *initial_trajectory_pose.mutable_relative_pose() = cartographer::transform::ToProto(pose);
                                                                             //初始化位姿
    initial_trajectory_pose.set_timestamp(cartographer::common::ToUniversal( //初始化时间戳
        ::cartographer_ros::FromRos(ros::Time(0))));
    *trajectory_options.trajectory_builder_options.mutable_initial_trajectory_pose() 
                                           = initial_trajectory_pose;        //赋值轨迹初始化位姿参数设置
  }

  if (!ValidateTrajectoryOptions(trajectory_options)) {                       //如果轨迹参数设置不是使用2d 或者3d建图方式 则返回状态无效
    response.status.message = "Invalid trajectory options.";
    LOG(ERROR) << response.status.message;
    response.status.code = cartographer_ros_msgs::StatusCode::INVALID_ARGUMENT;
  } else if (!ValidateTopicNames(trajectory_options)) {                       //如果轨迹参数设置使用重复话题信息 则返回状态无效
    response.status.message = "Topics are already used by another trajectory.";
    LOG(ERROR) << response.status.message;
    response.status.code = cartographer_ros_msgs::StatusCode::INVALID_ARGUMENT;
     ////////////////////////////////////////////////////////////////         //判断异常情况到这里
  } else {//确认完成
    response.status.message = "Success.";
    //主要函数在这里进入
    response.trajectory_id = AddTrajectory(trajectory_options);//又可以进入MapBuilderBridge::AddTrajectory
    response.status.code = cartographer_ros_msgs::StatusCode::OK;
  }
  return true;
}
//----------------------------------------StartTrajectoryWithDefaultTopics----------------------------------------------
// 利用默认方式配置轨迹 id 参数
void Node::StartTrajectoryWithDefaultTopics(const TrajectoryOptions& options) {
  absl::MutexLock lock(&mutex_);
  CHECK(ValidateTrajectoryOptions(options));
  AddTrajectory(options);                                                     //给对应id轨迹   创建插值器、传感器容器、map_builder_,
                                                                              //设置订阅其他传感器话题信息// 并返回id
}
//----------------------------------------StartTrajectoryWithDefaultTopics-----------------------------------

//----------------------------------------ComputeDefaultSensorIdsForMultipleBags-----------------------------------
// 对默认传感器 归类存储多个数据包
std::vector< std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>>
Node::ComputeDefaultSensorIdsForMultipleBags(const std::vector<TrajectoryOptions>& bags_options)const
{
  using SensorId = cartographer::mapping::TrajectoryBuilderInterface::SensorId;//赋值id号类别结构体
  std::vector<std::set<SensorId>> bags_sensor_ids;                             //创建数据包容器
  for (size_t i = 0; i < bags_options.size(); ++i) {
    std::string prefix;
    if (bags_options.size() > 1) {
      prefix = "bag_" + std::to_string(i + 1) + "_";                            //鉴定有多少个id数据包
    }
    std::set<SensorId> unique_sensor_ids;
    for (const auto& sensor_id : ComputeExpectedSensorIds(bags_options.at(i))) {
      unique_sensor_ids.insert(SensorId{sensor_id.type, prefix + sensor_id.id});
    }
    bags_sensor_ids.push_back(unique_sensor_ids);                                //存储归类的id数据
  }
  return bags_sensor_ids;
}
//----------------------------------------ComputeDefaultSensorIdsForMultipleBags-----------------------------------

//----------------------------------------AddOfflineTrajectory-----------------------------------
// 添加离线轨迹
int Node::AddOfflineTrajectory(
    const std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>& expected_sensor_ids,
    const TrajectoryOptions& options) 
{
  absl::MutexLock lock(&mutex_);                                                  //加锁
  const int trajectory_id =                                                        
               map_builder_bridge_.AddTrajectory(expected_sensor_ids, options);   //通过轨迹结构体和sensorId获取轨迹Id
  AddExtrapolator(trajectory_id, options);                                        //将轨迹添加到插值器
  AddSensorSamplers(trajectory_id, options);                                      //添加到传感器采样器
  return trajectory_id;
}
//----------------------------------------AddOfflineTrajectory-----------------------------------

//----------------------------------------HandleGetTrajectoryStates-----------------------------------
bool Node::HandleGetTrajectoryStates(
    ::cartographer_ros_msgs::GetTrajectoryStates::Request& request,
    ::cartographer_ros_msgs::GetTrajectoryStates::Response& response) 
{
  using TrajectoryState = ::cartographer::mapping::PoseGraphInterface::TrajectoryState;  //创建轨迹状态枚举类
  absl::MutexLock lock(&mutex_);                                                        //加锁
response.status.code = ::cartographer_ros_msgs::StatusCode::OK;                         //定义OK状态
  response.trajectory_states.header.stamp = ros::Time::now();                           //读取当前时时间戳
  // map_builder_bridge_.GetTrajectoryStates()通过map_builder_->pose_graph()->GetTrajectoryStates();获取轨迹状态
  for (const auto& entry : map_builder_bridge_.GetTrajectoryStates()) {                 //一一读取轨迹状态             
                                                                           
    response.trajectory_states.trajectory_id.push_back(entry.first);                    //存储:unordered_map类型 的序数
    switch (entry.second) {                                                             //判别状态,是什么状态就给回应容器插入什么状态
      case TrajectoryState::ACTIVE:                                                     //
        response.trajectory_states.trajectory_state.push_back(
            ::cartographer_ros_msgs::TrajectoryStates::ACTIVE);
        break;
      case TrajectoryState::FINISHED:
        response.trajectory_states.trajectory_state.push_back(
            ::cartographer_ros_msgs::TrajectoryStates::FINISHED);
        break;
      case TrajectoryState::FROZEN:
        response.trajectory_states.trajectory_state.push_back(
            ::cartographer_ros_msgs::TrajectoryStates::FROZEN);
        break;
      case TrajectoryState::DELETED:
        response.trajectory_states.trajectory_state.push_back(
            ::cartographer_ros_msgs::TrajectoryStates::DELETED);
        break;
    }
  }
  return true;
}
// =---------------------------------------HandleGetTrajectoryStates-----------------------------------

//----------------------------------------HandleFinishTrajectory-----------------------------------
// 返回是否完成轨迹生成，并返回true
bool Node::HandleFinishTrajectory(
    ::cartographer_ros_msgs::FinishTrajectory::Request& request,
    ::cartographer_ros_msgs::FinishTrajectory::Response& response) {
  absl::MutexLock lock(&mutex_);                                              //加锁
  response.status = FinishTrajectoryUnderLock(request.trajectory_id);         /// 判定是否完成轨迹生成，并返回状态
  return true;
}
//--------------------------------------HandleFinishTrajectory-----------------------------------

//----------------------------------------HandleWriteState-----------------------------------
//根据请求将信息的 存储到pose_graph 反序列器protoc内 和返回写入状态信息
bool Node::HandleWriteState(
    ::cartographer_ros_msgs::WriteState::Request& request,
    ::cartographer_ros_msgs::WriteState::Response& response) {
  absl::MutexLock lock(&mutex_);                                               //加锁

                                                                              
  if (map_builder_bridge_.SerializeState(request.filename,                    //从这里进入map_builder_
                                         request.include_unfinished_submaps)) {
    response.status.code = cartographer_ros_msgs::StatusCode::OK;             // 调用了WritePbStream所有传感器写入器，
                                                                              //主要写入pose_graph 反序列器protoc内
                                                                              //进入map_builder
                                                                              //如果进入到反序列器则返回成功
    response.status.message =
        absl::StrCat("State written to '", request.filename, "'.");           //显示成功
  } else {                                                                    //进不去存储信息则返回无效状态
    response.status.code = cartographer_ros_msgs::StatusCode::INVALID_ARGUMENT;
    response.status.message =
        absl::StrCat("Failed to write '", request.filename, "'.");
  }
  return true;
}
//----------------------------------------HandleWriteState-----------------------------------

//----------------------------------------HandleReadMetrics-----------------------------------
// 返回Cartographer的所有内部指标的最新值。 具体还不是很了解？？？？？？？？？？有哪些指标
// 运行时度量标准的集合是可选的，必须使用节点中的--collect_metrics命令行标志激活。
bool Node::HandleReadMetrics(
    ::cartographer_ros_msgs::ReadMetrics::Request& request,
    ::cartographer_ros_msgs::ReadMetrics::Response& response) {
  absl::MutexLock lock(&mutex_);                                                //加锁
  response.timestamp = ros::Time::now();                                        //获取当前时间
  if (!metrics_registry_) {
    response.status.code = cartographer_ros_msgs::StatusCode::UNAVAILABLE;
    response.status.message = "Collection of runtime metrics is not activated.";
    return true;
  }
  metrics_registry_->ReadMetrics(&response);
  response.status.code = cartographer_ros_msgs::StatusCode::OK;
  response.status.message = "Successfully read metrics.";
  return true;
}

//----------------------------------------HandleReadMetrics-----------------------------------


//----------------------------------------FinishAllTrajectories-----------------------------------
// 轨迹状态是 ACTICE 就检验状态是否为OK，来判断是否完成轨迹生成  
void Node::FinishAllTrajectories() {
  absl::MutexLock lock(&mutex_);
  for (const auto& entry : map_builder_bridge_.GetTrajectoryStates()) {       // 获取轨迹状态
    if (entry.second == TrajectoryState::ACTIVE) {                            // 轨迹状态是 ACTICE 就检验状态是否为OK，
      const int trajectory_id = entry.first;                                  // 来判断是否完成轨迹生成   
      CHECK_EQ( FinishTrajectoryUnderLock(trajectory_id).code,
                cartographer_ros_msgs::StatusCode::OK);
    }
  }
}
//--------------------------------------FinishAllTrajectories------------------------------

//----------------------------------------FinishTrajectory-----------------------------------
// 检验状态是否为OK，来判断是否完成轨迹生成 
bool Node::FinishTrajectory(const int trajectory_id) {
  absl::MutexLock lock(&mutex_);
  return FinishTrajectoryUnderLock(trajectory_id).code ==  cartographer_ros_msgs::StatusCode::OK;
}
//----------------------------------------FinishTrajectory-----------------------------------

//--------------------------------------RunFinalOptimization-------------------------------
 // 运行轨迹最后的优化
void Node::RunFinalOptimization() 
{
  {
    for (const auto& entry : map_builder_bridge_.GetTrajectoryStates())         // 获取轨迹状态
   {
      const int trajectory_id = entry.first;
      if (entry.second == TrajectoryState::ACTIVE) 
      {
        LOG(WARNING)
            << "Can't run final optimization if there are one or more active "
               "trajectories. Trying to finish trajectory with ID "
            << std::to_string(trajectory_id) << " now.";
        CHECK(FinishTrajectory(trajectory_id))                                 // 检验状态是否为OK，来判断是否完成轨迹生成 
            << "Failed to finish trajectory with ID "
            << std::to_string(trajectory_id) << ".";
      }
    }
  }
  // Assuming we are not adding new data anymore, the final optimization
  // can be performed without holding the mutex.
  map_builder_bridge_.RunFinalOptimization();                                  // 运行轨迹最后的优化
}
//----------------------------------------RunFinalOptimization-----------------------------------

//----------------------------------------HandleOdometryMessage-----------------------------------
// 调用sensor_bridge_ptr 处理添加里程计信息
void Node::HandleOdometryMessage(const int trajectory_id,
                                 const std::string& sensor_id,
                                 const nav_msgs::Odometry::ConstPtr& msg) 
{
  absl::MutexLock lock(&mutex_);                                                //加锁
  if (!sensor_samplers_.at(trajectory_id).odometry_sampler.Pulse())             //如果 实际采样率 还没超过 设定采样率 则不返回
      return;
  auto sensor_bridge_ptr = map_builder_bridge_.sensor_bridge(trajectory_id);    //通过id 获取轨迹传感器信息     
  auto odometry_data_ptr = sensor_bridge_ptr->ToOdometryData(msg);              //将里程计信息转化为里程计数据
  if (odometry_data_ptr != nullptr) 
  {
    extrapolators_.at(trajectory_id).AddOdometryData(*odometry_data_ptr);
  }
  sensor_bridge_ptr->HandleOdometryMessage(sensor_id, msg);                     //添加里程计信息
                                                                                //进入函数 ToOdometryData 获取（位姿、时间)共享指针
                                                                                // 处理里程计信息  进入 AddSensorData
                                                                                // TrajectoryBuilderInterface 中的虚函数AddSensorData实例化  
}
//----------------------------------------HandleOdometryMessage-----------------------------------

//----------------------------------------HandleNavSatFixMessage-----------------------------------
// 同上 确定实际采样率不超过设定值 ，然后调用sensor_bridge 处理 -- GPS  --数据 添加数据
void Node::HandleNavSatFixMessage(const int trajectory_id,
                                  const std::string& sensor_id,
                                  const sensor_msgs::NavSatFix::ConstPtr& msg) {
  absl::MutexLock lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).fixed_frame_pose_sampler.Pulse()) {       //如果采样率踩过设定值 返回
    return;
  }
  map_builder_bridge_.sensor_bridge(trajectory_id)                                   //进入 AddSensorData添加数据
      ->HandleNavSatFixMessage(sensor_id, msg);
}
//----------------------------------------HandleNavSatFixMessage-----------------------------------

//----------------------------------------HandleLandmarkMessage-----------------------------------
// 同上 确定实际采样率不超过设定值 ，然后调用sensor_bridge 处理---- 路标 ---数据 添加数据
void Node::HandleLandmarkMessage(
    const int trajectory_id, const std::string& sensor_id,
    const cartographer_ros_msgs::LandmarkList::ConstPtr& msg) 
{
  absl::MutexLock lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).landmark_sampler.Pulse()) 
  {
    return;
  }
  map_builder_bridge_.sensor_bridge(trajectory_id)->HandleLandmarkMessage(sensor_id, msg);
}
//----------------------------------------HandleLandmarkMessage-----------------------------------

//---------------------------------------HandleImuMessage-----------------------------------
// 同上 确定实际采样率不超过设定值 ，然后调用sensor_bridge 处理---- IMU ---数据 添加数据 并且 添加到插值器
void Node::HandleImuMessage(const int trajectory_id,
                            const std::string& sensor_id,
                            const sensor_msgs::Imu::ConstPtr& msg) 
{
  absl::MutexLock lock(&mutex_);//加锁
  if (!sensor_samplers_.at(trajectory_id).imu_sampler.Pulse()) {
    return;
  }
  auto sensor_bridge_ptr = map_builder_bridge_.sensor_bridge(trajectory_id);
  auto imu_data_ptr = sensor_bridge_ptr->ToImuData(msg); 
  if (imu_data_ptr != nullptr) {                                               //数据不为空就添加到插值器
    extrapolators_.at(trajectory_id).AddImuData(*imu_data_ptr);                //插值器添加IMU数据     
  } 
  sensor_bridge_ptr->HandleImuMessage(sensor_id, msg);
}
//---------------------------------------HandleImuMessage-----------------------------------

//---------------------------------------HandleLaserScanMessage-----------------------------------
// 同上 确定实际采样率不超过设定值 ，然后调用sensor_bridge 处理---- 激光 ---数据 添加数据  
void Node::HandleLaserScanMessage(const int trajectory_id,
                                  const std::string& sensor_id,
                                  const sensor_msgs::LaserScan::ConstPtr& msg) {
  absl::MutexLock lock(&mutex_);
  //::FixedRatioSampler rangefinder_sampler 所以Pulse为FixedRatioSampler::Pulse() 
  if (!sensor_samplers_.at(trajectory_id).rangefinder_sampler.Pulse()) {
    return;
  }
  map_builder_bridge_.sensor_bridge(trajectory_id)
      ->HandleLaserScanMessage(sensor_id, msg);                                 //调用map_builder_bridge_的消息处理
}
//---------------------------------------HandleLaserScanMessage-----------------------------------

//---------------------------------------HandleMultiEchoLaserScanMessage-----------------------------------
// 同上 确定实际采样率不超过设定值 ，然后调用sensor_bridge 处理---- 其他激光 ---数据 添加数据 
void Node::HandleMultiEchoLaserScanMessage(
    const int trajectory_id, const std::string& sensor_id,
    const sensor_msgs::MultiEchoLaserScan::ConstPtr& msg) {
  absl::MutexLock lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).rangefinder_sampler.Pulse()) {
    return;
  }
  map_builder_bridge_.sensor_bridge(trajectory_id)
      ->HandleMultiEchoLaserScanMessage(sensor_id, msg);
}
//---------------------------------------HandleMultiEchoLaserScanMessage-----------------------------------

//---------------------------------------HandlePointCloud2Message-----------------------------------
// 同上 确定实际采样率不超过设定值 ，然后调用sensor_bridge 处理---- 点云 ---数据 添加数据 
void Node::HandlePointCloud2Message(
    const int trajectory_id, const std::string& sensor_id,
    const sensor_msgs::PointCloud2::ConstPtr& msg) {
  absl::MutexLock lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).rangefinder_sampler.Pulse()) {
    return;
  }
  map_builder_bridge_.sensor_bridge(trajectory_id)
      ->HandlePointCloud2Message(sensor_id, msg);
}
//---------------------------------------HandlePointCloud2Message-----------------------------------

//---------------------------------------SerializeState-----------------------------------
void Node::SerializeState(const std::string& filename,
                          const bool include_unfinished_submaps) {
  absl::MutexLock lock(&mutex_);
  CHECK(
      map_builder_bridge_.SerializeState(filename, include_unfinished_submaps))
      << "Could not write state.";                                              // 调用了WritePbStream所有传感器写入器，
                                                                                // 主要写入pose_graph 反序列器protoc内
                                                                                // 进入map_builder          
}
//---------------------------------------SerializeState-----------------------------------

//---------------------------------------LoadState-----------------------------------
// 读取.pbstream文件
void Node::LoadState(const std::string& state_filename,
                     const bool load_frozen_state) {
  absl::MutexLock lock(&mutex_);
  map_builder_bridge_.LoadState(state_filename, load_frozen_state);              // 读取.pbstream文件
}
//---------------------------------------LoadState-----------------------------------

//---------------------------------------MaybeWarnAboutTopicMismatch----------------------------------
void Node::MaybeWarnAboutTopicMismatch(const ::ros::WallTimerEvent& unused_timer_event) 
{
  ::ros::master::V_TopicInfo  ros_topics;
  ::ros::master::getTopics(ros_topics);
  std::set<std::string>       published_topics;
  std::stringstream           published_topics_string;
  
  for (const auto& it : ros_topics) 
  {
    std::string resolved_topic = node_handle_.resolveName(it.name, false);
    published_topics.insert(resolved_topic);
    published_topics_string << resolved_topic << ",";
  }
  bool print_topics = false;
  for (const auto& entry : subscribers_) 
  {
    int trajectory_id = entry.first;
    for (const auto& subscriber : entry.second) 
    {
      std::string resolved_topic = node_handle_.resolveName(subscriber.topic);
      if (published_topics.count(resolved_topic) == 0) 
      {
        LOG(WARNING) << "Expected topic \"" << subscriber.topic
                     << "\" (trajectory " << trajectory_id << ")"
                     << " (resolved topic \"" << resolved_topic << "\")"
                     << " but no publisher is currently active.";
        print_topics = true;
      }
    }
  }
  if (print_topics) {
    LOG(WARNING) << "Currently available topics are: "
                 << published_topics_string.str();
  }
}

}  // namespace cartographer_ros
//---------------------------------------MaybeWarnAboutTopicMismatch----------------------------------