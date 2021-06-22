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

#include "cartographer/mapping/map_builder.h"

#include "absl/memory/memory.h"
#include "absl/types/optional.h"
#include "cartographer/common/time.h"
#include "cartographer/io/internal/mapping_state_serialization.h"
#include "cartographer/io/proto_stream.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer/io/serialization_format_migration.h"
#include "cartographer/mapping/internal/2d/local_trajectory_builder_2d.h"
#include "cartographer/mapping/internal/2d/pose_graph_2d.h"
#include "cartographer/mapping/internal/3d/local_trajectory_builder_3d.h"
#include "cartographer/mapping/internal/3d/pose_graph_3d.h"
#include "cartographer/mapping/internal/collated_trajectory_builder.h"
#include "cartographer/mapping/internal/global_trajectory_builder.h"
#include "cartographer/mapping/internal/motion_filter.h"
#include "cartographer/sensor/internal/collator.h"
#include "cartographer/sensor/internal/trajectory_collator.h"
#include "cartographer/sensor/internal/voxel_filter.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"

namespace cartographer {
namespace mapping {
namespace {

using mapping::proto::SerializedData;
// -------------------------------SelectRangeSensorIds---------------------------------
//挑选激光雷达id数据
std::vector<std::string> SelectRangeSensorIds(
    const std::set<MapBuilder::SensorId>& expected_sensor_ids) {
  std::vector<std::string> range_sensor_ids;
  for (const MapBuilder::SensorId& sensor_id : expected_sensor_ids) {         
    if (sensor_id.type == MapBuilder::SensorId::SensorType::RANGE) {             //如果id符合 ，就挑选为激光雷达数据
      range_sensor_ids.push_back(sensor_id.id);
    }
  }
  return range_sensor_ids;
}
// ------------------------------------MaybeAddPureLocalizationTrimmer--------------------------------------
// 可加纯定位微调器  纯定位和纯定位微调器的区别是 num_submaps_to_keep 数值不一致
// 通过改变任务器的参数来 改变 纯定位方式
void MaybeAddPureLocalizationTrimmer( const int trajectory_id,
                                      const proto::TrajectoryBuilderOptions& trajectory_options,
                                      PoseGraph* pose_graph) 
{
  if (trajectory_options.pure_localization()) {                                //如果是纯定位模式
    LOG(WARNING)
        << "'TrajectoryBuilderOptions::pure_localization' field is deprecated. "//使用pure_localization 而不使用pure_localization_trimmer
           "Use 'TrajectoryBuilderOptions::pure_localization_trimmer' instead.";   
    pose_graph->AddTrimmer(absl::make_unique<PureLocalizationTrimmer>(
        trajectory_id, 3 /* max_submaps_to_keep */));
    return;
  }
  if (trajectory_options.has_pure_localization_trimmer()) {                     //使用pure_localization_trimmer
    pose_graph->AddTrimmer(absl::make_unique<PureLocalizationTrimmer>(             
        trajectory_id,
        trajectory_options.pure_localization_trimmer().max_submaps_to_keep()));
  }
}

}  // namespace
// -----------------------------------------MaybeAddPureLocalizationTrimmer---------------------------------------

// -------------------------------------:MapBuilder-------------------------------------------
// 建图选项参数主要是 选择2d 还是 3d  ，就会选择不同的优化器
MapBuilder::MapBuilder(const proto::MapBuilderOptions& options)
    : options_(options), thread_pool_(options.num_background_threads()) {
  CHECK(options.use_trajectory_builder_2d() ^
        options.use_trajectory_builder_3d());
  if (options.use_trajectory_builder_2d()) {                                    //如果使用2d轨迹构建器
    pose_graph_ = absl::make_unique<PoseGraph2D>(                               //创建2d图优化函数智能指针
        options_.pose_graph_options(),                                          //配置参数
        absl::make_unique<optimization::OptimizationProblem2D>(options_.pose_graph_options().optimization_problem_options()),
                                                                                //创建优化器智能指针
        &thread_pool_);                                                         //线程池
  }
  if (options.use_trajectory_builder_3d()) {                                    //同上
    pose_graph_ = absl::make_unique<PoseGraph3D>(
        options_.pose_graph_options(),
        absl::make_unique<optimization::OptimizationProblem3D>(
            options_.pose_graph_options().optimization_problem_options()),
        &thread_pool_);
  }
  if (options.collate_by_trajectory()) {                                       ////根据collate_by_trajectory的不同，CollatorInterface有两种不同的实现
    sensor_collator_ = absl::make_unique<sensor::TrajectoryCollator>();
  } else {
    sensor_collator_ = absl::make_unique<sensor::Collator>();
  }
}
// -------------------------------------MapBuilder-------------------------------------------

// -----------------------------------:AddTrajectoryBuilder----------------------------------------
//   创建一个TrajectoryBuilder并返回他的index，即trajectory_id
int MapBuilder::AddTrajectoryBuilder(
    const std::set<SensorId>& expected_sensor_ids,
    const proto::TrajectoryBuilderOptions& trajectory_options,
    LocalSlamResultCallback local_slam_result_callback) 
{
  const int trajectory_id = trajectory_builders_.size();                         //通过轨迹器容量来读取当前id

  absl::optional<MotionFilter> pose_graph_odometry_motion_filter;                //创建滤波器
  if (trajectory_options.has_pose_graph_odometry_motion_filter())                //判断是否有里程计滤波器
  {
    LOG(INFO) << "Using a motion filter for adding odometry to the pose graph.";  
    pose_graph_odometry_motion_filter.emplace(
        MotionFilter(trajectory_options.pose_graph_odometry_motion_filter()));   //有的就将 赋值给滤波器参数
  }
  ////如果使用3d轨迹构建器
  if (options_.use_trajectory_builder_3d()) {                                    //如果使用3d轨迹构建器
    std::unique_ptr<LocalTrajectoryBuilder3D> local_trajectory_builder;
    if (trajectory_options.has_trajectory_builder_3d_options()) 
    {
      local_trajectory_builder = absl::make_unique<LocalTrajectoryBuilder3D>(    //创建轨迹3d构建器 智能指针
                                              trajectory_options.trajectory_builder_3d_options(),
                                              SelectRangeSensorIds(expected_sensor_ids));    //选择激光雷达id
    }
                                                                                         // 检查类型转化
    DCHECK(dynamic_cast<PoseGraph3D*>(pose_graph_.get()));                        //dynamic_cast是强制类型转化，
                                                                                  //把 PoseGraphInterface 的指针转化为 PoseGraph3D
    //  CollatedTrajectoryBuilder : public TrajectoryBuilderInterface  虚类实例化
    // 将采集数据放到轨迹生成器容器内
    trajectory_builders_.push_back(absl::make_unique<CollatedTrajectoryBuilder>(  
                              trajectory_options,                                  //参数设置
                              sensor_collator_.get(),                              //sensor_collator,收集传感器数据 指针
                              trajectory_id,                                       //轨迹
                              expected_sensor_ids,                                 //数据类型 激光雷达
                              CreateGlobalTrajectoryBuilder3D(                     //通过局部轨迹轨迹 来构建 全局轨迹生成器
                                        std::move(local_trajectory_builder), 
                                        trajectory_id,                             //轨迹
                                        static_cast<PoseGraph3D*>(pose_graph_.get()),   //类型转换
                                        local_slam_result_callback,                //回调函数
                                        pose_graph_odometry_motion_filter)));      //滤波器
  } else { //如果不使用3d轨迹构建器 那就是2d
    std::unique_ptr<LocalTrajectoryBuilder2D> local_trajectory_builder;            //创建局部2d轨迹生成器
    if (trajectory_options.has_trajectory_builder_2d_options())                    // 如果使用2d轨迹构建器
    {
      local_trajectory_builder = absl::make_unique<LocalTrajectoryBuilder2D>(       //创建局部2d轨迹生成器  智能指针
                                    trajectory_options.trajectory_builder_2d_options(),  //2d轨迹参数
                                    SelectRangeSensorIds(expected_sensor_ids));     //数据类型 激光雷达
    }
    DCHECK(dynamic_cast<PoseGraph2D*>(pose_graph_.get()));                          //dynamic_cast是强制类型转化，
    
    trajectory_builders_.push_back(absl::make_unique<CollatedTrajectoryBuilder>(    // 将采集2d数据放到轨迹生成器容器内，
                                      trajectory_options,                            //同上
                                      sensor_collator_.get(), 
                                      trajectory_id,
                                      expected_sensor_ids,
                                      CreateGlobalTrajectoryBuilder2D(std::move(
                                                local_trajectory_builder), 
                                                trajectory_id,
                                                static_cast<PoseGraph2D*>(pose_graph_.get()),
                                                local_slam_result_callback, 
                                                pose_graph_odometry_motion_filter)));
  }
  MaybeAddPureLocalizationTrimmer(  trajectory_id,                                   //纯定位微调器
                                    trajectory_options,
                                    pose_graph_.get());

  if (trajectory_options.has_initial_trajectory_pose())                              //如果有初始位姿
  {
    const auto& initial_trajectory_pose = trajectory_options.initial_trajectory_pose();
                                                                                      //获取参数设置的初始位姿

    pose_graph_->SetInitialTrajectoryPose( trajectory_id, 
                                           initial_trajectory_pose.to_trajectory_id(),
                                           transform::ToRigid3(initial_trajectory_pose.relative_pose()),
                                           common::FromUniversal(initial_trajectory_pose.timestamp()));
                                                                                      //  根据轨迹id号、相关变换矩阵、时间戳
                                                                                      //将初始位姿设置到图优化初始化轨迹位姿
  }
  proto::TrajectoryBuilderOptionsWithSensorIds options_with_sensor_ids_proto;         //定义序列化传感器id参数
  for (const auto& sensor_id : expected_sensor_ids)                                   //读取传感器id号数据容器
      *options_with_sensor_ids_proto.add_sensor_id() = ToProto(sensor_id);            //将数据 序列化 并存储到序列化容器

  *options_with_sensor_ids_proto.mutable_trajectory_builder_options() = trajectory_options;
                                                                                      //读取轨迹创建参数配置
  all_trajectory_builder_options_.push_back(options_with_sensor_ids_proto);           //放置到全局轨迹生成器参数内
  CHECK_EQ(  trajectory_builders_.size(),                                             //判断全局与局部是否一致
             all_trajectory_builder_options_.size());
  return trajectory_id;                                                               //返回当前轨迹序号
}
// ------------------------------------:AddTrajectoryBuilder--------------------------------------------

// --------------------------------------------------------------------------------
//增加反序列化轨迹参数器 到全局轨迹生成器参数设置器
int MapBuilder::AddTrajectoryForDeserialization(
         const proto::TrajectoryBuilderOptionsWithSensorIds& options_with_sensor_ids_proto)
{
  const int trajectory_id = trajectory_builders_.size();                               //获取当前轨迹id
  trajectory_builders_.emplace_back();                                                 //新增轨迹
  all_trajectory_builder_options_.push_back(options_with_sensor_ids_proto);            //存储轨迹参数设置
  CHECK_EQ(trajectory_builders_.size(), all_trajectory_builder_options_.size());       //判断设置参数轨迹量 与 全局轨迹量是否一致
  return trajectory_id;
}
// --------------------------------------------------------------------------------

// ----------------------------------FinishTrajectory----------------------------------------------
//完成轨迹构建
void MapBuilder::FinishTrajectory(const int trajectory_id) {                             //
  sensor_collator_->FinishTrajectory(trajectory_id);
  pose_graph_->FinishTrajectory(trajectory_id);
}
// --------------------------------------------------------------------------------

// -------------------------------------SubmapToProto-------------------------------------------
//子图序列化
std::string MapBuilder::SubmapToProto(const SubmapId& submap_id,
                                      proto::SubmapQuery::Response* const response) 
{
  if (submap_id.trajectory_id < 0 ||
      submap_id.trajectory_id >= num_trajectory_builders()) 
  {
    return "Requested submap from trajectory " +
           std::to_string(submap_id.trajectory_id) + " but there are only " +
           std::to_string(num_trajectory_builders()) + " trajectories.";
  }
  const auto submap_data = pose_graph_->GetSubmapData(submap_id);                    //通过id获取子图数据
  if (submap_data.submap == nullptr) {                                               //子图如果为空，判定该子图在轨迹中已经被修剪
    return "Requested submap " + std::to_string(submap_id.submap_index) +
           " from trajectory " + std::to_string(submap_id.trajectory_id) +
           " but it does not exist: maybe it has been trimmed.";
  }
  submap_data.submap->ToResponseProto(submap_data.pose, response);                    //
  return "";
}
// --------------------------------------------------------------------------------

// --------------------------------SerializeState------------------------------------------------
//调用io::WritePbStream工具，保存所有子图数据到序列化 
void MapBuilder::SerializeState(bool include_unfinished_submaps,                      //未完成子图        
                                io::ProtoStreamWriterInterface* const writer) {
  io::WritePbStream(*pose_graph_, 
                    all_trajectory_builder_options_, 
                    writer,
                    include_unfinished_submaps);
}
// --------------------------------------------------------------------------------

// --------------------------------SerializeStateToFile------------------------------------------------
// 将反序列器写到文件内
bool MapBuilder::SerializeStateToFile(bool include_unfinished_submaps,
                                      const std::string& filename) {
  io::ProtoStreamWriter writer(filename);

  io::WritePbStream(*pose_graph_,                                                      // 调用了所有传感器写入器，主要写入pose_graph 反序列器内 
                    all_trajectory_builder_options_, 
                    &writer,
                    include_unfinished_submaps);
  return (writer.Close());
}
// --------------------------------------------------------------------------------

// ----------------------------------LoadState----------------------------------------------
// 主要功能就是从proto流中构造出当前的状态。
std::map<int, int> MapBuilder::LoadState(
    io::ProtoStreamReaderInterface* const reader,
    bool load_frozen_state) 
{
  io::ProtoStreamDeserializer deserializer(reader);                          //构建反序列器

  // Create a copy of the pose_graph_proto, such that we can re-write the
  // trajectory ids.
  // 读取pose_graph                                                         //创建pose_graph_proto 进行重写 trajectory ids
  proto::PoseGraph pose_graph_proto = deserializer.pose_graph();             //反序列 图优化器
  const auto& all_builder_options_proto =  deserializer.all_trajectory_builder_options();
                                                                              //反序列 所有轨迹参数
  std::map<int, int> trajectory_remapping;
  //逐条trajectory恢复
  for (int i = 0; i < pose_graph_proto.trajectory_size(); ++i)                //图优化轨迹节点数量
  {
    auto& trajectory_proto = *pose_graph_proto.mutable_trajectory(i);         //
    const auto& options_with_sensor_ids_proto =  all_builder_options_proto.options_with_sensor_ids(i);
    const int new_trajectory_id =  AddTrajectoryForDeserialization(options_with_sensor_ids_proto);
    CHECK(trajectory_remapping.emplace(trajectory_proto.trajectory_id(), new_trajectory_id).second)
        << "Duplicate trajectory ID: " << trajectory_proto.trajectory_id();
    trajectory_proto.set_trajectory_id(new_trajectory_id);
    if (load_frozen_state) {
      pose_graph_->FreezeTrajectory(new_trajectory_id);
    }
  }
  // Apply the calculated remapping to constraints in the pose graph proto.
  for (auto& constraint_proto : *pose_graph_proto.mutable_constraint()) {
    constraint_proto.mutable_submap_id()->set_trajectory_id(
        trajectory_remapping.at(constraint_proto.submap_id().trajectory_id()));
    constraint_proto.mutable_node_id()->set_trajectory_id(
        trajectory_remapping.at(constraint_proto.node_id().trajectory_id()));
  }
// 分轨迹读取submap的pose
  MapById<SubmapId, transform::Rigid3d> submap_poses;                                 //创建子图位姿  根据轨迹和子图id
  for (const proto::Trajectory& trajectory_proto :
       pose_graph_proto.trajectory()) {
    for (const proto::Trajectory::Submap& submap_proto :
         trajectory_proto.submap()) {
      submap_poses.Insert(SubmapId{trajectory_proto.trajectory_id(),
                                   submap_proto.submap_index()},
                          transform::ToRigid3(submap_proto.pose()));
    }
  }
// 分轨迹读取node的pose
  MapById<NodeId, transform::Rigid3d> node_poses;                                       //创建节点位姿 根据轨迹和节点id
  for (const proto::Trajectory& trajectory_proto :
       pose_graph_proto.trajectory()) {
    for (const proto::Trajectory::Node& node_proto : trajectory_proto.node()) {
      node_poses.Insert(
          NodeId{trajectory_proto.trajectory_id(), node_proto.node_index()},
          transform::ToRigid3(node_proto.pose()));
    }
  }
 // 读取landmark的pose
  // Set global poses of landmarks.
  for (const auto& landmark : pose_graph_proto.landmark_poses()) {                            //设置全局路标位姿
    pose_graph_->SetLandmarkPose(landmark.landmark_id(),
                                 transform::ToRigid3(landmark.global_pose()),
                                 true);
  }

  if (options_.use_trajectory_builder_3d()) {
    CHECK_NE(deserializer.header().format_version(),
             io::kFormatVersionWithoutSubmapHistograms)
        << "The pbstream file contains submaps without rotational histograms. "
           "This can be converted with the 'pbstream migrate' tool, see the "
           "Cartographer documentation for details. ";
  }
  //定义序列化数据
  SerializedData proto;                                                                           //对所有数据进行反序列化    
  while (deserializer.ReadNextSerializedData(&proto)) {
    switch (proto.data_case()) {
      case SerializedData::kPoseGraph:                                                        //图
        LOG(ERROR) << "Found multiple serialized `PoseGraph`. Serialized "
                      "stream likely corrupt!.";
        break;
      case SerializedData::kAllTrajectoryBuilderOptions:                                       //所有轨迹参数
        LOG(ERROR) << "Found multiple serialized "
                      "`AllTrajectoryBuilderOptions`. Serialized stream likely "
                      "corrupt!.";
        break;
      // 读取submap  
      case SerializedData::kSubmap:                                                            //子图
      { 
        proto.mutable_submap()->mutable_submap_id()->set_trajectory_id(
            trajectory_remapping.at( proto.submap().submap_id().trajectory_id()));
        const SubmapId submap_id(proto.submap().submap_id().trajectory_id(),
                                 proto.submap().submap_id().submap_index());
        pose_graph_->AddSubmapFromProto(submap_poses.at(submap_id),
                                        proto.submap());
        break;
      }
      case SerializedData::kNode:                                                              //节点
      {
        proto.mutable_node()->mutable_node_id()->set_trajectory_id(
            trajectory_remapping.at(proto.node().node_id().trajectory_id()));
        const NodeId node_id(proto.node().node_id().trajectory_id(),
                             proto.node().node_id().node_index());
        const transform::Rigid3d& node_pose = node_poses.at(node_id);
        pose_graph_->AddNodeFromProto(node_pose, proto.node());
        break;
      }
      case SerializedData::kTrajectoryData:                                                    //轨迹数据
      {
        proto.mutable_trajectory_data()->set_trajectory_id(
            trajectory_remapping.at(proto.trajectory_data().trajectory_id()));
        pose_graph_->SetTrajectoryDataFromProto(proto.trajectory_data());
        break;
      }
      case SerializedData::kImuData:                                                           //imu数据
      {
        if (load_frozen_state) break;
        pose_graph_->AddImuData(
            trajectory_remapping.at(proto.imu_data().trajectory_id()),
            sensor::FromProto(proto.imu_data().imu_data()));
        break;
      }
      case SerializedData::kOdometryData:                                                    //里程计数据 
      {
        if (load_frozen_state) break;
        pose_graph_->AddOdometryData(
            trajectory_remapping.at(proto.odometry_data().trajectory_id()),
            sensor::FromProto(proto.odometry_data().odometry_data()));
        break;
      }
      case SerializedData::kFixedFramePoseData:                                              //GPS数据 
      {
        if (load_frozen_state) break;
        pose_graph_->AddFixedFramePoseData(
            trajectory_remapping.at(
                proto.fixed_frame_pose_data().trajectory_id()),
            sensor::FromProto(
                proto.fixed_frame_pose_data().fixed_frame_pose_data()));
        break;
      }
      case SerializedData::kLandmarkData:                                                    //路标数据 
      {
        if (load_frozen_state) break;
        pose_graph_->AddLandmarkData(
            trajectory_remapping.at(proto.landmark_data().trajectory_id()),
            sensor::FromProto(proto.landmark_data().landmark_data()));
        break;
      }
      default:
        LOG(WARNING) << "Skipping unknown message type in stream: "
                     << proto.GetTypeName();
    }
  }

  if (load_frozen_state) {                                                        //在图中添加子图对应的节点  理解为约束           
    // Add information about which nodes belong to which submap.
    // This is required, even without constraints.
    for (const proto::PoseGraph::Constraint& constraint_proto :pose_graph_proto.constraint())  
    {
      if (constraint_proto.tag() != proto::PoseGraph::Constraint::INTRA_SUBMAP) 
          continue;

      pose_graph_->AddNodeToSubmap( NodeId{  constraint_proto.node_id().trajectory_id(),
                                            constraint_proto.node_id().node_index()},        
                                    SubmapId{constraint_proto.submap_id().trajectory_id(),
                                            constraint_proto.submap_id().submap_index()});
    }
  } else {
    // When loading unfrozen trajectories, 'AddSerializedConstraints' will
    // take care of adding information about which nodes belong to which
    // submap.
    pose_graph_->AddSerializedConstraints(  FromProto(pose_graph_proto.constraint()));
  }
  CHECK(reader->eof());
  return trajectory_remapping;
}










std::map<int, int> MapBuilder::LoadStateFromFile(
    const std::string& state_filename, const bool load_frozen_state) {
  const std::string suffix = ".pbstream";
  if (state_filename.substr(
          std::max<int>(state_filename.size() - suffix.size(), 0)) != suffix) {
    LOG(WARNING) << "The file containing the state should be a "
                    ".pbstream file.";
  }
  LOG(INFO) << "Loading saved state '" << state_filename << "'...";
  io::ProtoStreamReader stream(state_filename);
  return LoadState(&stream, load_frozen_state);
}

std::unique_ptr<MapBuilderInterface> CreateMapBuilder(
    const proto::MapBuilderOptions& options) {
  return absl::make_unique<MapBuilder>(options);
}

}  // namespace mapping
}  // namespace cartographer
