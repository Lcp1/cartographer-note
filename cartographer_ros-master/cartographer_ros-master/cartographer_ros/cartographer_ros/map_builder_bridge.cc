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

#include "cartographer_ros/map_builder_bridge.h"

#include "absl/memory/memory.h"
#include "absl/strings/str_cat.h"
#include "cartographer/io/color.h"
#include "cartographer/io/proto_stream.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/time_conversion.h"
#include "cartographer_ros_msgs/StatusCode.h"
#include "cartographer_ros_msgs/StatusResponse.h"

namespace cartographer_ros {
namespace {

using ::cartographer::transform::Rigid3d;
// 显示参数设置
constexpr double kTrajectoryLineStripMarkerScale = 0.07;
constexpr double kLandmarkMarkerScale = 0.2;
constexpr double kConstraintMarkerScale = 0.025;
// 信息转化为rgba
::std_msgs::ColorRGBA ToMessage(const cartographer::io::FloatColor& color) {
  ::std_msgs::ColorRGBA result;
  result.r = color[0];
  result.g = color[1];
  result.b = color[2];
  result.a = 1.f;
  return result;
}
// 创建visualization_msgs对象信息显示设置
visualization_msgs::Marker CreateTrajectoryMarker(const int trajectory_id,
                                                  const std::string& frame_id) {
  visualization_msgs::Marker marker;
  marker.ns = absl::StrCat("Trajectory ", trajectory_id);//字符串连接和运算符
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.header.stamp = ::ros::Time::now();
  marker.header.frame_id = frame_id;
  marker.color = ToMessage(cartographer::io::GetColor(trajectory_id));//转RGB
  marker.scale.x = kTrajectoryLineStripMarkerScale;
  marker.pose.orientation.w = 1.;
  marker.pose.position.z = 0.05;
  return marker;
}
//获取路标指针  通过landmark_id寻找对应的index int类型
int GetLandmarkIndex(
    const std::string& landmark_id,
    std::unordered_map<std::string, int>* landmark_id_to_index) {
  auto it = landmark_id_to_index->find(landmark_id);
  // 如果是最后一个则新增一个并且假如到这个容器内，返回最新index
  if (it == landmark_id_to_index->end()) {
    const int new_index = landmark_id_to_index->size();
    landmark_id_to_index->emplace(landmark_id, new_index);
    return new_index;
  }
  // 否则指向id对应的下一个值
  return it->second;
}

// 画出可视化的标志物
visualization_msgs::Marker CreateLandmarkMarker(int landmark_index,
                                                const Rigid3d& landmark_pose,
                                                const std::string& frame_id) {
  visualization_msgs::Marker marker;               //路边信息
  marker.ns = "Landmarks";                         //命名空间namespace，
  marker.id = landmark_index;                      //与命名空间联合起来，形成唯一的id，
                                                    //  这个唯一的id可以将各个标志物区分开来，
                                                    //  使得程序可以对指定的标志物进行操作
  marker.type = visualization_msgs::Marker::SPHERE;   //类型
  marker.header.stamp = ::ros::Time::now();           //时间戳
  marker.header.frame_id = frame_id;                 
  marker.scale.x = kLandmarkMarkerScale;            //0.2柄直径
  marker.scale.y = kLandmarkMarkerScale;            //0.2箭头直径
  marker.scale.z = kLandmarkMarkerScale;            //0.2长度
  marker.color = ToMessage(cartographer::io::GetColor(landmark_index));
  marker.pose = ToGeometryMsgPose(landmark_pose);
  return marker;
}

// 存储markers 并初始化
void PushAndResetLineMarker(visualization_msgs::Marker* marker,
                            std::vector<visualization_msgs::Marker>* markers) {
  markers->push_back(*marker);
  ++marker->id;
  marker->points.clear();
}

}  // namespace

MapBuilderBridge::MapBuilderBridge(
    const NodeOptions& node_options,         //包括子图、轨迹、位姿话题发布周期
                                             //
    //进入MapBuilderInterface类
    std::unique_ptr<cartographer::mapping::MapBuilderInterface> map_builder,
    //tf侦听器
    tf2_ros::Buffer* const tf_buffer)
    : node_options_(node_options),           // 参数初始化
      map_builder_(std::move(map_builder)),  // 参数初始化
      tf_buffer_(tf_buffer) {}               // 参数初始化
// -------------------------------------------------------------------------------
// 读取.pbstream文件
void MapBuilderBridge::LoadState(const std::string& state_filename,
                                 bool load_frozen_state) {
  // Check if suffix of the state file is ".pbstream".
  const std::string suffix = ".pbstream";
  CHECK_EQ(state_filename.substr( std::max<int>(state_filename.size() - suffix.size(), 0)),
           suffix)
           << "The file containing the state to be loaded must be a "
              ".pbstream file.";
  LOG(INFO) << "Loading saved state '" 
            << state_filename 
            << "'...";
  cartographer::io::ProtoStreamReader stream(state_filename);
  //这个函数内容很大，占map_builder 大部分内容
  map_builder_->LoadState(&stream, load_frozen_state);  //进入map_builder
}
// -------------------------------------------------------------------------------
// -------------------------------------------------------------------------------
// ----------------------------sensor_bridges_----------------------------
// -------------------------------------------------------------------------------
// -------------------------------------------------------------------------------
// -------------------------------------------------------------------------------
// 通过轨迹结构体和sensorId获取轨迹Id
int MapBuilderBridge::AddTrajectory(
    const std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>&
        expected_sensor_ids,
    const TrajectoryOptions& trajectory_options) {
      //进入map_builder_ 添加轨迹
  const int trajectory_id = map_builder_->AddTrajectoryBuilder(
      expected_sensor_ids,                                      
      trajectory_options.trajectory_builder_options,              //轨迹参数
      [this](const int trajectory_id,                                //id号
             const ::cartographer::common::Time time,              //当前时间
             const Rigid3d local_pose,                             //局部位姿
             ::cartographer::sensor::RangeData range_data_in_local,//雷达信息
             const std::unique_ptr<
                 const ::cartographer::mapping::TrajectoryBuilderInterface::
                     InsertionResult>) {
        // 通过 LocalTrajectoryData::LocalSlamData 直接读取当前位姿信息
        OnLocalSlamResult(trajectory_id, time, local_pose, range_data_in_local);
      });
  LOG(INFO) << "Added trajectory with ID '" << trajectory_id << "'.";

  // Make sure there is no trajectory with 'trajectory_id' yet.
  //重点研究点 sensor_bridges_

  //确认id数量是否为零
  CHECK_EQ(sensor_bridges_.count(trajectory_id), 0);
  //等待后期深入了解
  sensor_bridges_[trajectory_id] = absl::make_unique<SensorBridge>( //进入 sensor_bridges_
                trajectory_options.num_subdivisions_per_laser_scan,
                trajectory_options.tracking_frame,
                node_options_.lookup_transform_timeout_sec, tf_buffer_,
                map_builder_->GetTrajectoryBuilder(trajectory_id)); //进入map_builder
  auto emplace_result =
      trajectory_options_.emplace(trajectory_id, trajectory_options);
  CHECK(emplace_result.second == true);
  return trajectory_id;
}

// -------------------------------------------------------------------------------

// 固定轨迹
void MapBuilderBridge::FinishTrajectory(const int trajectory_id) {
  LOG(INFO) << "Finishing trajectory with ID '" << trajectory_id << "'...";

  // Make sure there is a trajectory with 'trajectory_id'.
  CHECK(GetTrajectoryStates().count(trajectory_id));
  map_builder_->FinishTrajectory(trajectory_id);       //进入map_builder
  sensor_bridges_.erase(trajectory_id);                //清空容器unordered_map
}
// ------------------------------------------------------------------
// 运行轨迹最后的优化
void MapBuilderBridge::RunFinalOptimization() {
  LOG(INFO) << "Running final trajectory optimization...";
  map_builder_->pose_graph()->RunFinalOptimization();   //进入map_builder
}
// ------------------------------------------------------------------

bool MapBuilderBridge::SerializeState(const std::string& filename,
                                      const bool include_unfinished_submaps) {
  return map_builder_->SerializeStateToFile(include_unfinished_submaps,
                                                        // 调用了WritePbStream所有传感器写入器，主要写入pose_graph 反序列器protoc内
                                                        //进入map_builder
                                            filename);
}
// --------------------------------HandleSubmapQuery----------------------------------
//通过查询proto里面的反序列 id和submap指针  复制到texture
void MapBuilderBridge::HandleSubmapQuery(
    cartographer_ros_msgs::SubmapQuery::Request& request,
    cartographer_ros_msgs::SubmapQuery::Response& response) {
  cartographer::mapping::proto::SubmapQuery::Response response_proto;
  cartographer::mapping::SubmapId submap_id{request.trajectory_id,
                                            request.submap_index};
  const std::string error =
                                   //proto是Google提供的一个ProtoBuf库的工具Google Protobuf库
      map_builder_->SubmapToProto(submap_id, &response_proto);//来实现数据的序列化和反序列化。
                                                              // protobuffer 序列化后的几个主要优点：
                                                              // 1.序列化后体积很小，适合网络传输。
                                                              // 2.支持跨平台多语言。
                                                              // 3.消息格式升级和兼容性不错。
                                                              // 4.序列化格式速度很快，快于json的处理速度。
  if (!error.empty()) {
    LOG(ERROR) << error;
    response.status.code = cartographer_ros_msgs::StatusCode::NOT_FOUND;
    response.status.message = error;
    return;
  }

  response.submap_version = response_proto.submap_version();//版本
  //将接收到的response.texture_proto 内容复制到texture内
  for (const auto& texture_proto : response_proto.textures()) {
    response.textures.emplace_back();//texture: vector struct SubmapTexture --->SubmapTextures
                                     //包括 长宽 分辨率 体素（光强度，色彩强弱）
    auto& texture = response.textures.back();
    texture.cells.insert(texture.cells.begin(), texture_proto.cells().begin(),
                         texture_proto.cells().end());
    texture.width = texture_proto.width();                              //宽度
    texture.height = texture_proto.height();                            //高度
    texture.resolution = texture_proto.resolution();                    //分辨率
    texture.slice_pose = ToGeometryMsgPose(                             //转化为四元素信息
        cartographer::transform::ToRigid3(texture_proto.slice_pose()));
  }
  response.status.message = "Success.";//发布存储信息完成
  response.status.code = cartographer_ros_msgs::StatusCode::OK;
}
// -----------------------------HandleSubmapQuery-------------------------------------
// -----------------------------GetTrajectoryStates()-------------------------------------
// 获取轨迹状态
std::map<int, ::cartographer::mapping::PoseGraphInterface::TrajectoryState>
MapBuilderBridge::GetTrajectoryStates() {
  auto trajectory_states = map_builder_->pose_graph()->GetTrajectoryStates();
  // Add active trajectories that are not yet in the pose graph, but are e.g.
  // waiting for input sensor data and thus already have a sensor bridge.
  for (const auto& sensor_bridge : sensor_bridges_) {                     //进入sensor_bridges_
                                                                          //获取轨迹状态信息
    trajectory_states.insert(std::make_pair(                              //make_pair把元素组合成结构体
        sensor_bridge.first,
        ::cartographer::mapping::PoseGraphInterface::TrajectoryState::ACTIVE));
  }
  return trajectory_states;
}
// ------------------------------------------------------------------
// 获取子图队列信息
cartographer_ros_msgs::SubmapList MapBuilderBridge::GetSubmapList() {
  cartographer_ros_msgs::SubmapList submap_list;
  submap_list.header.stamp = ::ros::Time::now();
  submap_list.header.frame_id = node_options_.map_frame;
  for (const auto& submap_id_pose :
       map_builder_->pose_graph()->GetAllSubmapPoses()) {                //进入map_builder子图
    cartographer_ros_msgs::SubmapEntry submap_entry;
                                              
    submap_entry.is_frozen =                                             //进入map_builder
                     map_builder_->pose_graph()->IsTrajectoryFrozen(
                             submap_id_pose.id.trajectory_id);
    submap_entry.trajectory_id = submap_id_pose.id.trajectory_id;
    submap_entry.submap_index = submap_id_pose.id.submap_index;
    submap_entry.submap_version = submap_id_pose.data.version;
    submap_entry.pose = ToGeometryMsgPose(submap_id_pose.data.pose);
    submap_list.submap.push_back(submap_entry);
  }
  return submap_list;
}
// ------------------------------------------------------------------
//获取本地轨迹信息
std::unordered_map<int, MapBuilderBridge::LocalTrajectoryData>
MapBuilderBridge::GetLocalTrajectoryData() {
  std::unordered_map<int, LocalTrajectoryData> local_trajectory_data;
  for (const auto& entry : sensor_bridges_) {
    const int trajectory_id = entry.first;
    const SensorBridge& sensor_bridge = *entry.second;
    std::shared_ptr<const LocalTrajectoryData::LocalSlamData> local_slam_data;
    {
      absl::MutexLock lock(&mutex_);
      if (local_slam_data_.count(trajectory_id) == 0) {
        continue;
      }
      local_slam_data = local_slam_data_.at(trajectory_id);
    }

    // Make sure there is a trajectory with 'trajectory_id'.
    CHECK_EQ(trajectory_options_.count(trajectory_id), 1); //确认有轨迹生成
    local_trajectory_data[trajectory_id] = {
        local_slam_data,
                                       //进入map_builder 的图优化 局部到全局坐标转换
        map_builder_->pose_graph()->GetLocalToGlobalTransform(trajectory_id),
        sensor_bridge.tf_bridge().LookupToTracking(
                      local_slam_data->time,               //时间
                      trajectory_options_[trajectory_id].published_frame),
                                                          //frame
        trajectory_options_[trajectory_id]};
  }
  return local_trajectory_data;
}
// -----------------------------------------------------------
// 处理轨迹队列  返回response信息 包括请求轨迹的 时间戳 位姿 frame 的msg信息
void MapBuilderBridge::HandleTrajectoryQuery(
    cartographer_ros_msgs::TrajectoryQuery::Request& request,
    cartographer_ros_msgs::TrajectoryQuery::Response& response) {
  // This query is safe if the trajectory doesn't exist (returns 0 poses).
  // However, we can filter unwanted states at the higher level in the node.
                                                          //进入map_builder图优化的轨迹节点
  const auto node_poses = map_builder_->pose_graph()->GetTrajectoryNodePoses();
  for (const auto& node_id_data :
       node_poses.trajectory(request.trajectory_id)) {
    if (!node_id_data.data.constant_pose_data.has_value()) {
      continue;
    }
    geometry_msgs::PoseStamped pose_stamped;           //定义一个叫msg的对象，该对象使用header,pose两个数据成员
    pose_stamped.header.frame_id = node_options_.map_frame; //frame
    pose_stamped.header.stamp =                             //time时间戳
        ToRos(node_id_data.data.constant_pose_data.value().time);
    pose_stamped.pose = ToGeometryMsgPose(node_id_data.data.global_pose);
    response.trajectory.push_back(pose_stamped);
  }
  response.status.code = cartographer_ros_msgs::StatusCode::OK;  //状态
  response.status.message = absl::StrCat(                        //合并字符串
      "Retrieved ", response.trajectory.size(),
      " trajectory nodes from trajectory ", request.trajectory_id, ".");
}
//-------------------------------------------------------------------------
// 获取轨迹节点来获取TrajectoryNode的pose的列表 并且利用Marker 可视化
visualization_msgs::MarkerArray MapBuilderBridge::GetTrajectoryNodeList() {
  visualization_msgs::MarkerArray trajectory_node_list;
  const auto node_poses =   ;
  // Find the last node indices for each trajectory that have either
  // inter-submap or inter-trajectory constraints.
  std::map<int, int /* node_index */>
      trajectory_to_last_inter_submap_constrained_node;
  std::map<int, int /* node_index */>
      trajectory_to_last_inter_trajectory_constrained_node;
  for (const int trajectory_id : node_poses.trajectory_ids()) {
    trajectory_to_last_inter_submap_constrained_node[trajectory_id] = 0;
    trajectory_to_last_inter_trajectory_constrained_node[trajectory_id] = 0;
  }  
  //进入map_builder获取图优化约束列表
  const auto constraints = map_builder_->pose_graph()->constraints();
  for (const auto& constraint : constraints) {
    // ？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？
    if (constraint.tag ==
        cartographer::mapping::PoseGraphInterface::Constraint::INTER_SUBMAP) {
      if (constraint.node_id.trajectory_id ==//如果子图id 和 submap id相同
          constraint.submap_id.trajectory_id) {
      //比较子图id和节点id 
        trajectory_to_last_inter_submap_constrained_node[  //轨迹最新子图的约束节点
                              constraint.node_id.trajectory_id] =
             //选最大值 选什么最大值  id最大？
            std::max(trajectory_to_last_inter_submap_constrained_node.at(
                                         constraint.node_id.trajectory_id),
                     constraint.node_id.node_index);
      } else {
        trajectory_to_last_inter_trajectory_constrained_node
            [constraint.node_id.trajectory_id] =
                std::max(trajectory_to_last_inter_submap_constrained_node.at(
                             constraint.node_id.trajectory_id),
                         constraint.node_id.node_index);
       // ？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？
      }
    }
  }

  for (const int trajectory_id : node_poses.trajectory_ids()) {
    visualization_msgs::Marker marker =
        CreateTrajectoryMarker(trajectory_id, node_options_.map_frame);
        // 最后一个子图约束节点
    int last_inter_submap_constrained_node = std::max(//为什么是最大值
        node_poses.trajectory(trajectory_id).begin()->id.node_index,
        trajectory_to_last_inter_submap_constrained_node.at(trajectory_id));
        //最后一个轨迹约束节点
    int last_inter_trajectory_constrained_node = std::max(//为什么是最大值
        node_poses.trajectory(trajectory_id).begin()->id.node_index,
        trajectory_to_last_inter_trajectory_constrained_node.at(trajectory_id));
        // 从子图和轨迹里面挑一个最大的节点
    last_inter_submap_constrained_node =
                      std::max(last_inter_submap_constrained_node,
                               last_inter_trajectory_constrained_node);
    //判断当前轨迹是不是已经完成的了，是的就都将轨迹倒数第二个节点作为最新节点
    if (map_builder_->pose_graph()->IsTrajectoryFrozen(trajectory_id)) {
      last_inter_submap_constrained_node =
          (--node_poses.trajectory(trajectory_id).end())->id.node_index;
      last_inter_trajectory_constrained_node =
          last_inter_submap_constrained_node;
    }
    // 如果当前轨迹节点数据没有值，就将可视化的a值设为1
    marker.color.a = 1.0;
    for (const auto& node_id_data : node_poses.trajectory(trajectory_id)) {
      if (!node_id_data.data.constant_pose_data.has_value()) {
        PushAndResetLineMarker(&marker, &trajectory_node_list.markers);
        continue;
      }
      // 转化为msg信息
      const ::geometry_msgs::Point node_point =
          ToGeometryMsgPoint(node_id_data.data.global_pose.translation());
      // 存储节点到可视化容器
      marker.points.push_back(node_point);
      // 如果node id 为轨迹最新的节点，就先存储a值为1的可视化节点，并a设置为0.5
      if (node_id_data.id.node_index ==
          last_inter_trajectory_constrained_node) {
        PushAndResetLineMarker(&marker, &trajectory_node_list.markers);
        marker.points.push_back(node_point);
        marker.color.a = 0.5;
      }
      // 如果node id 为submap最新的节点，就先存储a值为1的可视化节点，并a设置为0.25
      if (node_id_data.id.node_index == last_inter_submap_constrained_node) {
        PushAndResetLineMarker(&marker, &trajectory_node_list.markers);
        marker.points.push_back(node_point);
        marker.color.a = 0.25;
      }
      // Work around the 16384 point limit in RViz by splitting the
      // trajectory into multiple markers.
      // 如果节点数目达到rviz设定最大值就直接存储可视化节点
      if (marker.points.size() == 16384) {
        PushAndResetLineMarker(&marker, &trajectory_node_list.markers);
        // Push back the last point, so the two markers appear connected.
        marker.points.push_back(node_point);
      }
    }
    //先存储在初始化可视化节点
    PushAndResetLineMarker(&marker, &trajectory_node_list.markers);
    size_t current_last_marker_id = static_cast<size_t>(marker.id - 1);
    // 如果当前只有一条轨迹，则设轨迹最大可是id值为当前可视化最新id值
    if (trajectory_to_highest_marker_id_.count(trajectory_id) == 0) {
      trajectory_to_highest_marker_id_[trajectory_id] = current_last_marker_id;
    } else {
      marker.action = visualization_msgs::Marker::DELETE;
      while (static_cast<size_t>(marker.id) <=
             trajectory_to_highest_marker_id_[trajectory_id]) {
              // 将可视化id一一传送到轨迹节点列表
        trajectory_node_list.markers.push_back(marker);
        ++marker.id;
      }
      // 读取最高可视化节点id值
      trajectory_to_highest_marker_id_[trajectory_id] = current_last_marker_id;
    }
  }
  return trajectory_node_list;
}
//-------------------------------------------------------------------------
// 获取路标位姿
visualization_msgs::MarkerArray MapBuilderBridge::GetLandmarkPosesList() {
  visualization_msgs::MarkerArray landmark_poses_list;
  // 进入map_builder 获取路标位姿
  const std::map<std::string, Rigid3d> landmark_poses =
      map_builder_->pose_graph()->GetLandmarkPoses();
  for (const auto& id_to_pose : landmark_poses) { //一个一个读取路标位姿
    landmark_poses_list.markers.push_back(CreateLandmarkMarker(//放进可视化列表
        GetLandmarkIndex(id_to_pose.first, &landmark_to_index_),
        id_to_pose.second, node_options_.map_frame));
  }
  return landmark_poses_list;
}
//-------------------------------------------------------------------------
// 获取约束列表 并且可视化
visualization_msgs::MarkerArray MapBuilderBridge::GetConstraintList() {
  visualization_msgs::MarkerArray constraint_list;                    //可视化消息队列   
  int marker_id = 0;                                                  //从零开始
  visualization_msgs::Marker constraint_intra_marker;                 //单个可视化消息
  constraint_intra_marker.id = marker_id++;                           //可视化id
  constraint_intra_marker.ns = "Intra constraints";                   //可视化命名空间
  constraint_intra_marker.type = visualization_msgs::Marker::LINE_LIST;//可视化名称
  constraint_intra_marker.header.stamp = ros::Time::now();             //可视化时间戳
  constraint_intra_marker.header.frame_id = node_options_.map_frame;   //可视化frame
  constraint_intra_marker.scale.x = kConstraintMarkerScale;            //可视化柄直径
  constraint_intra_marker.pose.orientation.w = 1.0;                    //可视化朝向
  
  // 残留可视化
  visualization_msgs::Marker residual_intra_marker = constraint_intra_marker;
  residual_intra_marker.id = marker_id++;
  residual_intra_marker.ns = "Intra residuals";
  // This and other markers which are less numerous are set to be slightly
  // above the intra constraints marker in order to ensure that they are
  // visible.
  residual_intra_marker.pose.position.z = 0.1;                         //高度为0.1,确保少量这些标记可见

  // 内部相同轨迹 约束
  visualization_msgs::Marker constraint_inter_same_trajectory_marker =
      constraint_intra_marker;
  constraint_inter_same_trajectory_marker.id = marker_id++;
  constraint_inter_same_trajectory_marker.ns =
      "Inter constraints, same trajectory";
  constraint_inter_same_trajectory_marker.pose.position.z = 0.1;

  visualization_msgs::Marker residual_inter_same_trajectory_marker =
      constraint_intra_marker;
  residual_inter_same_trajectory_marker.id = marker_id++;
  residual_inter_same_trajectory_marker.ns = "Inter residuals, same trajectory";
  residual_inter_same_trajectory_marker.pose.position.z = 0.1;
  // 内部不同轨迹 约束
  visualization_msgs::Marker constraint_inter_diff_trajectory_marker =
      constraint_intra_marker;
  constraint_inter_diff_trajectory_marker.id = marker_id++;
  constraint_inter_diff_trajectory_marker.ns =
      "Inter constraints, different trajectories";
  constraint_inter_diff_trajectory_marker.pose.position.z = 0.1;
  // 内部不同轨迹 少量约束
  visualization_msgs::Marker residual_inter_diff_trajectory_marker =
      constraint_intra_marker;
  residual_inter_diff_trajectory_marker.id = marker_id++;
  residual_inter_diff_trajectory_marker.ns =
      "Inter residuals, different trajectories";
  residual_inter_diff_trajectory_marker.pose.position.z = 0.1;
// 进入map_builder_ 获取轨迹节点位姿 所有子图位姿 约束
  const auto trajectory_node_poses =
      map_builder_->pose_graph()->GetTrajectoryNodePoses();
  const auto submap_poses = map_builder_->pose_graph()->GetAllSubmapPoses();
  const auto constraints = map_builder_->pose_graph()->constraints();


// 通过比较约束的tag类型 和 子图id和节点id是否一致 来选择显示类型
  for (const auto& constraint : constraints) {
    visualization_msgs::Marker *constraint_marker, *residual_marker;
    std_msgs::ColorRGBA color_constraint, color_residual;
    if (constraint.tag ==
        cartographer::mapping::PoseGraphInterface::Constraint::INTRA_SUBMAP) {
      constraint_marker = &constraint_intra_marker;
      residual_marker = &residual_intra_marker;
      // Color mapping for submaps of various trajectories - add trajectory id
      // to ensure different starting colors. Also add a fixed offset of 25
      // to avoid having identical colors as trajectories.
      // 不同轨迹的子贴图的颜色映射-添加轨迹id以确保不同的起始颜色。
      // 同时添加25的固定偏移，以避免与轨迹具有相同的颜色。
      color_constraint = ToMessage(
          cartographer::io::GetColor(constraint.submap_id.submap_index +
                                     constraint.submap_id.trajectory_id + 25));
      color_residual.a = 1.0;
      color_residual.r = 1.0;
    } else {
      if (constraint.node_id.trajectory_id ==
          constraint.submap_id.trajectory_id) {
        constraint_marker = &constraint_inter_same_trajectory_marker;///相同轨迹部分
        residual_marker = &residual_inter_same_trajectory_marker;
        // Bright yellow  黄色
        color_constraint.a = 1.0;
        color_constraint.r = 1.0; 
        color_constraint.g = 1.0;
      } else {
        constraint_marker = &constraint_inter_diff_trajectory_marker;///不同轨迹部分
        residual_marker = &residual_inter_diff_trajectory_marker;
        // Bright orange 橙色
        color_constraint.a = 1.0;
        color_constraint.r = 1.0;
        color_constraint.g = 165. / 255.;
      }
      // 少量部分
      // Bright cyan 亮青色 
      color_residual.a = 1.0;
      color_residual.b = 1.0; 
      color_residual.g = 1.0;
    }
    // 存储到显示器，++i先加后赋值
    for (int i = 0; i < 2; ++i) {
      constraint_marker->colors.push_back(color_constraint);
      residual_marker->colors.push_back(color_residual);
    }
    // 通过约束子图id 寻找子图位姿
    const auto submap_it = submap_poses.find(constraint.submap_id);
    if (submap_it == submap_poses.end()) {//如果是子图最后一个id，重新读取约束，不往下继续
      continue;
    }
    const auto& submap_pose = submap_it->data.pose;                        //读取子图位姿
    const auto node_it = trajectory_node_poses.find(constraint.node_id);   //读取约束节点位姿
    if (node_it == trajectory_node_poses.end()) {//如果是节点最后一个id，重新读取约束，不往下继续
      continue;
    }
    const auto& trajectory_node_pose = node_it->data.global_pose;         //读取约束节点全局地图位姿 
    const Rigid3d constraint_pose = submap_pose * constraint.pose.zbar_ij;
// 存储Marker信息 即是位姿
    constraint_marker->points.push_back(
        ToGeometryMsgPoint(submap_pose.translation()));
    constraint_marker->points.push_back(
        ToGeometryMsgPoint(constraint_pose.translation()));

    residual_marker->points.push_back(
        ToGeometryMsgPoint(constraint_pose.translation()));
    residual_marker->points.push_back(
        ToGeometryMsgPoint(trajectory_node_pose.translation()));
  }

  constraint_list.markers.push_back(constraint_intra_marker);
  constraint_list.markers.push_back(residual_intra_marker);
  constraint_list.markers.push_back(constraint_inter_same_trajectory_marker);
  constraint_list.markers.push_back(residual_inter_same_trajectory_marker);
  constraint_list.markers.push_back(constraint_inter_diff_trajectory_marker);
  constraint_list.markers.push_back(residual_inter_diff_trajectory_marker);
  return constraint_list;
}
// -------------------------------------------------
// 返回一个 SensorBridge  指针
SensorBridge* MapBuilderBridge::sensor_bridge(const int trajectory_id) {
  return sensor_bridges_.at(trajectory_id).get();
}
// -------------------------------------------------------------------
//通过 LocalTrajectoryData::LocalSlamData结构体 直接 获取局部SLAM定位结果
void MapBuilderBridge::OnLocalSlamResult(
    const int trajectory_id, 
    const ::cartographer::common::Time time,
    const Rigid3d local_pose,
    ::cartographer::sensor::RangeData range_data_in_local) {
  std::shared_ptr<const LocalTrajectoryData::LocalSlamData> local_slam_data =
      std::make_shared<LocalTrajectoryData::LocalSlamData>(
          LocalTrajectoryData::LocalSlamData{time,                             //获取局部SLAM位姿
                                             local_pose,
                                             std::move(range_data_in_local)});
  absl::MutexLock lock(&mutex_);                                                //锁住线程                 
  local_slam_data_[trajectory_id] = std::move(local_slam_data);                 //将指针赋值到类成员//本地slam 数据
}

}  // namespace cartographer_ros
