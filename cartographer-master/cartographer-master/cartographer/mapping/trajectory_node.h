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

#ifndef CARTOGRAPHER_MAPPING_TRAJECTORY_NODE_H_
#define CARTOGRAPHER_MAPPING_TRAJECTORY_NODE_H_

#include <memory>
#include <vector>

#include "Eigen/Core"
#include "absl/types/optional.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/proto/trajectory_node_data.pb.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping {

struct TrajectoryNodePose {
  struct ConstantPoseData {
    common::Time time;
    transform::Rigid3d local_pose;
  };
  // The node pose in the global SLAM frame.
  transform::Rigid3d global_pose;

  absl::optional<ConstantPoseData> constant_pose_data;
};

struct TrajectoryNode {     //时间 　旋转矩阵　点云数据　　局部ｓｌａｍ位姿
  struct Data {
    common::Time time;

    // Transform to approximately gravity align the tracking frame as
    // determined by local SLAM.
    Eigen::Quaterniond gravity_alignment;　　　                     
    // 一个表示旋转矩阵的四元数。该旋转矩阵将非水平面的传感器数据投射到水平面上
     // 利用IMU的重力传感器可计算出该旋转矩阵 为雷达与水平方向的俯仰夹角，用于在水平方向进行投影

    // Used for loop closure in 2D: voxel filtered returns in the
    // 'gravity_alignment' frame.
    sensor::PointCloud filtered_gravity_aligned_point_cloud;　    　
    //经过水平投射后的点云数据，可用于2D情况下做Loop Closure.

    // Used for loop closure in 3D.
    sensor::PointCloud high_resolution_point_cloud;                
    //高分辨率点云数据
    sensor::PointCloud low_resolution_point_cloud;                 
    //低分辨率点云数据 
    Eigen::VectorXf rotational_scan_matcher_histogram;            
     //旋转匹配直方图   VectorXf是一个长度可变的向量。
 
    // The node pose in the local SLAM frame.
    transform::Rigid3d local_pose;                                  
    //节点在Local 子图中的Pose
  };

  common::Time time() const { return constant_data->time; }     //返回data时间

  // This must be a shared_ptr. If the data is used for visualization while the
  // node is being trimmed, it must survive until all use finishes.
  std::shared_ptr<const Data> constant_data;　　　　　　　　　　　//创建了一个Data结构体,

  // The node pose in the global SLAM frame.
  transform::Rigid3d global_pose;                                //节点在全局 SLAM中的Pose
};

proto::TrajectoryNodeData ToProto(const TrajectoryNode::Data& constant_data);      //将:Data 结构体数据进行序列化
TrajectoryNode::Data FromProto(const proto::TrajectoryNodeData& proto);

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_TRAJECTORY_NODE_H_
