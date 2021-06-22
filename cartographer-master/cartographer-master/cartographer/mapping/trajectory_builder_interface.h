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

#ifndef CARTOGRAPHER_MAPPING_TRAJECTORY_BUILDER_INTERFACE_H_
#define CARTOGRAPHER_MAPPING_TRAJECTORY_BUILDER_INTERFACE_H_

#include <functional>
#include <memory>
#include <string>

#include "absl/memory/memory.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"
#include "cartographer/mapping/submaps.h"
#include "cartographer/sensor/fixed_frame_pose_data.h"
#include "cartographer/sensor/imu_data.h"
#include "cartographer/sensor/landmark_data.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/sensor/timed_point_cloud_data.h"

namespace cartographer {
namespace mapping {

proto::TrajectoryBuilderOptions CreateTrajectoryBuilderOptions(
    common::LuaParameterDictionary* const parameter_dictionary);

class LocalSlamResultData;

// This interface is used for both 2D and 3D SLAM. Implementations wire up a
// global SLAM stack, i.e. local SLAM for initial pose estimates, scan matching
// to detect loop closure, and a sparse pose graph optimization to compute
// optimized pose estimates.
class TrajectoryBuilderInterface {
 public:
  struct InsertionResult {
    NodeId node_id;　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　
    //包含两个部分：
    //一个int型的trajectory_id和一个int型的node_index
    //重载了一些trajectory_id和node_index与其他比较运算符 == != <
    //并且对trajectory_id和node_index进行序列化

    std::shared_ptr<const TrajectoryNode::Data> constant_data;               
    //结构体 : 时间 　旋转矩阵　经过投影滤波点云　　局部slam位姿

    std::vector<std::shared_ptr<const Submap>> insertion_submaps;            
    //主要处理子图的位姿 子图状态(是否完成)  
    //子图的proto类型数据 以及转换proto
    // 建立好的子图列表
  };

  // A callback which is called after local SLAM processes an accumulated
  // 'sensor::RangeData'. If the data was inserted into a submap, reports the
  // assigned 'NodeId', otherwise 'nullptr' if the data was filtered out.
  using LocalSlamResultCallback =
      std::function<void(int /* trajectory ID */, 
                         common::Time,
                         transform::Rigid3d /* local pose estimate */,     
                         sensor::RangeData /* in local frame */,
                         std::unique_ptr<const InsertionResult>)>;

  struct SensorId {                                                        
      //定义传感器id  枚举类型 
    enum class SensorType { RANGE = 0, 
                            IMU,
                            ODOMETRY,
                            FIXED_FRAME_POSE,
                            LANDMARK,
                            LOCAL_SLAM_RESULT
    };

    SensorType type;                                                      
    //创建传感器id  枚举类型 对象
    std::string id;                                                       
    //字符串

    bool operator==(const SensorId& other) const {
      return std::forward_as_tuple(type, id) ==  
             std::forward_as_tuple(other.type, other.id);
    }

    bool operator<(const SensorId& other) const {
      return std::forward_as_tuple(type, id) <
             std::forward_as_tuple(other.type, other.id);
    }
  };

  TrajectoryBuilderInterface() {}
  // 虚函数 
  virtual ~TrajectoryBuilderInterface() {}

  TrajectoryBuilderInterface(const TrajectoryBuilderInterface&) = delete;
  TrajectoryBuilderInterface& operator=(const TrajectoryBuilderInterface&) =
      delete;
      //下面主要是添加不同传感器信息的函数
      //虚函数,需要实例化
  virtual void AddSensorData(
      const std::string& sensor_id,
      const sensor::TimedPointCloudData& timed_point_cloud_data) = 0;
  virtual void AddSensorData(const std::string& sensor_id,                      
                            //imu
                             const sensor::ImuData& imu_data) = 0;
  virtual void AddSensorData(const std::string& sensor_id,                      
  //odom
                             const sensor::OdometryData& odometry_data) = 0;
  virtual void AddSensorData(                                                   //GPS
      const std::string& sensor_id,
      const sensor::FixedFramePoseData& fixed_frame_pose) = 0;
  virtual void AddSensorData(const std::string& sensor_id,                       //路标
                             const sensor::LandmarkData& landmark_data) = 0;
  // Allows to directly add local SLAM results to the 'PoseGraph'. Note that it
  // is invalid to add local SLAM results for a trajectory that has a
  // 'LocalTrajectoryBuilder2D/3D'.
  virtual void AddLocalSlamResultData( 
      std::unique_ptr<mapping::LocalSlamResultData> local_slam_result_data) = 0;   //Data :传感器id 获取时间 添加轨迹生成器
                                                                                   //添加 轨迹id到全局位姿
};

proto::SensorId ToProto(const TrajectoryBuilderInterface::SensorId& sensor_id);    //传感器id 转proto 序列化
TrajectoryBuilderInterface::SensorId FromProto(                                    //获取传感器id proto 信息
    const proto::SensorId& sensor_id_proto);

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_TRAJECTORY_BUILDER_INTERFACE_H_
