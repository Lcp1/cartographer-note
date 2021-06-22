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
// -----------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------
// 该文件根据2d或者3d在submap2D.cpp 或者submap3D.cpp 中实现
// -----------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------
// --------------------------------------------------------------------------
// --------------------------------------------------------------------------
// 主要处理子图的位姿 子图状态(是否完成)  子图的proto类型数据 以及转换proto
// --------------------------------------------------------------------------
#ifndef CARTOGRAPHER_MAPPING_SUBMAPS_H_
#define CARTOGRAPHER_MAPPING_SUBMAPS_H_

#include <memory>
#include <vector>

#include "Eigen/Geometry"
#include "cartographer/common/math.h"
#include "cartographer/common/port.h"
#include "cartographer/mapping/id.h"
#include "cartographer/mapping/probability_values.h"
#include "cartographer/mapping/proto/serialization.pb.h"
#include "cartographer/mapping/proto/submap_visualization.pb.h"
#include "cartographer/mapping/trajectory_node.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

// Converts the given probability to log odds.
inline float Logit(float probability) {
  return std::log(probability / (1.f - probability));
}

const float kMaxLogOdds = Logit(kMaxProbability);
const float kMinLogOdds = Logit(kMinProbability);

// Converts a probability to a log odds integer. 0 means unknown, [kMinLogOdds,
// kMaxLogOdds] is mapped to [1, 255].
inline uint8 ProbabilityToLogOddsInteger(const float probability) {
  const int value = common::RoundToInt((Logit(probability) - kMinLogOdds) *
                                       254.f / (kMaxLogOdds - kMinLogOdds)) +
                    1;
  CHECK_LE(1, value);
  CHECK_GE(255, value);
  return value;
}

// An individual submap, which has a 'local_pose' in the local map frame, keeps
// track of how many range data were inserted into it, and sets
// 'insertion_finished' when the map no longer changes and is ready for loop
// closing.

class Submap {
 public:
  Submap(const transform::Rigid3d& local_submap_pose)                       
  //将局部slam位姿局部变量,赋值到全局位姿变量
      : local_pose_(local_submap_pose) {}
  virtual ~Submap() {}


// 序列化与反序列化
// 三个纯虚类 需要继承它的 类去实现
  virtual proto::Submap ToProto(bool include_grid_data) const = 0;          
  //栅格数据序列化 
  virtual void UpdateFromProto(const proto::Submap& proto) = 0;             
  //更新子图 在proto中的信息

  // Fills data into the 'response'.
 
  virtual void ToResponseProto(                                            
     // 把submap的放入到response的proto流中。
      const transform::Rigid3d& global_submap_pose,                         
      //方便service中查询submap讯息
      proto::SubmapQuery::Response* response) const = 0;                                                                     
      

  // Pose of this submap in the local map frame.
  transform::Rigid3d local_pose() const { return local_pose_; }              
  //返回submap在局部地图中的位姿局部位姿    local_pose_

  // Number of RangeData inserted.
  int num_range_data() const { return num_range_data_; }                     
  //返回该子图激光数量 是帧  ??????????
  void set_num_range_data(const int num_range_data) {                        
    //将局部激光数量变量 赋值到 全局变量 
    num_range_data_ = num_range_data;
  }

  bool insertion_finished() const { return insertion_finished_; }            
  // 查看布尔型成员变量finished_，即该子图是否还需要更新
  void set_insertion_finished(bool insertion_finished) {                     
    //设置该子图已经完成
    insertion_finished_ = insertion_finished;
  }

 private:
  const transform::Rigid3d local_pose_;                                      
  //  局部位姿
  int num_range_data_ = 0;                                                  
   //  激光帧数
  bool insertion_finished_ = false;                                          
  //  子图信息插入完成状态
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_SUBMAPS_H_
