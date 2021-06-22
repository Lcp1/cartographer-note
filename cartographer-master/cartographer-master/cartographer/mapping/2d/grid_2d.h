/*
 * Copyright 2018 The Cartographer Authors
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

#ifndef CARTOGRAPHER_MAPPING_2D_GRID_2D_H_
#define CARTOGRAPHER_MAPPING_2D_GRID_2D_H_

#include <vector>

#include "cartographer/mapping/2d/map_limits.h"
#include "cartographer/mapping/grid_interface.h"
#include "cartographer/mapping/probability_values.h"
#include "cartographer/mapping/proto/grid_2d.pb.h"
#include "cartographer/mapping/proto/submap_visualization.pb.h"
#include "cartographer/mapping/proto/submaps_options_2d.pb.h"
#include "cartographer/mapping/value_conversion_tables.h"

namespace cartographer {
namespace mapping {

proto::GridOptions2D CreateGridOptions2D(
    common::LuaParameterDictionary* const parameter_dictionary);

enum class GridType { PROBABILITY_GRID, TSDF };

class Grid2D : public GridInterface
{
 public:
  Grid2D(const MapLimits& limits, 
   //栅格地图参数设置限制(const double resolution, 默认0.5cm
   //const Eigen::Vector2d& max, max_.x()和.y()分别表示x、y方向的最大值
   //const CellLimits& cell_limits)

         float min_correspondence_cost,                                     
         //z最小空闲概率

         float max_correspondence_cost,                                     
         //z最大空闲概率

         ValueConversionTables* conversion_tables);
  explicit Grid2D(const proto::Grid2D& proto,
                  ValueConversionTables* conversion_tables);

  // Returns the limits of this Grid2D.
   //返回全局限制参数 返回该栅格地图的范围、分辨率等。
  const MapLimits& limits() const                                          
  {   return limits_;  }                                        

  // Finishes the update sequence.
  void FinishUpdate();

  // Returns the correspondence cost of the cell with 'cell_index'.
   //返回一个坐标的空闲概率
  float GetCorrespondenceCost(const Eigen::Array2i& cell_index) const     
  {
    if (!limits().Contains(cell_index)) 
        return max_correspondence_cost_;
    return (*value_to_correspondence_cost_table_)[correspondence_cost_cells()[ToFlatIndex(cell_index)]];
  }

  virtual GridType GetGridType() const = 0;

  // Returns the minimum possible correspondence cost.
  float GetMinCorrespondenceCost() const { return min_correspondence_cost_; }

  // Returns the maximum possible correspondence cost.
  float GetMaxCorrespondenceCost() const { return max_correspondence_cost_; }

  // Returns true if the probability at the specified index is known.
  // 判断一个栅格位姿是否已经有相应的概率值
  bool IsKnown(const Eigen::Array2i& cell_index) const {                    
    return limits_.Contains(cell_index) &&
           correspondence_cost_cells_[ToFlatIndex(cell_index)] != kUnknownCorrespondenceValue;
  }

  // Fills in 'offset' and 'limits' to define a subregion of that contains all
  // known cells.
  // 进行一下裁剪。裁剪一个subregion，使得该subregion包含了所有的已有概率值的cells
  void ComputeCroppedLimits(Eigen::Array2i* const offset,                     
                            CellLimits* const limits) const;

  // Grows the map as necessary to include 'point'. This changes the meaning of
  // these coordinates going forward. This method must be called immediately
  // after 'FinishUpdate', before any calls to 'ApplyLookupTable'.
  virtual void GrowLimits(const Eigen::Vector2f& point);
  // 得到一个裁剪后的栅格图
  virtual std::unique_ptr<Grid2D> ComputeCroppedGrid() const = 0;             

  virtual proto::Grid2D ToProto() const;

  virtual bool DrawToSubmapTexture(
      proto::SubmapQuery::Response::SubmapTexture* const texture,
      transform::Rigid3d local_pose) const = 0;

 protected:
  void GrowLimits(const Eigen::Vector2f& point,
                  const std::vector<std::vector<uint16>*>& grids,
                  const std::vector<uint16>& grids_unknown_cell_values);
// 返回记录栅格地图空闲概率值的[1-32767]
  const std::vector<uint16>& correspondence_cost_cells() const {                
    return correspondence_cost_cells_;
  }
  const std::vector<int>& update_indices() const { return update_indices_; }
   // 返回一个已知概率值的区域。
  const Eigen::AlignedBox2i& known_cells_box() const {                         
    return known_cells_box_;
  }
// 返回记录栅格地图空闲概率值的指针
  std::vector<uint16>* mutable_correspondence_cost_cells() {                    
    return &correspondence_cost_cells_;
  }

  std::vector<int>* mutable_update_indices() { return &update_indices_; }
  Eigen::AlignedBox2i* mutable_known_cells_box() { return &known_cells_box_; }

  // Converts a 'cell_index' into an index into 'cells_'.
   //将pixel坐标再转化为序号的坐标
  int ToFlatIndex(const Eigen::Array2i& cell_index) const                      
  {                   
    CHECK(limits_.Contains(cell_index)) << cell_index;
    return limits_.cell_limits().num_x_cells * cell_index.y() + cell_index.x();
  }

 private:
  MapLimits limits_;
   //存储概率值，这里的概率值是Free的概率值 是栅格空闲的概率[1-32767]
  std::vector<uint16> correspondence_cost_cells_;                             
  float min_correspondence_cost_;
  float max_correspondence_cost_;
   //更新的索引
  std::vector<int> update_indices_;                                           

  // Bounding box of known cells to efficiently compute cropping limits.
    //已知边界框
  Eigen::AlignedBox2i known_cells_box_;                                      
  const std::vector<float>* value_to_correspondence_cost_table_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_GRID_2D_H_
