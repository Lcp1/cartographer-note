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
#include "cartographer/mapping/2d/probability_grid.h"

#include <limits>

#include "absl/memory/memory.h"
#include "cartographer/mapping/probability_values.h"
#include "cartographer/mapping/submaps.h"

namespace cartographer {
namespace mapping {
// --------------------------------ProbabilityGrid--------------------------------------------------
//定义概率栅格
ProbabilityGrid::ProbabilityGrid(const MapLimits& limits,
                                 ValueConversionTables* conversion_tables)
       //初始化  Grid2D栅格                           
    : Grid2D(limits,                                                         
             kMinCorrespondenceCost,       
             kMaxCorrespondenceCost,
             conversion_tables),
      conversion_tables_(conversion_tables) 
      {}

//-------------------------------------ProbabilityGrid----------------------------------------------
//定义proto流概率栅格
ProbabilityGrid::ProbabilityGrid(const proto::Grid2D& proto,
                                 ValueConversionTables* conversion_tables)
                                : Grid2D(proto, conversion_tables),           
                                 // 定义proto流的概率栅格
                                  conversion_tables_(conversion_tables) 
{
  CHECK(proto.has_probability_grid_2d());                                     
   //检查有没有概率栅格数据
}

// Sets the probability of the cell at 'cell_index' to the given
// 'probability'. Only allowed if the cell was unknown before.
// -------------------------------------------------------------------------------
//  可用来设置一个给定局部map栅格坐标的概率值
void ProbabilityGrid::SetProbability(const Eigen::Array2i& cell_index,          
                                          //map地图坐标值
                                     const float probability) 
{
  uint16& cell =
      (*mutable_correspondence_cost_cells())[ToFlatIndex(cell_index)];          
       //通过指针函数 和坐标 查找出空闲概率值
  CHECK_EQ(cell, kUnknownProbabilityValue);                                     
   //检查是否为空
  cell = CorrespondenceCostToValue(ProbabilityToCorrespondenceCost(probability));
 // 先将占用概率转空闲概率 再用空闲概率 转[1,32767] 整数
 // 返回到cell内 并指向cell_index 坐标下的空闲概率
 // 边界规范 并且 转int  1-32766.f
  mutable_known_cells_box()->extend(cell_index.matrix());                         
  // 边界延申????
}

// Applies the 'odds' specified when calling ComputeLookupTableToApplyOdds()
// to the probability of the cell at 'cell_index' if the cell has not already
// been updated. Multiple updates of the same cell will be ignored until
// FinishUpdate() is called. Returns true if the cell was updated.
//
// If this is the first call to ApplyOdds() for the specified cell, its value
// will be set to probability corresponding to 'odds'.
// ----------------------------------ApplyLookupTable---------------------------------------
// 通过查表来更新栅格单元的占用概率的
bool ProbabilityGrid::ApplyLookupTable(const Eigen::Array2i& cell_index,
                                       const std::vector<uint16>& table) 
{
  DCHECK_EQ(table.size(), kUpdateMarker);                                       
  //判断表大小是否等于15
  const int flat_index = ToFlatIndex(cell_index);                               
  //将地图坐标位姿返回坐标索引序号
  uint16* cell = &(*mutable_correspondence_cost_cells())[flat_index];           
  //通过序号读取空闲概率
  if (*cell >= kUpdateMarker)                                                  
   /// 空闲概率 大于一单元 就返回false
  {
    return false;
  }
  mutable_update_indices()->push_back(flat_index);                              
  //并将位姿序号更新到全局位姿序号
  *cell = table[*cell];
  DCHECK_GE(*cell, kUpdateMarker);
  mutable_known_cells_box()->extend(cell_index.matrix());     
    //mutable_known_cells_box()是Grid2D的成员函数，返回存放已知概率值的一个子区域的盒子。
  // 现在就是把该cell放入已知概率值的盒子中                  
  //扩展已知网格边界
  return true;
}
// ----------------------------------GetGridType()---------------------------------------
// 获取栅格类型 覆盖概率栅格类型
GridType ProbabilityGrid::GetGridType() const 
{
  return GridType::PROBABILITY_GRID;
}
// ----------------------------------GetProbability---------------------------------------
// Returns the probability of the cell with 'cell_index'.
float ProbabilityGrid::GetProbability(const Eigen::Array2i& cell_index) const
{
  if (!limits().Contains(cell_index)) return kMinProbability;
  return CorrespondenceCostToProbability(
    ValueToCorrespondenceCost(
      correspondence_cost_cells()[ToFlatIndex(cell_index)])); 
//首先将二维坐标转序号 
 //根据序号查找该栅格的空闲概率[1-32767]
 //空闲概率[1-32767]转float空闲概率
 //再把空闲float型概率转占用概率
}

proto::Grid2D ProbabilityGrid::ToProto() const {
  proto::Grid2D result;
  result = Grid2D::ToProto();
  result.mutable_probability_grid_2d();
  return result;
}

std::unique_ptr<Grid2D> ProbabilityGrid::ComputeCroppedGrid() const {
  Eigen::Array2i offset;
  CellLimits cell_limits;
  ComputeCroppedLimits(&offset, &cell_limits);                                  
  //获取再全局地图的边界的起点边和终点
  const double resolution = limits().resolution();                              
  //边界 的分辨率
  const Eigen::Vector2d max =
      limits().max() - resolution * Eigen::Vector2d(offset.y(), offset.x());    
      //通过将边界值减去原点值,就可以求出最大坐标距离
  std::unique_ptr<ProbabilityGrid> cropped_grid =                               
  //创建概率对象
      absl::make_unique<ProbabilityGrid>(
          MapLimits(resolution, max, cell_limits), conversion_tables_);
  for (const Eigen::Array2i& xy_index : XYIndexRangeIterator(cell_limits)) 
  {
    if (!IsKnown(xy_index + offset)) continue;                                  
    //如果再limit范围内的点没查找到对应空闲概率,直接退出
    cropped_grid->SetProbability(xy_index, GetProbability(xy_index + offset));   
    //给剪切概率容器 的坐标位置设置空闲概率值
  }

  return std::unique_ptr<Grid2D>(cropped_grid.release());                         
  //release()是一个释放捕捉的函数
  //所以释放了指向刚刚设置有空闲概率的部分???????
}

bool ProbabilityGrid::DrawToSubmapTexture(
    proto::SubmapQuery::Response::SubmapTexture* const texture,
    transform::Rigid3d local_pose) const {
  Eigen::Array2i offset;
  CellLimits cell_limits;
  ComputeCroppedLimits(&offset, &cell_limits);

  std::string cells;
  for (const Eigen::Array2i& xy_index : XYIndexRangeIterator(cell_limits)) {
    if (!IsKnown(xy_index + offset)) {
      cells.push_back(0 /* unknown log odds value */);
      cells.push_back(0 /* alpha */);
      continue;
    }
    // We would like to add 'delta' but this is not possible using a value and
    // alpha. We use premultiplied alpha, so when 'delta' is positive we can
    // add it by setting 'alpha' to zero. If it is negative, we set 'value' to
    // zero, and use 'alpha' to subtract. This is only correct when the pixel
    // is currently white, so walls will look too gray. This should be hard to
    // detect visually for the user, though.
    const int delta =
        128 - ProbabilityToLogOddsInteger(GetProbability(xy_index + offset));
    const uint8 alpha = delta > 0 ? 0 : -delta;
    const uint8 value = delta > 0 ? delta : 0;
    cells.push_back(value);
    cells.push_back((value || alpha) ? alpha : 1);
  }

  common::FastGzipString(cells, texture->mutable_cells());
  texture->set_width(cell_limits.num_x_cells);
  texture->set_height(cell_limits.num_y_cells);
  const double resolution = limits().resolution();
  texture->set_resolution(resolution);
  const double max_x = limits().max().x() - resolution * offset.y();
  const double max_y = limits().max().y() - resolution * offset.x();
  *texture->mutable_slice_pose() = transform::ToProto(
      local_pose.inverse() *
      transform::Rigid3d::Translation(Eigen::Vector3d(max_x, max_y, 0.)));

  return true;
}

}  // namespace mapping
}  // namespace cartographer
