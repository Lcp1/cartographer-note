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

#include "cartographer/mapping/2d/probability_grid_range_data_inserter_2d.h"

#include <cstdlib>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/mapping/2d/xy_index.h"
#include "cartographer/mapping/internal/2d/ray_to_pixel_mask.h"
#include "cartographer/mapping/probability_values.h"
#include "glog/logging.h"

// 执行步骤：
// 1.GrowAsNeeded实现当前grid map边界的扩展，即由于新的scan加入，可能会导致地图变大；
// 2.将地图分辨提高kSubpixelScale=1000倍，目的是为后面画直线精度更加精确；
// 3.获取高分辨率地图下的激光原点range_origin坐标索引;
// 4.获取高分率地图下所有有效激光点云的坐标索引；
// 5.获取还原原始地图分辨率坐标cell的value，然后查询hit_table表格进行更新，即z=hit条件下的更新；
// 6.采用RayToPixelMask画线的方法，获取激光原点到点云之间直线的所有点坐标；
// 7.通过还原原始地图分辨率获取value，查询miss_table表格进行更新，即z=miss条件下的更新；
 
namespace cartographer {
namespace mapping {
namespace {

// Factor for subpixel accuracy of start and end point for ray casts.
constexpr int kSubpixelScale = 1000;
// 将地图分辨提高kSubpixelScale=1000倍，目的是为后面画直线精度更加精确；
//根据激光数据位置,扩展边界区域范围
// 输入:range_data,
// 输出:probability_grid
// ---------------------------GrowAsNeeded----------------------------------------------
void GrowAsNeeded(const sensor::RangeData& range_data,
                  ProbabilityGrid* const probability_grid) 
{
  Eigen::AlignedBox2f bounding_box(range_data.origin.head<2>());
// 这个类代表由边界构成的区域

  // Padding around bounding box to avoid numerical issues at cell boundaries.
  // 在边界框周围填充，以避免单元格边界出现数字问题。
  constexpr float kPadding = 1e-6f;
  for (const sensor::RangefinderPoint& hit : range_data.returns) 
  {
    bounding_box.extend(hit.position.head<2>());
  //  extend() 扩展区域的边界，使得该区域包含输入的点。
  }
  for (const sensor::RangefinderPoint& miss : range_data.misses) 
  {
    bounding_box.extend(miss.position.head<2>());
      //  extend() 扩展区域的边界，使得该区域包含输入的点。
  }
  probability_grid->GrowLimits(bounding_box.min() -
                               kPadding * Eigen::Vector2f::Ones());

  probability_grid->GrowLimits(bounding_box.max() +
                               kPadding * Eigen::Vector2f::Ones());
}

// --------------------------CastRays-----------------------------------------------
// 计算hit 栅格， 同时计算传感器到到hit栅格经过的 miss栅格
// input :insert_free_space配置项，默认为true，表示需要更新miss情况的概率
// 输出:hit_table miss_table
void CastRays(const sensor::RangeData& range_data,
              const std::vector<uint16>& hit_table,
              const std::vector<uint16>& miss_table,
              const bool insert_free_space, ProbabilityGrid* probability_grid) 
{
  GrowAsNeeded(range_data, probability_grid);
  // 根据 新的激光数据 更新grid的边界大小
  const MapLimits& limits = probability_grid->limits();
  //获取栅格地图的limits
  const double superscaled_resolution = limits.resolution() / kSubpixelScale;
  // 将5cm分辨率 拆分成1000份 ,变成超细分辨率
  const MapLimits superscaled_limits(
                          superscaled_resolution, limits.max(),  //1分别获取分辨率 
                          CellLimits(limits.cell_limits().num_x_cells * kSubpixelScale,
                          // CellLimits返回:最大范围值二维,栅格数值 num_x_cells = 0; num_y_cells = 0;
                                    limits.cell_limits().num_y_cells * kSubpixelScale));                
  const Eigen::Array2i begin =
                          superscaled_limits.GetCellIndex(range_data.origin.head<2>());
  // 据RangeData原点的前两项(x,y)，获取其对应的栅格化坐标。该坐标是我们所求的射线的原点。
  // Compute and add the end points.
  std::vector<Eigen::Array2i> ends;
  // 定义一个向量集合，该集合存储RangeData中的hits的点。
  ends.reserve(range_data.returns.size());
  // 这里就是根据returns集合的大小，给ends预分配一块存储区
  for (const sensor::RangefinderPoint& hit : range_data.returns) 
  //一个一个读取hit集合
  {
    ends.push_back(superscaled_limits
                      //通过栅格坐标定义地图限制值
                        .GetCellIndex(hit.position.head<2>()));
                        // // 给出一个point在Submap中的坐标，求其栅格坐标
    probability_grid->ApplyLookupTable(ends.back() / kSubpixelScale, hit_table);
    // ens.back()返回的是vector中的最末尾项，也就是我们刚刚压入vector中的那一项；
    // 这里我猜测，hit_table就是预先计算好的。如果一个cell，原先的值是value，那么在检测到hit后应该更新为多少
  }

  if (!insert_free_space) 
    return;
   // 如果配置项里设置是不考虑free space。那么函数到这里结束，只处理完hit后返回即可
   // 否则的话，需要计算那条射线，射线中间的点都是free space，同时，没有检测到hit的misses集里也都是free
//  如下:

  // Now add the misses.
  for (const Eigen::Array2i& end : ends) 
  {//
    std::vector<Eigen::Array2i> ray =
        RayToPixelMask(begin, end, kSubpixelScale);
        //如果单元begin> end,就调换一下顺序 RayToPixelMask (end, begin,kSubpixelScale)
        //重复运行 ends.size()次
        // 6.采用RayToPixelMask画线的方法，获取激光原点到点云之间直线的所有点坐标；
    for (const Eigen::Array2i& cell_index : ray) 
    {
      probability_grid->ApplyLookupTable(cell_index, miss_table);
      //根据坐标cell_index,查询在表miss_table的占用概率
    }
  }

  // Finally, compute and add empty rays based on misses in the range data.
   // 更新所有 range中miss的点， 则整条光速直线均为miss更新
  for (const sensor::RangefinderPoint& missing_echo : range_data.misses) 
  {
    std::vector<Eigen::Array2i> ray = RayToPixelMask(
        begin, superscaled_limits.GetCellIndex(missing_echo.position.head<2>()),
        kSubpixelScale);
        range_data.misses
    for (const Eigen::Array2i& cell_index : ray) 
    {
      probability_grid->ApplyLookupTable(cell_index, miss_table);
        //根据坐标cell_index,查询在表miss_table的占用概率
    }
  }
}
}  // namespace

// -------------------------CreateProbabilityGridRangeDataInserterOptions2D--------------------
//读取参数设置
proto::ProbabilityGridRangeDataInserterOptions2D
CreateProbabilityGridRangeDataInserterOptions2D(
    common::LuaParameterDictionary* parameter_dictionary) 
{
  proto::ProbabilityGridRangeDataInserterOptions2D options;
  options.set_hit_probability(
      parameter_dictionary->GetDouble("hit_probability"));
  options.set_miss_probability(
      parameter_dictionary->GetDouble("miss_probability"));
  options.set_insert_free_space(
      parameter_dictionary->HasKey("insert_free_space")
          ? parameter_dictionary->GetBool("insert_free_space")
          : true);
  CHECK_GT(options.hit_probability(), 0.5);
  CHECK_LT(options.miss_probability(), 0.5);
  return options;
}

// --------------------ProbabilityGridRangeDataInserter2D------------------------
// 通过占用概率odd 转 空闲概率
ProbabilityGridRangeDataInserter2D::ProbabilityGridRangeDataInserter2D(
    const proto::ProbabilityGridRangeDataInserterOptions2D& options)
    : options_(options),
      hit_table_(ComputeLookupTableToApplyCorrespondenceCostOdds(
                  //计算hit空闲概率
                       Odds(options.hit_probability()))),
                       //转换为odd表示
      miss_table_(ComputeLookupTableToApplyCorrespondenceCostOdds(
                       Odds(options.miss_probability()))) {}

// ------------------------------Insert---------------------------------
void ProbabilityGridRangeDataInserter2D::Insert(
    const sensor::RangeData& range_data, GridInterface* const grid) const 
{
  // 将Grid类型强制转化为ProbabilityGrid类型
  ProbabilityGrid* const probability_grid = static_cast<ProbabilityGrid*>(grid);
  CHECK(probability_grid != nullptr);
  // By not finishing the update after hits are inserted, we give hits priority
  // (i.e. no hits will be ignored because of a miss in the same cell).
  // 采用画线法更新地图,计算出一条从原点到激光点的射线，射线端点处的点是Hit，射线中间的点是Free。
  // 把所有这些点要在地图上把相应的cell进行更新
//   struct RangeData {
//   Eigen::Vector3f origin;   //{x0,y0,z0},sensor坐标。
//   PointCloud returns;       //反射位置{x,y,z}，表征有物体反射。
//   PointCloud misses;        //无反射,自由空间
// };
  CastRays(range_data, //包含的一系列点 [h1,h2,h3...]
           hit_table_, 
           miss_table_, 
           options_.insert_free_space(),
           probability_grid);
  probability_grid->FinishUpdate();
}

}  // namespace mapping
}  // namespace cartographer
