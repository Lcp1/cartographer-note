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
// 栅格地图中起点是限制坐标轴的中点 表示为offset
#include "cartographer/mapping/2d/grid_2d.h"

namespace cartographer {
namespace mapping {
namespace {
//-----------------------------------MinCorrespondenceCostFromProto-----------------------------------------------
// 计算proto的最小空闲概率
float MinCorrespondenceCostFromProto(const proto::Grid2D& proto) {
  if (proto.min_correspondence_cost() == 0.f &&
      proto.max_correspondence_cost() == 0.f)
  // 如果最大值最小值都为0,表明是就版本的proto
  {
    LOG(WARNING) << "proto::Grid2D: min_correspondence_cost "
                    "is initialized with 0 indicating an older version of the "
                    "protobuf format. Loading default values.";
    return kMinCorrespondenceCost;
  } else {
    return proto.min_correspondence_cost();
  }
}
//-----------------------------------MaxCorrespondenceCostFromProto-----------------------------------------------
// 计算proto的最大空闲概率
float MaxCorrespondenceCostFromProto(const proto::Grid2D& proto) {
  if (proto.min_correspondence_cost() == 0.f &&
      proto.max_correspondence_cost() == 0.f) {
    LOG(WARNING) << "proto::Grid2D: max_correspondence_cost "
                    "is initialized with 0 indicating an older version of the "
                    "protobuf format. Loading default values.";
    return kMaxCorrespondenceCost;
  }
  else 
  {
    return proto.max_correspondence_cost();
  }
}
}  // namespace
//-----------------------------------MCreateGridOptions2D -------------------------------------
//在parameter_dictionary内读取参数设置  创建栅格参数设置项
proto::GridOptions2D CreateGridOptions2D(
    common::LuaParameterDictionary* const parameter_dictionary) 
{
  proto::GridOptions2D options;
  const std::string grid_type_string = parameter_dictionary->GetString("grid_type");
  proto::GridOptions2D_GridType grid_type;
  CHECK(proto::GridOptions2D_GridType_Parse(grid_type_string, &grid_type))
                    << "Unknown GridOptions2D_GridType kind: " 
                    << grid_type_string;
  options.set_grid_type(grid_type);
  options.set_resolution(parameter_dictionary->GetDouble("resolution"));
  return options;
}
//-----------------------------------Grid2D -------------------------------------
// 定义Grid2D栅格
Grid2D::Grid2D(const MapLimits& limits,
               float min_correspondence_cost,
               float max_correspondence_cost,
               ValueConversionTables* conversion_tables)
    //初始化参数
    : limits_(limits),
      correspondence_cost_cells_(
          limits_.cell_limits().num_x_cells * limits_.cell_limits().num_y_cells,
          // 限制栅格网格数量
          kUnknownCorrespondenceValue),
      min_correspondence_cost_(min_correspondence_cost),                     
      //最小空闲概率赋值给全局变量
      max_correspondence_cost_(max_correspondence_cost),                     
       //最大空闲概率赋值给全局变量
      value_to_correspondence_cost_table_(conversion_tables->GetConversionTable(  max_correspondence_cost, 
                                                                                  min_correspondence_cost,
                                                                                  max_correspondence_cost)) 
{
  CHECK_LT(min_correspondence_cost_, max_correspondence_cost_);              
  //意为小于，函数判断是否x小于y，当x<y时，函数打印x<y。
}
//-----------------------------------Grid2D -------------------------------------
// 定义proto流的栅格
Grid2D::Grid2D(const proto::Grid2D& proto,
               ValueConversionTables* conversion_tables)
    : limits_(proto.limits()),
      correspondence_cost_cells_(),
      min_correspondence_cost_(MinCorrespondenceCostFromProto(proto)),
      max_correspondence_cost_(MaxCorrespondenceCostFromProto(proto)),
      value_to_correspondence_cost_table_(conversion_tables->GetConversionTable(
                                                      max_correspondence_cost_,
                                                      min_correspondence_cost_,
                                                      max_correspondence_cost_)) 
{
  CHECK_LT(min_correspondence_cost_, max_correspondence_cost_);              
  //如果最小值 小于 最大值 ,则打印 最小值小于最大值
  if (proto.has_known_cells_box())                                           
  //如果proto有数据
  {
    const auto& box = proto.known_cells_box();                               
    //读取proto的网格容器 发布到全局变量 是矩阵量
    known_cells_box_ =
        Eigen::AlignedBox2i(Eigen::Vector2i(box.min_x(), box.min_y()),
                            Eigen::Vector2i(box.max_x(), box.max_y()));
  }
  correspondence_cost_cells_.reserve(proto.cells_size());                     
  //存储网格数量                     是vector容器数量
  for (const auto& cell : proto.cells()) {
    CHECK_LE(cell, std::numeric_limits<uint16>::max());                       
     //判断网格是否在范围内
    correspondence_cost_cells_.push_back(cell);
  }
}

// Finishes the update sequence.????????????????????????????完成更新队列
void Grid2D::FinishUpdate() {
  while (!update_indices_.empty()) {
    DCHECK_GE(correspondence_cost_cells_[update_indices_.back()],
              kUpdateMarker);
    correspondence_cost_cells_[update_indices_.back()] -= kUpdateMarker;
    update_indices_.pop_back();
  }
}
// -------------------------------ComputeCroppedLimits(--------------------------------------------
// Fills in 'offset' and 'limits' to define a subregion of that contains all
// known cells.
// 计算裁剪限制limits
void Grid2D::ComputeCroppedLimits(Eigen::Array2i* const offset,
                                  CellLimits* const limits) const 
{
  if (known_cells_box_.isEmpty())                                          
  //如果网格数为空
  {
    *offset = Eigen::Array2i::Zero();
    *limits = CellLimits(1, 1);
    return;
  }
  *offset = known_cells_box_.min().array();                                 
  //如果不为空,设边界最小值为起点
  *limits = CellLimits(known_cells_box_.sizes().x() + 1,                    
  //将已知的网格数量加1设为边界限制
                       known_cells_box_.sizes().y() + 1);
}
// --------------------------------GrowLimits-------------------------------------------------------------------
// Grows the map as necessary to include 'point'. This changes the meaning of
// these coordinates going forward. This method must be called immediately
// after 'FinishUpdate', before any calls to 'ApplyLookupTable'.
// 增长极限
void Grid2D::GrowLimits(const Eigen::Vector2f& point) 
{
  GrowLimits(point, 
             {mutable_correspondence_cost_cells()},
             {kUnknownCorrespondenceValue});
}

// --------------------------------GrowLimits-------------------------------------------------------------------
// 重载
// --------------------------------GrowLimits-------------------------------------------------------------------
// grid2d作为基类，存储单元为uint16整型数，即0~65535，而不关心具体实际意义。其目的是方便代码复用，即建立不同的继承类。
// 可通过value_to_correspondence_cost_table_表格进行还原真实表示意义。
// 其中GrowLimits用于扩展此gridmap的大小，即当加入一个已知栅格状态时，需要判断并扩展当前gridmap。
// 栅格grids里面有 grids.size()个栅格 ,即是每个像素
// 每个像素有limits_.cell_limits().num_x_cell*limits_.cell_limits().num_y_cells个网格
// 输入:  point grids
// 输出: 更新 grids
void Grid2D::GrowLimits(const Eigen::Vector2f& point,                                 //点云坐标
                        const std::vector<std::vector<uint16>*>& grids,               //二维空间
                        const std::vector<uint16>& grids_unknown_cell_values)         //不清楚的概率值
{
  CHECK(update_indices_.empty());
  // /如果当前的存在point不在范围内，即需要更新，采用迭代方法放大地图边界，
  while (!limits_.Contains(
                   //检查给pixel坐标是否大于0，小于等于最大值
                    limits_.GetCellIndex(point)))
                    //将点云坐标往绝对值变大四舍五入法
                    //即向与中心方向相反的方向移动                       
   //GetCellIndex先给出一个point在Submap中的坐标，求其栅格坐标        #include "cartographer/mapping/2d/map_limits.h"
  // Contains 返回布尔型，判断所给pixel坐标是否大于0，小于等于最大值   #include "cartographer/mapping/2d/map_limits.h"
  {
    //获取原来的地图大小的中心坐标，即栅格索引
    const int x_offset = limits_.cell_limits().num_x_cells / 2;                 
    //地图中点 x        
    const int y_offset = limits_.cell_limits().num_y_cells / 2;                 
    //地图中点 y  
    // grid最大值更新原来的一半， 地图总大小放大一倍。 即从地图中心位置上下左右均放大原大小一半
    const MapLimits new_limits(
        limits_.resolution(),
        limits_.max() + limits_.resolution() * Eigen::Vector2d(y_offset, x_offset),    //最大范围值   
         //最大值矩阵
        CellLimits(2 * limits_.cell_limits().num_x_cells,                       
                   2 * limits_.cell_limits().num_y_cells));
                   //扩展一倍空间
    const int stride = new_limits.cell_limits().num_x_cells;
    //定义像素空间x轴限制值stride 行数,二维空间转一维索引
    const int offset = x_offset + stride * y_offset;
    //定义像素空间起始序号值offset
    const int new_size = new_limits.cell_limits().num_x_cells *
                         new_limits.cell_limits().num_y_cells;
    //定义像素容量大小new_size 
    // 给每个栅格网格赋值概率   即将原来的概率赋值在新的grid中
    for (size_t grid_index = 0; grid_index < grids.size(); ++grid_index) 
    {
      std::vector<uint16> new_cells(new_size,grids_unknown_cell_values[grid_index]);
      // 即是定义了了new_size个uint16类型的值,且给出每个元素的概率初值为grids_unknown_cell_values[grid_index]
      // 例子:vector<int> a(10,1); //定义了10个整型元素的向量,且给出每个元素的初值为1
      for (int i = 0; i < limits_.cell_limits().num_y_cells; ++i) 
      {
        //以y轴方向开始计数
        for (int j = 0; j < limits_.cell_limits().num_x_cells; ++j) 
        {
          //x轴表示为下一行
          new_cells[offset + j + i * stride] =
              (*grids[grid_index])[j + i * limits_.cell_limits().num_x_cells];
              //      栅格指针              网格序号
              //从offset序号值开始添加概率值到对应网格
        }
      }
      *grids[grid_index] = new_cells;     
      //将新增的像素添加到指定的序号的栅格地图
    }
    limits_ = new_limits;
    //获取新的像素限制范围   

    if (!known_cells_box_.isEmpty()) 
    // 重新计算有效栅格空间边界，即由于起点发生改变，则矩形框的坐标需进行转换
    {
      known_cells_box_.translate(Eigen::Vector2i(x_offset, y_offset));
      //通过将边界范围起点平移(x_offset, y_offset)量
    }
  }
}

// --------------------------------ToProto-------------------------------------------------------------------
//转proto内容 包括,box设置  栅格界限 空闲概率范围
proto::Grid2D Grid2D::ToProto() const 
{
  proto::Grid2D result;
  *result.mutable_limits() = mapping::ToProto(limits_);
 // 将栅格界限转proto流信息
  *result.mutable_cells() = {correspondence_cost_cells_.begin(),
                             correspondence_cost_cells_.end()};
  // 将空闲概率赋值到proto流的网格内
  CHECK(update_indices().empty()) << "Serializing a grid during an update is "
                                     "not supported. Finish the update first.";
//   已知概率值的区域如果不是空的读取范围,并存到result.mutable_known_cells_box()内
  if (!known_cells_box().isEmpty()) {
    auto* const box = result.mutable_known_cells_box();
    //通过box指针指向result.mutable_known_cells_box(),
    // 修改box即是修改result.mutable_known_cells_box()
    box->set_max_x(known_cells_box().max().x());
    box->set_max_y(known_cells_box().max().y());
    box->set_min_x(known_cells_box().min().x());
    box->set_min_y(known_cells_box().min().y());
    //重新设置边界区域范围
  }
  result.set_min_correspondence_cost(min_correspondence_cost_);
  // 读取空闲概率最大值最小值
  result.set_max_correspondence_cost(max_correspondence_cost_);
  return result;
}

}  // namespace mapping
}  // namespace cartographer
