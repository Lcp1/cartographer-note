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

#include "cartographer/mapping/2d/submap_2d.h"

#include <cinttypes>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <limits>

#include "Eigen/Geometry"
#include "absl/memory/memory.h"
#include "cartographer/common/port.h"
#include "cartographer/mapping/2d/probability_grid_range_data_inserter_2d.h"
#include "cartographer/mapping/internal/2d/tsdf_range_data_inserter_2d.h"
#include "cartographer/mapping/range_data_inserter_interface.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
  // ----------------------------------CreateSubmapsOptions2D----------------------------------------------
//创建子图,定义参数 /src/cartographer/configuration_files/trajectory_builder_2d.lua
proto::SubmapsOptions2D CreateSubmapsOptions2D(
              common::LuaParameterDictionary* const parameter_dictionary)       
               //参数配置         
{
  proto::SubmapsOptions2D options;                                               
  //子图参数数据流
  options.set_num_range_data( parameter_dictionary->GetNonNegativeInt("num_range_data"));
  //读取激光点云数量看0 
  *options.mutable_grid_options_2d() =                                       
      CreateGridOptions2D( parameter_dictionary->GetDictionary("grid_options_2d").get());
      //栅格参数 栅格类型 分辨率
      //grid_type = "PROBABILITY_GRID",resolution = 0.05,
  *options.mutable_range_data_inserter_options() =
      CreateRangeDataInserterOptions(parameter_dictionary->GetDictionary("range_data_inserter").get());
      // 类型 hit miss
      //range_data_inserter_type = 
      //           "PROBABILITY_GRID_INSERTER_2D",
      // probability_grid_range_data_inserter = {
         //           insert_free_space = true,
          //           hit_probability = 0.55,
          //           miss_probability = 0.49,}
  bool valid_range_data_inserter_grid_combination = false;                      
  //激光插入器网格组合范围有效
  const proto::GridOptions2D_GridType& grid_type = options.grid_options_2d().grid_type();
  //栅格参数网格类型
  const proto::RangeDataInserterOptions_RangeDataInserterType&
      range_data_inserter_type = options.range_data_inserter_options().range_data_inserter_type();
      //激光数据插入类型
  if (grid_type == proto::GridOptions2D::PROBABILITY_GRID &&                    
      range_data_inserter_type == proto::RangeDataInserterOptions::PROBABILITY_GRID_INSERTER_2D) 
      valid_range_data_inserter_grid_combination = true;
      //如果概率栅格类型为:PROBABILITY_GRID
      //插入激光数据类型PROBABILITY_GRID_INSERTER_2D
      //就判定插入器网格组合范围有效
  if (grid_type == proto::GridOptions2D::TSDF &&                                
      range_data_inserter_type == proto::RangeDataInserterOptions::TSDF_INSERTER_2D)
      valid_range_data_inserter_grid_combination = true;
      //如果概率栅格类型为:TSDF覆盖栅格
      //插入激光数据类型TSDF_INSERTER_2D
      //就判定插入器网格组合范围有效
  CHECK(valid_range_data_inserter_grid_combination)                             
  //检查插入网格数据是否有效
      << "Invalid combination grid_type " << grid_type
      << " with range_data_inserter_type " << range_data_inserter_type;
  CHECK_GT(options.num_range_data(), 0);                                        
  // 子图激光帧数量是否等于0
  return options;
}
  // ----------------------------------CreateSubmapsOptions2D----------------------------------------------

  // ----------------------------------Submap2D----------------------------------------------
  //设置初始位姿
Submap2D::Submap2D(const Eigen::Vector2f& origin,                               
//初始位姿  
                   std::unique_ptr<Grid2D> grid,
                   ValueConversionTables* conversion_tables) 
                   //参数初始化
    : Submap(transform::Rigid3d::Translation(                                   
      //初始化变换矩阵
              Eigen::Vector3d(origin.x(),origin.y(), 0.))),                      
      conversion_tables_(conversion_tables)                                     
      //转换表
      // 浮点数与到uint16转换表格，估计用于概率图转换成整型进行计算
// 其中conversion_tables_表示0~32767的整型数对应的概率值，由于其概率上边界和下边界都是预配置的，
// 因此其对应关系为固定，可以提前计算好所有0-32767所有value对应的概率，后续使用时直接查表即可，提高运行速度。
{
  grid_ = std::move(grid);
}
  // ----------------------------------Submap2D----------------------------------------------

  // ----------------------------------Submap2D----------------------------------------------
Submap2D::Submap2D(const proto::Submap2D& proto, ValueConversionTables* conversion_tables)
//初始化
    : Submap(transform::ToRigid3(proto.local_pose())),                         
    //子图proto流局部位姿
      conversion_tables_(conversion_tables)                                     
      //转换表
{
  if (proto.has_grid())                                                         
  //判断proto是否有栅格,
  {
    if (proto.grid().has_probability_grid_2d())                                  
    //判断是不是概率覆盖地图,是的就创建 概率 grid_
    {
      grid_ =  absl::make_unique<ProbabilityGrid>(proto.grid(), conversion_tables_);
    } 
    else if (proto.grid().has_tsdf_2d())                                         
    //如果是TSDF地图,就创建 TSDF grid_
    {
      grid_ = absl::make_unique<TSDF2D>(proto.grid(), conversion_tables_);
    } 
    else 
    {
      LOG(FATAL) << "proto::Submap2D has grid with unknown type.";
    }
  }
  set_num_range_data(proto.num_range_data());                                    
  //将局部激光数量变量 赋值到 全局变量 
  set_insertion_finished(proto.finished());                                      
  //设置该子图插入 全局已经完成状态inished_
}
  // ----------------------------------Submap2D----------------------------------------------

  // ----------------------------------ToProto----------------------------------------------
//如果有栅格数据 ,就将栅格数据转到 Proto 内:proto.mutable_submap_2d()->mutable_grid()
proto::Submap Submap2D::ToProto(const bool include_grid_data) const 
{
  proto::Submap proto;                                                           
  //创建栅格proto 对象
  auto* const submap_2d = proto.mutable_submap_2d();                             
  //submap_2d指针 指向proto.的子图易变部分
  *submap_2d->mutable_local_pose() = transform::ToProto(local_pose());          
   //将本地位姿转proto类型
  submap_2d->set_num_range_data(num_range_data());                               
  //将局部激光数量变量 赋值到 全局变量 
  submap_2d->set_finished(insertion_finished());                                 
  //设置该子图插入 全局已经完成状态inished_
  if (include_grid_data) {
    CHECK(grid_);
    *submap_2d->mutable_grid() = grid_->ToProto();                                
    //调用grid_中的ToProto函数把概率图保存到proto中
  }
  return proto;
}
  // -----------------------------------ToProto----------------------------------------------

  // ----------------------------------UpdateFromProto--------------------------------------------
  //从proto读取子图栅格数据
void Submap2D::UpdateFromProto(const proto::Submap& proto) 
{
  CHECK(proto.has_submap_2d());
  const auto& submap_2d = proto.submap_2d();
  set_num_range_data(submap_2d.num_range_data());                                
  //将局部激光数量变量 赋值到 全局变量    
  set_insertion_finished(submap_2d.finished());                                  
  //设置该子图插入 全局已经完成状态inished_
  if (proto.submap_2d().has_grid())                                              
  //如果有栅格数据,就判断是 覆盖概率地图 还是 TSDF地图
  {                                                                              
    //并依据判断读取出来到 全局变量 grid_ 
    if (proto.submap_2d().grid().has_probability_grid_2d())                      
    //是概率栅格地图
    {
      grid_ = absl::make_unique<ProbabilityGrid>(proto.submap_2d().grid(),  
                                                 conversion_tables_);
    } 
    else if (proto.submap_2d().grid().has_tsdf_2d())                             
     //是TSDF栅格地图
    {
      grid_ = absl::make_unique<TSDF2D>(proto.submap_2d().grid(),
                                        conversion_tables_);
    } else {
      LOG(FATAL) << "proto::Submap2D has grid with unknown type.";                
      //否则报未知类型
    }
  }
}
  // ----------------------------------UpdateFromProto--------------------------------------------

  // ----------------------------------ToResponseProto--------------------------------------------
  //回应 子图 proto
void Submap2D::ToResponseProto(
    const transform::Rigid3d&,
    proto::SubmapQuery::Response* const response) const {
  if (!grid_) return;
  response->set_submap_version(num_range_data());                                 
  // 回应该子图激光数量
  proto::SubmapQuery::Response::SubmapTexture* const texture =response->add_textures();
  grid()->DrawToSubmapTexture(texture, local_pose());                             
  //根据是栅格地图概率 还是 TSDF 栅格转换到 子图文本格式
}
 // ----------------------------------ToResponseProto--------------------------------------------

 // ----------------------------------InsertRangeData--------------------------------------------
// 给激光数据容器插入数据
void Submap2D::InsertRangeData(
    const sensor::RangeData& range_data,
    const RangeDataInserterInterface* range_data_inserter) 
{
  CHECK(grid_);                                                                   
  //检查是否栅格化
  CHECK(!insertion_finished());                                                   
  //检查图是否已被finished        
  range_data_inserter->Insert(range_data, grid_.get());                           
  //插入激光数据  
  set_num_range_data(num_range_data() + 1);                                       
  //将局部激光数量变量加一并 赋值到 全局变量
}
// ----------------------------------InsertRangeData--------------------------------------------

// ----------------------------------Finish() --------------------------------------------
//计算裁剪栅格

void Submap2D::Finish() {
  CHECK(grid_);
  CHECK(!insertion_finished());
  grid_ = grid_->ComputeCroppedGrid();                                             
  //计算裁剪栅格
  set_insertion_finished(true);                                                   
   //设置 全局finished_状态设置已经完成
}
// ----------------------------------Finish() --------------------------------------------

// ----------------------------------ActiveSubmaps2D --------------------------------------------
//参数配置
ActiveSubmaps2D::ActiveSubmaps2D(const proto::SubmapsOptions2D& options)
    : options_(options), 
    range_data_inserter_(CreateRangeDataInserter()) 
    {}
// ----------------------------------ActiveSubmaps2D--------------------------------------------

// ----------------------------------submaps()  --------------------------------------------
//定义活动子图 是读取全局变量 submaps_
std::vector<std::shared_ptr<const Submap2D>> ActiveSubmaps2D::submaps() const 
{
  return std::vector<std::shared_ptr<const Submap2D>>(submaps_.begin(),
                                                      submaps_.end());
}

// ----------------------------------submaps()  --------------------------------------------

// ----------------------------------InsertRangeData --------------------------------------------
//活动子图插入数据
// submaps_列表实际最多只两个submap，一个认为是old_map，另一个认为是new_map，
std::vector<std::shared_ptr<const Submap2D>> ActiveSubmaps2D::InsertRangeData(
    const sensor::RangeData& range_data) 
{
  // 如果第一次，即无任何submap2d时
  // 或者如果new的submap的内部含有的激光个数达到 配置的阈值
  // 则需要添加新的子图,激光数据的原始坐标为新的子图初始位姿
   // 注意这是在插入前先进行了判断，也就说上次循环已经满足阈值条件
  if (submaps_.empty() || submaps_.back()->num_range_data() == options_.num_range_data()) 
      AddSubmap(range_data.origin.head<2>());

  //型子图和旧子图同时插入新的帧,表明旧的子图同时包含新子图的激光帧
  for (auto& submap : submaps_)  //submaps_ <=2  old 和new map
      submap->InsertRangeData(range_data, range_data_inserter_.get());

  // 如果旧的子图达到配置阈值的两倍 就表明新的子图插入激光帧已经到达阈值
  // 则 将old的submap进行结束封装，表明submap结束，设置submap2d 结束标志位，
  // 同时也进行裁剪仅保留有效value的边界
  if (submaps_.front()->num_range_data() == 2 * options_.num_range_data()) 
    submaps_.front()->Finish();

  // 返回新的子图和旧的子图
  return submaps();

}

// ----------------------------------InsertRangeData --------------------------------------------


// ----------------------------------CreateRangeDataInserter() -------------------------------------------
//根据地图类型创建 激光数据插入器
std::unique_ptr<RangeDataInserterInterface>ActiveSubmaps2D::CreateRangeDataInserter() 
{
  switch (options_.range_data_inserter_options().range_data_inserter_type()) 
  {
    case proto::RangeDataInserterOptions::PROBABILITY_GRID_INSERTER_2D:              
    //概率
       return absl::make_unique<ProbabilityGridRangeDataInserter2D>(
          options_.range_data_inserter_options()
              .probability_grid_range_data_inserter_options_2d());
    case proto::RangeDataInserterOptions::TSDF_INSERTER_2D:                          
     //TSDF
       return absl::make_unique<TSDFRangeDataInserter2D>(
          options_.range_data_inserter_options()
              .tsdf_range_data_inserter_options_2d());
    default:
      LOG(FATAL) << "Unknown RangeDataInserterType.";
  }
}
// ----------------------------------CreateRangeDataInserter() -------------------------------------------

// ----------------------------------CreateGrid( -------------------------------------------
//根据栅格类型 创建活动 栅格  
std::unique_ptr<GridInterface> ActiveSubmaps2D::CreateGrid(
    const Eigen::Vector2f& origin)                                           
    //原始位姿
{
  constexpr int kInitialSubmapSize = 100;                                    
  //初始子图空间大小
  float resolution = options_.grid_options_2d().resolution();
  switch (options_.grid_options_2d().grid_type()) 
  {
    case proto::GridOptions2D::PROBABILITY_GRID:                             
    //概率
      return absl::make_unique<ProbabilityGrid>(
          MapLimits(resolution,                                              
          //分辨率
                    origin.cast<double>() + 0.5 * kInitialSubmapSize * resolution * Eigen::Vector2d::Ones(),
                    //最远坐标
                    CellLimits(kInitialSubmapSize,kInitialSubmapSize)),       
                    //子图数量  
          &conversion_tables_);
    case proto::GridOptions2D::TSDF:                                         
     //TSDF
      return absl::make_unique<TSDF2D>(
          MapLimits(resolution,
                    origin.cast<double>() + 0.5 * kInitialSubmapSize * resolution *  Eigen::Vector2d::Ones(),
                    CellLimits(kInitialSubmapSize,  kInitialSubmapSize)),
          options_.range_data_inserter_options()
              .tsdf_range_data_inserter_options_2d()
              .truncation_distance(),                                         
              //截止距离
          options_.range_data_inserter_options()
              .tsdf_range_data_inserter_options_2d()
              .maximum_weight(),                                              
              //权重        
          &conversion_tables_);                                               
          //转换表   
    default:
      LOG(FATAL) << "Unknown GridType.";
  }
}
// ----------------------------------CreateGrid -------------------------------------------

// ----------------------------------AddSubmap -------------------------------------------
// 添加活动子图
void ActiveSubmaps2D::AddSubmap(const Eigen::Vector2f& origin)
{
  if (submaps_.size() >= 2)                                                   
  //如果子图数量超过2 ,说明存在old和new子图,就剪切掉old的子图  
  {
    // This will crop the finished Submap before inserting a new Submap to
    // reduce peak memory usage a bit.
    // 裁剪完成的old子贴图
    CHECK(submaps_.front()->insertion_finished()); 
    //剔除old子图,之前的新子图变成旧子图
    submaps_.erase(submaps_.begin());                                         
    //清空
  }
  // 插入一个新的submap2d
  submaps_.push_back(absl::make_unique<Submap2D>(
      origin,                                                                
      //初始位姿

      std::unique_ptr<Grid2D>( static_cast<Grid2D*>(CreateGrid(origin).release())),
      //栅格地图指针
      
      &conversion_tables_));                                                 
      //转换表
}

}  // namespace mapping
}  // namespace cartographer
