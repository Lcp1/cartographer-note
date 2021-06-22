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

#include "cartographer/mapping/internal/2d/scan_matching/real_time_correlative_scan_matcher_2d.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>

#include "Eigen/Geometry"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/math.h"
#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/mapping/internal/2d/tsdf_2d.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {
namespace {
//计算再TSDF栅格地图的得分
float ComputeCandidateScore(const TSDF2D& tsdf,
                            const DiscreteScan2D& discrete_scan,
                            int x_index_offset, 
                            int y_index_offset) 
{
  float candidate_score = 0.f;
  float summed_weight = 0.f;
  for (const Eigen::Array2i& xy_index : discrete_scan) 
  {
    const Eigen::Array2i proposed_xy_index(xy_index.x() + x_index_offset,
                                           xy_index.y() + y_index_offset);
    const std::pair<float, float> tsd_and_weight =  
                        tsdf.GetTSDAndWeight(proposed_xy_index);
    const float normalized_tsd_score =
                        (tsdf.GetMaxCorrespondenceCost() - std::abs(tsd_and_weight.first)) /
                        tsdf.GetMaxCorrespondenceCost();
    const float weight = tsd_and_weight.second;
    candidate_score += normalized_tsd_score * weight;
    summed_weight += weight;
  }
  if (summed_weight == 0.f) return 0.f;
  candidate_score /= summed_weight;
  CHECK_GE(candidate_score, 0.f);
  return candidate_score;
}

float ComputeCandidateScore(const ProbabilityGrid& probability_grid,
                            const DiscreteScan2D& discrete_scan,
                            int x_index_offset, int y_index_offset) {
  float candidate_score = 0.f;
  for (const Eigen::Array2i& xy_index : discrete_scan) {
    const Eigen::Array2i proposed_xy_index(xy_index.x() + x_index_offset,
                                           xy_index.y() + y_index_offset);
    const float probability =
        probability_grid.GetProbability(proposed_xy_index);
    candidate_score += probability;
  }
  candidate_score /= static_cast<float>(discrete_scan.size());
  CHECK_GT(candidate_score, 0.f);
  return candidate_score;
}

}  // namespace

RealTimeCorrelativeScanMatcher2D::RealTimeCorrelativeScanMatcher2D(
    const proto::RealTimeCorrelativeScanMatcherOptions& options)
    : options_(options) {}

// 生成搜索栅格地图空间点容器
std::vector<Candidate2D>
RealTimeCorrelativeScanMatcher2D::GenerateExhaustiveSearchCandidates(
                                    const SearchParameters& search_parameters) const 
{
  int num_candidates = 0;
  for (int scan_index = 0; scan_index != search_parameters.num_scans;++scan_index) 
  {
    const int num_linear_x_candidates =//x轴搜索区间
                            (search_parameters.linear_bounds[scan_index].max_x -
                            search_parameters.linear_bounds[scan_index].min_x + 1);
    const int num_linear_y_candidates =//y轴搜索区间
                            (search_parameters.linear_bounds[scan_index].max_y -
                            search_parameters.linear_bounds[scan_index].min_y + 1);
    num_candidates += num_linear_x_candidates * num_linear_y_candidates;// 二维空间转一维,位置序号区间数值  多次循环累加值
  }
  std::vector<Candidate2D> candidates;
  candidates.reserve(num_candidates);//分配容器变量空间
  for (int scan_index = 0; scan_index != search_parameters.num_scans;
       ++scan_index) 
  {
    for (int x_index_offset = search_parameters.linear_bounds[scan_index].min_x;
         x_index_offset <= search_parameters.linear_bounds[scan_index].max_x;
         ++x_index_offset) 
    {
      for (int y_index_offset = search_parameters.linear_bounds[scan_index].min_y;
           y_index_offset <= search_parameters.linear_bounds[scan_index].max_y;
           ++y_index_offset) 
      {// 存储二维地图每一点
        candidates.emplace_back(scan_index, 
                                x_index_offset, 
                                y_index_offset,
                                search_parameters);
      }
    }
  }
  CHECK_EQ(candidates.size(), num_candidates);//校验数目是否一致
  return candidates;
}
// --------------------------------------------------------------

double RealTimeCorrelativeScanMatcher2D::Match(
    const transform::Rigid2d& initial_pose_estimate,
    const sensor::PointCloud& point_cloud, 
    const Grid2D& grid,
    transform::Rigid2d* pose_estimate) const 
{
  CHECK(pose_estimate != nullptr);//初始位姿不为空
  const Eigen::Rotation2Dd initial_rotation = initial_pose_estimate.rotation();//读取初始旋转矩阵
  const sensor::PointCloud rotated_point_cloud = sensor::TransformPointCloud(
                                            point_cloud,
                                            transform::Rigid3f::Rotation(Eigen::AngleAxisf(//绕Z轴旋转
                                                                                    initial_rotation.cast<float>().angle(), //转成float型
  //构建搜索参数                                                                                  Eigen::Vector3f::UnitZ())));            //关于Z轴的旋转
  const SearchParameters search_parameters(
                              options_.linear_search_window(),  //线性窗口
                              options_.angular_search_window(), //角度窗口
                              rotated_point_cloud,              //旋转点云
                              grid.limits().resolution());      //栅格分辨率
  // 根据搜索框生成多个旋转后的点云
  const std::vector<sensor::PointCloud> rotated_scans =
                    GenerateRotatedScans( rotated_point_cloud, 
                                          search_parameters);
  // 将每个点云加上平移translated_point后投影到网格中,//地图序列化离散点
  const std::vector<DiscreteScan2D> discrete_scans = DiscretizeScans(
                                grid.limits(), 
                                rotated_scans,
                                Eigen::Translation2f(initial_pose_estimate.translation().x(),
                                                    initial_pose_estimate.translation().y()));
  std::vector<Candidate2D> candidates =// 生成搜索空间点容器
                                 GenerateExhaustiveSearchCandidates(search_parameters);
  // 对候选解打分
  ScoreCandidates(grid,              //栅格
                  discrete_scans,    //离散点云
                  search_parameters,  // 搜索参数  
                  &candidates);       // 生成搜索栅格地图空间点容器

  const Candidate2D& best_candidate =
                            *std::max_element(candidates.begin(), candidates.end());
  *pose_estimate = transform::Rigid2d(
      {
       initial_pose_estimate.translation().x() + best_candidate.x,
       initial_pose_estimate.translation().y() + best_candidate.y
      },
      initial_rotation * Eigen::Rotation2Dd(best_candidate.orientation));
  return best_candidate.score;
}

void RealTimeCorrelativeScanMatcher2D::ScoreCandidates(
    const Grid2D& grid,                                 //栅格
    const std::vector<DiscreteScan2D>& discrete_scans,  //离散点(一维序号)
    const SearchParameters& search_parameters,          //搜索窗口参数
    std::vector<Candidate2D>* const candidates) const   //栅格地图空间候选点
{
  for (Candidate2D& candidate : *candidates) 
  {
    switch (grid.GetGridType()) //选择地图类型
    {
      case GridType::PROBABILITY_GRID://概率
        candidate.score = ComputeCandidateScore(
                                static_cast<const ProbabilityGrid&>(grid),
                                discrete_scans[candidate.scan_index], 
                                candidate.x_index_offset,
                                candidate.y_index_offset);
          break;
      case GridType::TSDF:   //TSDF
        candidate.score = ComputeCandidateScore(
                                static_cast<const TSDF2D&>(grid),
                                discrete_scans[candidate.scan_index], 
                                candidate.x_index_offset,
                                candidate.y_index_offset);
          break;
    }
    candidate.score *=
        std::exp(-common::Pow2(std::hypot(candidate.x, candidate.y) *
                                   options_.translation_delta_cost_weight() +
                               std::abs(candidate.orientation) *
                                   options_.rotation_delta_cost_weight()));
  }
}

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer
