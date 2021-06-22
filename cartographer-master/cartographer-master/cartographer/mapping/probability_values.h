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

#ifndef CARTOGRAPHER_MAPPING_PROBABILITY_VALUES_H_
#define CARTOGRAPHER_MAPPING_PROBABILITY_VALUES_H_

#include <cmath>
#include <vector>

#include "cartographer/common/math.h"
#include "cartographer/common/port.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

namespace {
//边界规范 并且 转int  1-32766.f
inline uint16 BoundedFloatToValue(const float float_value,
                                  const float lower_bound,
                                  const float upper_bound) {
  const int value =
      common::RoundToInt(                                                        
          (common::Clamp(float_value, lower_bound, upper_bound) - lower_bound) *
          (32766.f / (upper_bound - lower_bound))) + 1;
      //将在全局的范围空间 转局部范围空间 再将概率值转[1-32767]int范围.
  // DCHECK for performance.
  DCHECK_GE(value, 1);
  DCHECK_LE(value, 32767);
  return value;
}

}  // namespace
//求占用与空闲概率比值
inline float Odds(float probability) {
  return probability / (1.f - probability);   
}
  //求概率，即x=y/(1+y)
inline float ProbabilityFromOdds(const float odds) {
  return odds / (odds + 1.f);     
}
//空闲概率
inline float ProbabilityToCorrespondenceCost(const float probability) {
  return 1.f - probability;
}
//空闲概率转占用概率
inline float CorrespondenceCostToProbability(const float correspondence_cost) {
  return 1.f - correspondence_cost;
}

constexpr float kMinProbability = 0.1f; 
// 最小概率值为0.1
constexpr float kMaxProbability = 1.f - kMinProbability;
// 最大概率值为0.9
constexpr float kMinCorrespondenceCost = 1.f - kMaxProbability;
// 最小空闲概率值为0.1
constexpr float kMaxCorrespondenceCost = 1.f - kMinProbability;
// 最大空闲概率值为0.1
// Clamps probability to be in the range [kMinProbability, kMaxProbability].
inline float ClampProbability(const float probability) {
  return common::Clamp(probability, kMinProbability, kMaxProbability);
}
// Clamps correspondece cost to be in the range [kMinCorrespondenceCost,
// kMaxCorrespondenceCost].
/*
限制概率p在[0.1,0.9]之间
*/
inline float ClampCorrespondenceCost(const float correspondence_cost) {
  return common::Clamp(correspondence_cost, kMinCorrespondenceCost,
                       kMaxCorrespondenceCost);
}

constexpr uint16 kUnknownProbabilityValue = 0;
constexpr uint16 kUnknownCorrespondenceValue = kUnknownProbabilityValue;
constexpr uint16 kUpdateMarker = 1u << 15;    
// 左移的话kUpdateMarker是2的15次方：32768，概率值转化成整数value之后的最大范围。程序中有判断是否越界

// Converts a correspondence_cost to a uint16 in the [1, 32767] range.
// -----------------------------------CorrespondenceCostToValue--------------------------------------------
//边界规范 并且 转int  1-32766.f
// 将float类型的数据转换为uint16类型，并将空闲概率输入从区间[kMinCorrespondenceCost,kMaxCorrespondenceCost]
// 映射到[1,32767]。
inline uint16 CorrespondenceCostToValue(const float correspondence_cost) {
  return BoundedFloatToValue(correspondence_cost,
                             kMinCorrespondenceCost,
                             kMaxCorrespondenceCost);
}
// -----------------------------------ProbabilityToValue--------------------------------------------
// Converts a probability to a uint16 in the [1, 32767] range.
// 将占用概率映射到[1, 32767]
inline uint16 ProbabilityToValue(const float probability) {
  return BoundedFloatToValue(probability, kMinProbability, kMaxProbability);
}

extern const std::vector<float>* const kValueToProbability;
//声明value到占用概率的映射
extern const std::vector<float>* const kValueToCorrespondenceCost;
//声明value到空闲概率的映射
// -----------------------------------ValueToProbability--------------------------------------------
// Converts a uint16 (which may or may not have the update marker set) to a
// probability in the range [kMinProbability, kMaxProbability].
// 将[1, 32767]映射到probability 
inline float ValueToProbability(const uint16 value) {
  return (*kValueToProbability)[value];
}
// -----------------------------------VValueToCorrespondenceCost--------------------------------------------
// Converts a uint16 (which may or may not have the update marker set) to a
// correspondence cost in the range [kMinCorrespondenceCost,
// kMaxCorrespondenceCost].
// 将[1,32767] 转空闲概率
// 反映射 [1,32767]->[0.1,0.9]
inline float ValueToCorrespondenceCost(const uint16 value) {
  return (*kValueToCorrespondenceCost)[value];
}
// 对输入概率值取反，这意味着如果我们的输入描述的是栅格单元的占用概率，那么实际存储的则是栅格单元空闲的概率
//占用概率转空闲概率 取反
inline uint16 ProbabilityValueToCorrespondenceCostValue(
    uint16 probability_value) {
  if (probability_value == kUnknownProbabilityValue) {
    return kUnknownCorrespondenceValue;
  }
  bool update_carry = false;
  if (probability_value > kUpdateMarker) {
    probability_value -= kUpdateMarker;
    update_carry = true;
  }
  uint16 result = CorrespondenceCostToValue(
                   //第三步将空闲[0.1,0.9]转空闲数值[1,32767]
                     ProbabilityToCorrespondenceCost(
                      //  第二步将占用[0.1,0.9]转空闲[0.1,0.9]
                           ValueToProbability(probability_value)));
                          //  第一步将占用数值[1,32767]映射 到 占用[0.1,0.9]
  if (update_carry) result += kUpdateMarker;
  return result;
}
// --------------------------------CorrespondenceCostValueToProbabilityValue---------------------------------------
// 空闲概率数值 [1,32767]转占用概率数值[1,32767]
inline uint16 CorrespondenceCostValueToProbabilityValue(
    uint16 correspondence_cost_value) {
  if (correspondence_cost_value == kUnknownCorrespondenceValue)
    return kUnknownProbabilityValue;
    // 如果是Unknown值还返回unknown值。Probability和CorrespondenceCost的Unknown值都是0
  bool update_carry = false;
  if (correspondence_cost_value > kUpdateMarker) {
    // 左移的话kUpdateMarker是2的15次方：32768，概率值转化成整数value之后的最大范围。程序中有判断是否越界
    correspondence_cost_value -= kUpdateMarker;
    // 越界就减去32767
    update_carry = true;
  }
  uint16 result = ProbabilityToValue(
                        CorrespondenceCostToProbability(
                            ValueToCorrespondenceCost(correspondence_cost_value)));
  if (update_carry) result += kUpdateMarker;
  //原先减去过一个最大范围，现在再加回来
  return result;
}
// -----------------------------------------------------------------------
std::vector<uint16> ComputeLookupTableToApplyOdds(float odds);
std::vector<uint16> ComputeLookupTableToApplyCorrespondenceCostOdds(float odds);

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_PROBABILITY_VALUES_H_
