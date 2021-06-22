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

#include "cartographer/mapping/probability_values.h"

#include "absl/memory/memory.h"

namespace cartographer {
namespace mapping {

namespace {

constexpr int kValueCount = 32768;
//----------------------------------------------------------------------------------
// 0 is unknown, [1, 32767] maps to [lower_bound, upper_bound].
// 通过在[0,1,2,...,32767, 0,1,2,...,32767]范围之间先算好对应的概率值,
// 存在PrecomputeValueToBoundedFloat返回的vector容器中,后期直接查表法就可以获取值
// 较一边计算速度快
// [1, 32767] 数值范围转边界 [0.1,0.9]float型
float SlowValueToBoundedFloat(const uint16 value, 
                              const uint16 unknown_value,//0
                              const float unknown_result,
                              const float lower_bound,
                              const float upper_bound) 
{
  CHECK_LT(value, kValueCount);
  //value是否小于32768
  if (value == unknown_value) return unknown_result;
  //value如果等于0 则返回0
  const float kScale = (upper_bound - lower_bound) / (kValueCount - 2.f);
  //间隔值 除以32768 
  return value * kScale + (lower_bound - kScale);
  // 已知value求在[upper_bound ,lower_bound]中的值

}
//----------------------------------------------------------------------------------
// 已知value求在[upper_bound ,lower_bound]中的值
// 先直接预算号后期查表法
std::unique_ptr<std::vector<float>> PrecomputeValueToBoundedFloat(
    const uint16 unknown_value, 
    const float unknown_result,
    const float lower_bound, 
    const float upper_bound) 
{
  auto result = absl::make_unique<std::vector<float>>();
  // 创建vector型指针
  // Repeat two times, so that both values with and without the update marker
  // can be converted to a probability.
  constexpr int kRepetitionCount = 2;
  result->reserve(kRepetitionCount * kValueCount);
  //将2*32768空间反转即[0,1,2,...,32767,
  //                   0,1,2,...,32767]
  for (int repeat = 0; repeat != kRepetitionCount; ++repeat)  
  // 不到2
  {
    for (int value = 0; value != kValueCount; ++value) 
    // 不到32768
    {
      result->push_back(SlowValueToBoundedFloat(
          value, 
          unknown_value,//0
          unknown_result, 
          lower_bound, 
          upper_bound));
          // 返回value求在[0.1,0.9]中对应的值
    }
  }
  return result;
}
//----------------------------------------------------------------------------------
std::unique_ptr<std::vector<float>> PrecomputeValueToProbability() {
  return PrecomputeValueToBoundedFloat(kUnknownProbabilityValue,//0
                                       kMinProbability, 
                                       kMinProbability,
                                       kMaxProbability);
}
//----------------------------------------------------------------------------------
// 调用PrecomputeValueToBoundedFloat与计算图表概率值
std::unique_ptr<std::vector<float>> PrecomputeValueToCorrespondenceCost() 
{
  return PrecomputeValueToBoundedFloat(
      kUnknownCorrespondenceValue, //0
      kMaxCorrespondenceCost,
      kMinCorrespondenceCost, 
      kMaxCorrespondenceCost);
}

}  // namespace
//----------------------------------------------------------------------------------
const std::vector<float>* const kValueToProbability =
    PrecomputeValueToProbability().release();
    // 返回占用概率值并赋值null释放空间

const std::vector<float>* const kValueToCorrespondenceCost =
    PrecomputeValueToCorrespondenceCost().release();
    // 返回空闲概率值并赋值null释放空间
 //----------------------------------------------------------------------------------   
std::vector<uint16> ComputeLookupTableToApplyOdds(const float odds) 
{
  std::vector<uint16> result;
  result.reserve(kValueCount);
  result.push_back(ProbabilityToValue(
                    //概率转数值[1-32768]
                    ProbabilityFromOdds(odds)) +    
                    //求概率，即x=y/(1+y
                   kUpdateMarker);                
                   //32768
  for (int cell = 1; cell != kValueCount; ++cell) {
    result.push_back(ProbabilityToValue(
                       ProbabilityFromOdds(
                         //通过观测更新后的odds求概率，即x=y/(1+y 
                         odds *Odds((*kValueToProbability)[cell]))) +
                         //上一次值     乘以 //观测:求占用与空闲概率比值
                         //  Odd(s|z)=Odd(s) * p(z|s=1)/p(z|s=1)
                     kUpdateMarker);
                     //327678
  }
  return result;
}
//----------------------------------------------------------------------------------
std::vector<uint16> ComputeLookupTableToApplyCorrespondenceCostOdds(
    float odds) 
{
  std::vector<uint16> result;
  result.reserve(kValueCount);
  result.push_back(CorrespondenceCostToValue(
                    //空闲概率转[1-32768]
                      ProbabilityToCorrespondenceCost(
                        //占用概率转空闲概率
                          ProbabilityFromOdds(odds))) +
                          //odds求占用概率
                   kUpdateMarker);
                  //  32768
  for (int cell = 1; cell != kValueCount; ++cell) {
    result.push_back(
        CorrespondenceCostToValue(
          //求[1-32768]数值
            ProbabilityToCorrespondenceCost(
              //求空闲概率
                ProbabilityFromOdds(
                  //求占用概率
                      odds * Odds(CorrespondenceCostToProbability(
                           (*kValueToCorrespondenceCost)[cell]))))) +
                      //Odd(s|z)
        kUpdateMarker);
  }
  return result;
}

}  // namespace mapping
}  // namespace cartographer
