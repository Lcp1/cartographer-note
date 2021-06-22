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

#include "cartographer_ros/node_constants.h"

#include "glog/logging.h"

namespace cartographer_ros {

std::vector<std::string> ComputeRepeatedTopicNames(const std::string& topic,
                                                   const int num_topics) {
  CHECK_GE(num_topics, 0);//EQ即great equation，意为“大于等于”，函数判断是否x大于等于y，当x>=y时，函数打印出x>=y。
                            // #define CHECK_EQ(x,y) CHECK_OP(x,y,EQ,==)
                            // #define CHECK_NE(x,y) CHECK_OP(x,y,NE,!=)
                            // #define CHECK_LE(x,y) CHECK_OP(x,y,LE,<=)
                            // #define CHECK_LT(x,y) CHECK_OP(x,y,LT,<)
                            // #define CHECK_GE(x,y) CHECK_OP(x,y,GE,>=)
                            // #define CHECK_GT(x,y) CHECK_OP(x,y,GT,>)

  if (num_topics == 1) {
    return {topic};
  }
  std::vector<std::string> topics;//定义字符串容器
  topics.reserve(num_topics); //reserve() 为容器预留足够的空间，避免不必要的重复分配。
                              //预留空间大于等于字符串的长度。
  for (int i = 0; i < num_topics; ++i) {
    topics.emplace_back(topic + "_" + std::to_string(i + 1));
  }
  return topics;
}

}  // namespace cartographer_ros
