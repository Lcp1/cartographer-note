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

#include "absl/memory/memory.h"
#include "cartographer/mapping/map_builder.h"
#include "cartographer_ros/node.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/ros_log_sink.h"
#include "gflags/gflags.h"
#include "tf2_ros/transform_listener.h"

DEFINE_bool(collect_metrics, false,
            "Activates the collection of runtime metrics. If activated, the "
            "metrics can be accessed via a ROS service.");
DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");
DEFINE_string(load_state_filename, "",
              "If non-empty, filename of a .pbstream file to load, containing "
              "a saved SLAM state.");
DEFINE_bool(load_frozen_state, true,
            "Load the saved state as frozen (non-optimized) trajectories.");
DEFINE_bool(
    start_trajectory_with_default_topics, true,
    "Enable to immediately start the first trajectory with default topics.");
DEFINE_string(
    save_state_filename, "",
    "If non-empty, serialize state and write it to disk before shutting down.");

namespace cartographer_ros {
namespace {

void Run() {
  constexpr double kTfBufferCacheTimeInSeconds = 10.;
  tf2_ros::Buffer tf_buffer{::ros::Duration(kTfBufferCacheTimeInSeconds)};//设置tf缓存时间
  tf2_ros::TransformListener tf(tf_buffer);//tf监听器
  //该struct中包含了对一些基本参数的设置，比如接收tf的timeout时间设置、子图发布周期设置等
  NodeOptions node_options;
  //在/cartographer_ros/cartographer_ros/trajectory_options.h文件参数设置，在lua文件内设置
  TrajectoryOptions trajectory_options;
  std::tie(node_options, trajectory_options) =//读取文件设置，读取历史数据包？？？？？？？？？？？这些名字都不懂
      LoadOptions(FLAGS_configuration_directory, FLAGS_configuration_basename);

  auto map_builder =//利用node_options.map_builder_options构造map_builder类
      cartographer::mapping::CreateMapBuilder(node_options.map_builder_options);
  Node node(node_options, std::move(map_builder), &tf_buffer,
            FLAGS_collect_metrics);//创建了一个大类，从这里进入定位系统
  if (!FLAGS_load_state_filename.empty()) {//判断是否要加载历史地图？？？？？？？？？？？这些名字都不懂
    node.LoadState(FLAGS_load_state_filename, FLAGS_load_frozen_state);
  }

  if (FLAGS_start_trajectory_with_default_topics) {//判断是否需要使用默认话题，关系到在launch文件设置remap话题
    node.StartTrajectoryWithDefaultTopics(trajectory_options);
  }

  ::ros::spin();//将会进入循环， 一直调用回调函数chatterCallback(),

  node.FinishAllTrajectories();//结束所有轨迹
  node.RunFinalOptimization();//最后进行一次优化

  if (!FLAGS_save_state_filename.empty()) {//判断是否需要保存地图
    node.SerializeState(FLAGS_save_state_filename,
                        true /* include_unfinished_submaps */);
  }
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(!FLAGS_configuration_directory.empty())//如果配置连接里面内容为空，则输出配置丢失，但我找不到变量定义位置
      << "-configuration_directory is missing.";
  CHECK(!FLAGS_configuration_basename.empty())
      << "-configuration_basename is missing.";

  ::ros::init(argc, argv, "cartographer_node");//创建node节点
  ::ros::start();

  cartographer_ros::ScopedRosLogSink ros_log_sink;//ros打印日志
  cartographer_ros::Run();
  ::ros::shutdown();
}
