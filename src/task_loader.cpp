/*
 * Copyright (c) 2016, Compal Electronics, INC.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of
 *    conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to
 *    endorse or promote products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <boost/shared_ptr.hpp>

#include <pluginlib/class_loader.h>
#include <std_msgs/Empty.h>
#include <decision_maneger/task.h>

typedef boost::shared_ptr<decision_manager::Task> TaskPtr;
TaskPtr task_handle;

void TestRun(const std_msgs::Empty::ConstPtr& et) {
  ROS_INFO_STREAM("run");
  task_handle->Run();
}

void TestStop(const std_msgs::Empty::ConstPtr& et) {
  ROS_INFO_STREAM("stop");
  task_handle->Stop();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "task_loader");
  ros::NodeHandle nh;
  ros::Subscriber run_sub = nh.subscribe("task_run", 1, TestRun);
  ros::Subscriber stop_sub = nh.subscribe("task_stop", 1, TestStop);
  pluginlib::ClassLoader<decision_manager::Task> task_loader(
    "decision_maneger", "decision_manager::Task");

  try {
    boost::shared_ptr<decision_manager::Task> test_task =
      task_loader.createInstance("decision_manager_plugin::GoalManager");
    test_task->Initialize(nh);
    task_handle.reset(test_task.get());
    ros::spin();
  } catch(pluginlib::PluginlibException& ex) {
    ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());  // NOLINT
  }

  return 0;
}
