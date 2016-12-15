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
#include <decision_manager/task.h>
#include <boost/shared_ptr.hpp>

#include <pluginlib/class_loader.h>
#include <std_msgs/Empty.h>
#include <string>
#include <vector>

class TaskContainerTEST : public decision_manager::TaskListener {
  typedef boost::shared_ptr<decision_manager::Task> TaskPtr;

 public:
  explicit TaskContainerTEST(ros::NodeHandle n)
    : nh_(n), task_loader_(kPluginPkgName_, kPluginBaseClassName_) {
    run_sub_ = nh_.subscribe<std_msgs::Empty>(kRunSubName_, 1,
      boost::bind(&TaskContainerTEST::RunCb, this, _1));
    stop_sub_ = nh_.subscribe<std_msgs::Empty>(kStopSubName_, 1,
      boost::bind(&TaskContainerTEST::StopCb, this, _1));
    try {
      task_handle_list_.push_back(
        task_loader_.createInstance(kPluginTaskClassName_));
      task_handle_list_.push_back(
        task_loader_.createInstance(kPluginTaskClassName2_));
    } catch(pluginlib::PluginlibException& ex) {
      ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());  // NOLINT
    }

    std::vector<TaskPtr>::const_iterator titer = task_handle_list_.begin();
    for (; titer != task_handle_list_.end(); ++titer) {
      (*titer)->Initialize(nh_, "null", false, false);
      ROS_INFO_STREAM((*titer)->GetTaskName() << ": is initialized.");
      (*titer)->AddTaskListener(decision_manager::TaskListenerPtr(this));
    }
  }
  void RunCb(const std_msgs::Empty::ConstPtr& et) {
    std::vector<TaskPtr>::const_iterator titer = task_handle_list_.begin();
    for (; titer != task_handle_list_.end(); ++titer) {
      ROS_INFO_STREAM((*titer)->GetTaskName() << ": run");
      (*titer)->Run();
    }
  }
  void StopCb(const std_msgs::Empty::ConstPtr& et) {
    std::vector<TaskPtr>::const_iterator titer = task_handle_list_.begin();
    for (; titer != task_handle_list_.end(); ++titer) {
      ROS_INFO_STREAM((*titer)->GetTaskName() << ": stop");
      (*titer)->Stop();
    }
  }
  virtual void OnTaskComplete(decision_manager::Task& task) {
  }
  virtual void OnTaskCancelled(decision_manager::Task& task) {
  }
  virtual void OnTaskFailed(decision_manager::Task& task) {
  }
  virtual void OnTaskStopped(decision_manager::Task& task) {
  }
  virtual void OnGoalEvent(decision_manager::Task& task,
                           decision_manager::TaskCommand& cmd) {
    ROS_INFO_STREAM(task.GetTaskName() << ": has been called");
  }

 private:
  ros::NodeHandle nh_;
  ros::Subscriber run_sub_;
  ros::Subscriber stop_sub_;
  pluginlib::ClassLoader<decision_manager::Task> task_loader_;
  std::vector<TaskPtr> task_handle_list_;
  static const std::string kPluginPkgName_;
  static const std::string kPluginBaseClassName_;
  static const std::string kPluginTaskClassName_;
  static const std::string kPluginTaskClassName2_;
  static const std::string kRunSubName_;
  static const std::string kStopSubName_;
};

const std::string TaskContainerTEST::kRunSubName_ = "task_run";
const std::string TaskContainerTEST::kStopSubName_ = "task_stop";
const std::string TaskContainerTEST::kPluginPkgName_ = "decision_manager";
const std::string TaskContainerTEST::kPluginBaseClassName_ =
  "decision_manager::Task";
const std::string TaskContainerTEST::kPluginTaskClassName_ =
  "decision_manager_plugin::GoalManager";
const std::string TaskContainerTEST::kPluginTaskClassName2_ =
  "decision_manager_plugin::FindingTool";

int main(int argc, char** argv) {
  ros::init(argc, argv, "task_container_test");
  ros::NodeHandle nh;
  TaskContainerTEST TCT(nh);
  ros::spin();
  return 0;
}
