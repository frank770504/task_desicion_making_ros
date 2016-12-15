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
#include <decision_manager/task_container.h>
#include <boost/shared_ptr.hpp>

#include <pluginlib/class_loader.h>
#include <std_msgs/Empty.h>
#include <string>
#include <vector>
#include <map>

class TaskContainerUsageTEST : public decision_manager::TaskListener {
  typedef boost::shared_ptr<decision_manager::Task> TaskPtr;
  typedef std::map<std::string, TaskPtr> TaskMap;

 public:
  explicit TaskContainerUsageTEST(ros::NodeHandle n)
    : nh_(n), task_container_(n) {
    run_sub_ = nh_.subscribe<std_msgs::Empty>(kRunSubName_, 1,
      boost::bind(&TaskContainerUsageTEST::RunCb, this, _1));
    stop_sub_ = nh_.subscribe<std_msgs::Empty>(kStopSubName_, 1,
      boost::bind(&TaskContainerUsageTEST::StopCb, this, _1));
    task_container_.SetTasksListener(decision_manager::TaskListenerPtr(this));
  }
  void RunCb(const std_msgs::Empty::ConstPtr& et) {
    TaskMap _map = task_container_.GetTasks();
    TaskMap::const_iterator titer = _map.begin();
    for (; titer != _map.end(); ++titer) {
      ROS_INFO_STREAM((titer->second)->GetTaskName() << ": run");
      (titer->second)->Run();
    }
  }
  void StopCb(const std_msgs::Empty::ConstPtr& et) {
    TaskMap _map = task_container_.GetTasks();
    TaskMap::const_iterator titer = _map.begin();
    for (; titer != _map.end(); ++titer) {
      ROS_INFO_STREAM((titer->second)->GetTaskName() << ": stop");
      (titer->second)->Stop();
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
  virtual void OnGoalEvent(decision_manager::Task& task) {
    ROS_INFO_STREAM(task.GetTaskName() << ": has been called");
  }

 private:
  ros::NodeHandle nh_;
  ros::Subscriber run_sub_;
  ros::Subscriber stop_sub_;
  static const std::string kRunSubName_;
  static const std::string kStopSubName_;
  decision_manager::TaskContainer task_container_;
};

const std::string TaskContainerUsageTEST::kRunSubName_ = "task_run";
const std::string TaskContainerUsageTEST::kStopSubName_ = "task_stop";

int main(int argc, char** argv) {
  ros::init(argc, argv, "task_container_usage_test");
  ros::NodeHandle nh;
  TaskContainerUsageTEST TCUT(nh);
  ros::spin();
  return 0;
}
