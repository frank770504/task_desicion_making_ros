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
#include <decision_manager/task_executor.h>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/asio/io_service.hpp>

#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <string>
#include <vector>
#include <map>

// command: "task_name task_command"

class TaskExecutorTEST : public decision_manager::TaskListener {
  typedef boost::shared_ptr<decision_manager::Task> TaskPtr;
  typedef std::map<std::string, TaskPtr> TaskMap;

 public:
  explicit TaskExecutorTEST(ros::NodeHandle n)
    : nh_(n), task_container_(n) {
    run_sub_ = nh_.subscribe<std_msgs::Empty>(kRunSubName_, 1,
      boost::bind(&TaskExecutorTEST::RunCb, this, _1));
    stop_sub_ = nh_.subscribe<std_msgs::Empty>(kStopSubName_, 1,
      boost::bind(&TaskExecutorTEST::StopCb, this, _1));
    cmd_sub_ = nh_.subscribe<std_msgs::String>(kCmdSubName_, 1,
      boost::bind(&TaskExecutorTEST::CmdCb, this, _1));
    task_container_.SetTasksListener(decision_manager::TaskListenerPtr(this));
  }
  void RunCb(const std_msgs::Empty::ConstPtr& et) {
    TaskMap _map = task_container_.GetTasks();
    TaskMap::const_iterator titer = _map.begin();
    for (; titer != _map.end(); ++titer) {
      ROS_INFO_STREAM((titer->second)->GetTaskName() << ": run");
      task_executor_.PostTask((titer->second), decision_manager::TASK_RUN);
      //~ (titer->second)->Run();
    }
  }
  void StopCb(const std_msgs::Empty::ConstPtr& et) {
    TaskMap _map = task_container_.GetTasks();
    TaskMap::const_iterator titer = _map.begin();
    for (; titer != _map.end(); ++titer) {
      ROS_INFO_STREAM((titer->second)->GetTaskName() << ": stop");
      task_executor_.PostTask((titer->second), decision_manager::TASK_STOP);
      //~ (titer->second)->Stop();
    }
  }
  std::vector<std::string> StringSplit(std::string str, std::string pattern) {
    std::string::size_type pos;
    std::vector<std::string> result;
    str += pattern;
    int size = str.size();
    for (int i = 0; i < size; i++) {
      pos = str.find(pattern, i);
      if (pos < size) {
        std::string s = str.substr(i, pos - i);
        result.push_back(s);
        i = pos + pattern.size() - 1;
      }
    }
    return result;
  }
  void CmdCb(const std_msgs::String::ConstPtr& str) {
    std::vector<std::string> cmd = StringSplit(str->data, " ");
    ROS_INFO_STREAM("recived command: " << "\"" << str->data << "\"");
    decision_manager::TaskPtr task_ptr = task_container_.GetTask(cmd[0]);
    if (task_ptr != decision_manager::NullPtr) {
      if (cmd[1] == decision_manager::kTaskCmdRun) {
        ROS_INFO_STREAM(task_ptr->GetTaskName() << ": run using cmd");
        task_executor_.PostTask(task_ptr, decision_manager::TASK_RUN);
      } else if (cmd[1] == decision_manager::kTaskCmdStop) {
        ROS_INFO_STREAM(task_ptr->GetTaskName() << ": stop using cmd");
        task_executor_.PostTask(task_ptr, decision_manager::TASK_STOP);
      } else {
        ROS_INFO_STREAM(cmd[1]
          << " is a wrong command in task: " << task_ptr->GetTaskName());
      }
    } else {
      ROS_INFO_STREAM(cmd[0] << " is a wrong task name");
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
  virtual void OnGoalControl(decision_manager::Task& task) {
    ROS_INFO_STREAM(task.GetTaskName() << ": has been called");
  }

 private:
  ros::NodeHandle nh_;
  ros::Subscriber run_sub_;
  ros::Subscriber stop_sub_;
  ros::Subscriber cmd_sub_;
  static const std::string kRunSubName_;
  static const std::string kStopSubName_;
  static const std::string kCmdSubName_;
  decision_manager::TaskContainer task_container_;
  decision_manager::TaskExecutor task_executor_;
};

const std::string TaskExecutorTEST::kRunSubName_ = "task_run";
const std::string TaskExecutorTEST::kStopSubName_ = "task_stop";
const std::string TaskExecutorTEST::kCmdSubName_ = "task_cmd";

int main(int argc, char** argv) {
  ros::init(argc, argv, "task_container_usage_test");
  ros::NodeHandle nh;
  TaskExecutorTEST TCUT(nh);
  ros::spin();
  return 0;
}
