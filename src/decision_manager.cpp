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

#include <decision_manager/decision_manager.h>
#include <string>
#include <vector>

namespace decision_manager {

const std::string DecisionManager::kCmdSubName_ = "task_cmd";

DecisionManager::DecisionManager(ros::NodeHandle n)
  : nh_(n), task_container_(n) {
  web_cmd_sub_ = nh_.subscribe<std_msgs::String>(kCmdSubName_, 1,
    boost::bind(&DecisionManager::WebCmdCallback, this, _1));
  task_container_.SetTasksListener(TaskListenerPtr(this));
  tasks_map_ = task_container_.GetTasks();
  TaskMap::const_iterator titer = tasks_map_.begin();
  for (; titer != tasks_map_.end(); ++titer) {
    task_wait_list_.push_back(titer->first);
  }
}

void DecisionManager::WebCmdCallback(const std_msgs::String::ConstPtr& str) {
  std::vector<std::string> cmd = StringSplit(str->data, " ");
  ROS_INFO_STREAM("recived command: " << "\"" << str->data << "\"");
  TaskPtr task_ptr = task_container_.GetTask(cmd[0]);
  // TODO(FrankChen): Task state/depends check (for what??)
  if (task_ptr != NullPtr) {
    if (cmd[1] == kTaskCmdRun) {
      ROS_INFO_STREAM(task_ptr->GetTaskName() << ": run");
      task_executor_.PostTask(task_ptr, TASK_RUN);
    } else if (cmd[1] == kTaskCmdStop) {
      TaskStatus ts = task_ptr->GetTaskState();
      if (!ts.IsStoppable()) {
        ROS_WARN_STREAM(task_ptr->GetTaskName() << " cannot stop");
      } else {
        ROS_INFO_STREAM(task_ptr->GetTaskName() << ": stop");
        task_executor_.PostTask(task_ptr, TASK_STOP);
      }
    } else {
      ROS_INFO_STREAM(cmd[1]
        << " is a wrong command in task: " << task_ptr->GetTaskName());
    }
  } else {
    ROS_INFO_STREAM(cmd[0] << " is a wrong task name");
  }
}

void DecisionManager::DecisionMaking() {
}

void DecisionManager::OnTaskComplete(Task& task) {
}
void DecisionManager::OnTaskCancelled(Task& task) {
}
void DecisionManager::OnTaskFailed(Task& task) {
}
void DecisionManager::OnTaskStopped(Task& task) {
}
void DecisionManager::OnGoalEvent(Task& task, TaskCommand& cmd) {
  ROS_INFO_STREAM(task.GetTaskName() << ": has been called (listener hero)");
  ROS_INFO_STREAM(cmd.GetCommand() << ": has been set (listener command)");
  ROS_INFO_STREAM(cmd.GetTaskName() << ": has been called (listener sidekick)");
}

std::vector<std::string> DecisionManager::StringSplit(
  std::string str, std::string pattern) {
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

};  // namespace decision_manager
