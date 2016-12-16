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
  TaskPtr taskPtr = task_container_.GetTask(cmd[0]);
  TaskCommand taskCommand;
  if (taskPtr != NullPtr) {
    if (cmd[1] == kTaskCommandRun) {
      taskCommand.RunTask(taskPtr->GetTaskName());
    } else if (cmd[1] == kTaskCommandWait) {
      taskCommand.StopTask(taskPtr->GetTaskName());
    } else {
      ROS_INFO_STREAM(cmd[1]
        << " is a wrong command in task: " << taskPtr->GetTaskName());
    }
  } else {
    ROS_INFO_STREAM(cmd[0] << " is a wrong task name");
  }
  DecisionMaking(taskCommand, taskPtr);
}

void DecisionManager::DecisionListChecking(const TaskPtr& taskPtr) {
ROS_INFO_STREAM(__FUNCTION__ << " ================================");
  ROS_INFO_STREAM(taskPtr->GetTaskStatus()
    << " is the status of task: " << taskPtr->GetTaskName());
  std::vector<std::string>::iterator exec_it;
  std::vector<std::string>::iterator wait_it;
  exec_it = std::find(
    task_exec_list_.begin(), task_exec_list_.end(), taskPtr->GetTaskName());
  wait_it = std::find(
    task_wait_list_.begin(), task_wait_list_.end(), taskPtr->GetTaskName());
  if (taskPtr->GetTaskStatus() == kTaskStatusRun) {
    if (exec_it == task_exec_list_.end()) {
      // if not in task_exec_list_ then join in
      task_exec_list_.push_back(taskPtr->GetTaskName());
    }
    if (wait_it != task_wait_list_.end()) {
      // if in task_wait_list_ then remove out
      task_wait_list_.erase(wait_it);
    }
  } else if (taskPtr->GetTaskStatus() == kTaskStatusStop) {
    if (exec_it != task_exec_list_.end()) {
      // if in task_exec_list_ then remove outs
      task_exec_list_.erase(exec_it);
    }
    if (wait_it == task_wait_list_.end()) {
      // if not in task_wait_list_ then join in
      task_wait_list_.push_back(taskPtr->GetTaskName());
    }
  } else {
    ROS_INFO_STREAM(taskPtr->GetTaskStatus()
      << " is a wrong status of task: " << taskPtr->GetTaskName());
  }
  ROS_INFO_STREAM("task_exec_list_, after manuplating task: "
                  << taskPtr->GetTaskName());
  for (int i = 0; i < task_exec_list_.size(); i++) {
    ROS_INFO_STREAM(task_exec_list_[i]);
  }
  ROS_INFO_STREAM("task_wait_list_, after manuplating task: "
                  << taskPtr->GetTaskName());
  for (int i = 0; i < task_wait_list_.size(); i++) {
    ROS_INFO_STREAM(task_wait_list_[i]);
  }
ROS_INFO_STREAM("=====================================");
}

void DecisionManager::DecisionMaking(TaskCommand& cmd, const TaskPtr& task_ptr) {
  TaskPtr taskPtr = task_ptr;
  if (taskPtr == NullPtr) {
    taskPtr = task_container_.GetTask(cmd.GetTaskName());
  }
  if (cmd.GetCommand() == kTaskCommandRun) {
    ROS_INFO_STREAM(taskPtr->GetTaskName() << ": run");
    task_executor_.PostTask(taskPtr, TASK_RUN);  // 1: two lines group
    taskPtr->SetTaskStatus(kTaskStatusRun);  // 2: two lines group
    DecisionListChecking(taskPtr);
  } else if (cmd.GetCommand() == kTaskCommandWait) {
    TaskStatus ts = taskPtr->GetTaskState();
    if (!ts.IsStoppable()) {
      ROS_WARN_STREAM(taskPtr->GetTaskName() << " cannot stop");
    } else {
      ROS_INFO_STREAM(taskPtr->GetTaskName() << ": stop");
      task_executor_.PostTask(taskPtr, TASK_STOP);
      // it is set in OnTaskComplete()
      // taskPtr->SetTaskStatus(kTaskStatusStop);
    }
  } else if (cmd.GetCommand() == kTaskCommandUntil) {
  } else {
    ROS_INFO_STREAM(cmd.GetCommand()
      << " is a wrong command in task: " << taskPtr->GetTaskName());
  }
}

void DecisionManager::OnTaskComplete(Task& task) {
ROS_INFO_STREAM(__FUNCTION__);
  task.SetTaskStatus(kTaskStatusStop);
  DecisionListChecking(task_container_.GetTask(task.GetTaskName()));
ROS_INFO_STREAM("=====================================");
}
void DecisionManager::OnTaskCancelled(Task& task) {
ROS_INFO_STREAM(__FUNCTION__ << " ================================");
ROS_INFO_STREAM("=====================================");
}
void DecisionManager::OnTaskFailed(Task& task) {
ROS_INFO_STREAM(__FUNCTION__ << " ================================");
ROS_INFO_STREAM("=====================================");
}
void DecisionManager::OnTaskStopped(Task& task) {
ROS_INFO_STREAM(__FUNCTION__ << " ================================");
  task.SetTaskStatus(kTaskStatusStop);
  DecisionListChecking(task_container_.GetTask(task.GetTaskName()));
ROS_INFO_STREAM("=====================================");
}
void DecisionManager::OnGoalEvent(Task& task, TaskCommand& cmd) {
ROS_INFO_STREAM(__FUNCTION__ << " ================================");
  ROS_INFO_STREAM(task.GetTaskName() << ": has been called (listener hero)");
  ROS_INFO_STREAM(cmd.GetCommand() << ": has been set (listener command)");
  ROS_INFO_STREAM(cmd.GetTaskName() << ": has been called (listener sidekick)");
  DecisionMaking(cmd);
ROS_INFO_STREAM("=====================================");
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
