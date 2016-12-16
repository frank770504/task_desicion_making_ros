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
#ifndef INCLUDE_DECISION_MANAGER_TASK_H_
#define INCLUDE_DECISION_MANAGER_TASK_H_

#include <ros/ros.h>
#include <decision_manager/task_listener.h>
#include <string>
#include <vector>

namespace decision_manager {

enum TaskExecuteIndex {
  TASK_RUN,
  TASK_STOP
};

static const std::string kTaskStatusRun = "task.status.run";
static const std::string kTaskStatusStop = "task.status.stop";
static const std::string kTaskStatusUntil = "task.status.until";

class TaskStatus {
 public:
  TaskStatus()
    : is_stoppable_(false),
      is_able_to_cancel_(false) {
  }
  void SetStoppableFlag(bool st) {
    is_stoppable_ = st;
  }
  bool IsStoppable() {
    return is_stoppable_;
  }
  void SetAbleToCancelFlag(bool st) {
    is_able_to_cancel_ = st;
  }
  bool IsAbleToCancel() {
    return is_able_to_cancel_;
  }
  void SetTaskStatus(const std::string& st) {
    status_ = st;
  }
  const std::string& GetTaskStatus() {
    return status_;
  }
 private:
  bool is_stoppable_;
  bool is_able_to_cancel_;
  std::string status_;
};  // TODO(FrankChen) define structure.

static const std::string kTaskCommandRun = "task.command.run";
static const std::string kTaskCommandStop = "task.command.stop";
static const std::string kTaskCommandUntil = "task.command.until";

class TaskCommand {
 public:
  TaskCommand() {}
  TaskCommand& RunTask(std::string task_name) {
    command_ = kTaskCommandRun;
    task_name_ = task_name;
    return *this;
  }
  TaskCommand& StopTask(std::string task_name) {
    command_ = kTaskCommandStop;
    task_name_ = task_name;
    return *this;
  }
  TaskCommand& StopSelfUntil(std::string task_name) {
    command_ = kTaskCommandUntil;
    task_name_ = task_name;
    return *this;
  }
  const std::string& GetTaskName() const  {
   return task_name_;
  }
  const std::string& GetCommand() const  {
   return command_;
  }
 private:
  std::string command_;
  std::string task_name_;
};

class Task {
 public:
     const TaskStatus& GetTaskState() const {
       return taskStatus_;
     }
     const std::string& GetTaskName() const  {
       return taskName_;
     }
     void AddTaskListener(const TaskListenerPtr& ptr) {
       taskListeners_.push_back(ptr);
     }
     // TODO(FrankChen): add defined command
     void OnTaskEventCaller(Task& task, int func_id) {
       std::vector<TaskListenerPtr>::iterator iter = taskListeners_.begin();
       for (; iter != taskListeners_.end(); ++iter) {
          if (func_id == OnTaskCompleteID) {
            (*iter)->OnTaskComplete(task);
          } else if (func_id == OnTaskCancelledID) {
            (*iter)->OnTaskCancelled(task);
          } else if (func_id == OnTaskFailedID) {
            (*iter)->OnTaskFailed(task);
          } else if (func_id == OnTaskStoppedID) {
            (*iter)->OnTaskStopped(task);
          }
       }
     }
     void OnGoalEventCaller(Task& task, TaskCommand& cmd) {
       std::vector<TaskListenerPtr>::iterator iter = taskListeners_.begin();
       for (; iter != taskListeners_.end(); ++iter) {
            (*iter)->OnGoalEvent(task, cmd);
       }
     }
     void SetTaskStatus(const std::string& st) {
       taskStatus_.SetTaskStatus(st);
     }
     const std::string& GetTaskStatus() {
       return taskStatus_.GetTaskStatus();
     }
     virtual void Run() = 0;
     virtual void Stop() = 0;
     virtual void Initialize(ros::NodeHandle n, std::string task_name,
                             bool can_stop, bool can_cancel) = 0;
     virtual ~Task() {}

 protected:
     Task() {}
     void SetTaskName(std::string name) {
       taskName_ = name;
     }
     void SetTaskStateBool(bool can_stop, bool can_cancel) {
       taskStatus_.SetStoppableFlag(can_stop);
       taskStatus_.SetAbleToCancelFlag(can_cancel);
     }
     void ExecuteInner();

 protected:
     std::vector<TaskListenerPtr> taskListeners_;
     TaskCommand taskCommand_;

 private:
     std::string taskName_;
     std::vector<std::string> actions_;
     TaskStatus taskStatus_;
};
typedef boost::shared_ptr<Task> TaskPtr;
static const TaskPtr NullPtr;
}  // namespace decision_manager
#endif  // INCLUDE_DECISION_MANAGER_TASK_H_
