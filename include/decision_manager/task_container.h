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
#include <decision_manager/task_listener.h>
#include <boost/shared_ptr.hpp>

#include <pluginlib/class_loader.h>
#include <string>
#include <vector>

#ifndef INCLUDE_DECISION_MANAGER_TASK_CONTAINER_H_
#define INCLUDE_DECISION_MANAGER_TASK_CONTAINER_H_

namespace decision_manager {

class Task;

class TaskContainer {
  typedef boost::shared_ptr<Task> TaskPtr;
  typedef pluginlib::ClassLoader<Task> PluginLoader;
  typedef std::map<std::string, TaskPtr> TaskMap;

 public:
  explicit TaskContainer(ros::NodeHandle n);
  ~TaskContainer() {
  }
  const TaskPtr& GetTask(const std::string& name) const;
  const TaskMap& GetTasks() const {
    return task_map_handle_list_;
  }
  void SetTasksListener(const TaskListenerPtr& listener);
 private:
  ros::NodeHandle nh_;
  boost::shared_ptr<PluginLoader> task_loader_;
  TaskMap task_map_handle_list_;
  XmlRpc::XmlRpcValue yml_params_;
  static const std::string kPluginPkgName_;

  static const std::string kPluginBaseClassTypeKey_;
  std::string PluginBaseClassType_;

  static const std::string kPluginTypeNamespaceKey_;
  std::string PluginTypeNamespace_;

  static const std::string kPluginUsedTaskListKey_;

  static const std::string kTaskNameKey_;
  static const std::string kTaskCanStopKey_;
  static const std::string kTaskCanCancelKey_;

  static const std::string kRunSubName_;
  static const std::string kStopSubName_;
};
};  // namespace decision_manager

#endif  // INCLUDE_DECISION_MANAGER_TASK_CONTAINER_H_
