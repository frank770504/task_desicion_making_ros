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

#include <decision_manager/task_container.h>
#include <boost/shared_ptr.hpp>

#include <pluginlib/class_loader.h>
#include <string>
#include <vector>

namespace decision_manager {

const std::string TaskContainer::kPluginPkgName_ = "decision_manager";
const std::string TaskContainer::kPluginBaseClassTypeKey_ =
  kPluginPkgName_ + "/base_class_type";
const std::string TaskContainer::kPluginTypeNamespaceKey_ =
  kPluginPkgName_ + "/plugin_namespace";
const std::string TaskContainer::kPluginUsedTaskListKey_ =
  kPluginPkgName_ + "/used_task_list";

const std::string TaskContainer::kTaskCanStopKey_ = "can_stop";
const std::string TaskContainer::kTaskCanCancelKey_ = "can_cancel";

TaskContainer::TaskContainer(ros::NodeHandle n)
  : nh_(n) {
    // load yaml
    if (!nh_.getParam(kPluginBaseClassTypeKey_, PluginBaseClassType_)) {
      ROS_ERROR_STREAM("get " << kPluginBaseClassTypeKey_ << " error");
      return;
    }
    task_loader_.reset(new PluginLoader(kPluginPkgName_, PluginBaseClassType_));
    if (!nh_.getParam(kPluginTypeNamespaceKey_, PluginTypeNamespace_)) {
      ROS_ERROR_STREAM("get " << kPluginTypeNamespaceKey_ << " error");
      return;
    }
    if (!nh_.getParam(kPluginUsedTaskListKey_, yml_params_)) {
      ROS_ERROR_STREAM("get " << kPluginUsedTaskListKey_ << " error");
      return;
    } else {
      // load task by name
      for (int i = 0; i < yml_params_.size(); i++) {
        std::string _str = yml_params_[i];
        try {
          task_map_handle_list_[_str] =
            task_loader_->createInstance(PluginTypeNamespace_ + "::" + _str);
        } catch(pluginlib::PluginlibException& ex) {
          ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());  // NOLINT
        }
      }
    }
    // task init
    TaskMap::const_iterator titer = task_map_handle_list_.begin();
    int i = 0;
    for (; titer != task_map_handle_list_.end(); ++titer) {
      std::string _key = kPluginPkgName_ + "/" + titer->first;
      if (!nh_.getParam(_key, yml_params_)) {
        ROS_ERROR_STREAM("get " << _key << " error");
        continue;
      } else {
        // TODO(FrankChen): Initialized time-out
        (titer->second)->Initialize(nh_,
                                    (titer->first),
                                    yml_params_[kTaskCanStopKey_],
                                    yml_params_[kTaskCanCancelKey_]);
        ROS_INFO_STREAM((titer->second)->GetTaskName() << ": is initialized.");
      }
    }
}

const TaskPtr& TaskContainer::GetTask(const std::string& name) const {
  if (task_map_handle_list_.find(name) != task_map_handle_list_.end()) { // NOLINT
    return task_map_handle_list_.at(name);
  }
  return NullPtr;
}

void TaskContainer::SetTasksListener(const TaskListenerPtr& listener) {
  TaskMap::const_iterator titer = task_map_handle_list_.begin();
  for (; titer != task_map_handle_list_.end(); ++titer) {
    (titer->second)->AddTaskListener(listener);
  }
}

};  // namespace decision_manager
