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
#include <map>

class TaskListenerLoadTEST : public decision_manager::TaskListener {
  typedef boost::shared_ptr<decision_manager::Task> TaskPtr;
  typedef pluginlib::ClassLoader<decision_manager::Task> PluginLoader;
  typedef std::map<std::string, TaskPtr> TaskMap;

 public:
  explicit TaskListenerLoadTEST(ros::NodeHandle n)
    : nh_(n) {
    run_sub_ = nh_.subscribe<std_msgs::Empty>(kRunSubName_, 1,
      boost::bind(&TaskListenerLoadTEST::RunCb, this, _1));
    stop_sub_ = nh_.subscribe<std_msgs::Empty>(kStopSubName_, 1,
      boost::bind(&TaskListenerLoadTEST::StopCb, this, _1));

    // load yaml
    if (!nh_.getParam(kPluginBaseClassTypeKey_, PluginBaseClassType_)) {
      ROS_ERROR_STREAM("get " << kPluginBaseClassTypeKey_ << " error");
      return;
    }
    task_loader_.reset(new PluginLoader(kPluginPkgName_, PluginBaseClassType_));
    if (!nh_.getParam(kPluginTypeNamespaceKey_, PluginTypeNamespace_)) {
      ROS_ERROR_STREAM("get " << kPluginBaseClassTypeKey_ << " error");
      return;
    }
    if (!nh_.getParam(kPluginUsedTaskListKey_, yml_params_)) {
      ROS_ERROR_STREAM("get " << kPluginUsedTaskListKey_ << " error");
      return;
    } else {
      // load task name
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

    // task class loader

    TaskMap::const_iterator titer = task_map_handle_list_.begin();
    int i = 0;
    for (; titer != task_map_handle_list_.end(); ++titer) {
      std::string _key = kPluginPkgName_ + "/" + titer->first;
      if (!nh_.getParam(_key, yml_params_)) {
        ROS_ERROR_STREAM("get " << _key << " error");
        continue;
      } else {
        (titer->second)->Initialize(nh_,
                                    yml_params_[kTaskNameKey_],
                                    yml_params_[kTaskCanStopKey_],
                                    yml_params_[kTaskCanCancelKey_]);
        ROS_INFO_STREAM((titer->second)->GetTaskName() << ": is initialized.");
        (titer->second)->AddTaskListener(decision_manager::TaskListenerPtr(this));  // NOLINT
      }
    }
  }
  void RunCb(const std_msgs::Empty::ConstPtr& et) {
    TaskMap::const_iterator titer = task_map_handle_list_.begin();
    for (; titer != task_map_handle_list_.end(); ++titer) {
      ROS_INFO_STREAM((titer->second)->GetTaskName() << ": run");
      (titer->second)->Run();
    }
  }
  void StopCb(const std_msgs::Empty::ConstPtr& et) {
    TaskMap::const_iterator titer = task_map_handle_list_.begin();
    for (; titer != task_map_handle_list_.end(); ++titer) {
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

const std::string TaskListenerLoadTEST::kRunSubName_ = "task_run";
const std::string TaskListenerLoadTEST::kStopSubName_ = "task_stop";
const std::string TaskListenerLoadTEST::kPluginPkgName_ = "decision_manager";
const std::string TaskListenerLoadTEST::kPluginBaseClassTypeKey_ =
  kPluginPkgName_ + "/base_class_type";
const std::string TaskListenerLoadTEST::kPluginTypeNamespaceKey_ =
  kPluginPkgName_ + "/plugin_namespace";
const std::string TaskListenerLoadTEST::kPluginUsedTaskListKey_ =
  kPluginPkgName_ + "/used_task_list";

const std::string TaskListenerLoadTEST::kTaskNameKey_ = "task_name";
const std::string TaskListenerLoadTEST::kTaskCanStopKey_ = "can_stop";
const std::string TaskListenerLoadTEST::kTaskCanCancelKey_ = "can_cancel";

int main(int argc, char** argv) {
  ros::init(argc, argv, "task_listener_loader");
  ros::NodeHandle nh;
  TaskListenerLoadTEST TLLT(nh);
  ros::spin();
  return 0;
}
