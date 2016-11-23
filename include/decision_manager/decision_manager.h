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
#ifndef INCLUDE_DECISION_MANAGER_DECISION_MANAGER_H_
#define INCLUDE_DECISION_MANAGER_DECISION_MANAGER_H_

#include <decision_manager/task.h>
#include <decision_manager/task_container.h>
#include <decision_manager/task_executor.h>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/asio/io_service.hpp>

#include <std_msgs/String.h>
#include <string>
#include <vector>

namespace decision_manager {
class DecisionManager : public TaskListener {
  typedef std::map<std::string, TaskPtr> TaskMap;

 public:
  explicit DecisionManager(ros::NodeHandle n);

  void WebCmdCallback(const std_msgs::String::ConstPtr& str);
  void InnerCmdCallback(const std_msgs::String::ConstPtr& str);
  virtual void OnTaskComplete(Task& task);
  virtual void OnTaskCancelled(Task& task);
  virtual void OnTaskFailed(Task& task);
  virtual void OnTaskStopped(Task& task);
  virtual void OnGoalControl(Task& task);

 private:
  std::vector<std::string> StringSplit(std::string str, std::string pattern);
  void DecisionMaking();
  ros::NodeHandle nh_;
  ros::Subscriber web_cmd_sub_;
  ros::Subscriber inner_cmd_sub_;
  static const std::string kCmdSubName_;
  TaskContainer task_container_;
  TaskExecutor task_executor_;
};
};  // namespace decision_manager

#endif  // INCLUDE_DECISION_MANAGER_DECISION_MANAGER_H_
