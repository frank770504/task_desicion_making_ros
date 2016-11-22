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
#ifndef INCLUDE_DECISION_MANAGER_FINDING_TOOL_H_
#define INCLUDE_DECISION_MANAGER_FINDING_TOOL_H_

#include <decision_manager/task.h>
#include <laser_tool/FindShelf.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <queue>
#include <string>
#include <vector>

namespace decision_manager_plugin {

class FindingTool : public decision_manager::Task {
 public:
  FindingTool();
  ~FindingTool();
  // Fullfill Task interface
  void Initialize(ros::NodeHandle n, std::string task_name,
                  bool can_stop, bool can_cancel);
  void Run();
  void Stop();
 private:
  ros::NodeHandle nh_;
  static const std::string kFindShelfServiceName_;
  static const std::string kFindShelfSucceedCmd_;
  ros::ServiceClient find_shelf_service_client_;
  laser_tool::FindShelf finding_tool_srv_catcher_;
  std::vector<std::string> shelf_location_;
 private:
  std::vector<std::string> StringSplit(std::string str, std::string pattern);
};
};  // namespace decision_manager_plugin
#endif  // INCLUDE_DECISION_MANAGER_FINDING_TOOL_H_
