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

#include <decision_manager/finding_tool.h>
#include <pluginlib/class_list_macros.h>
#include <string>
#include <vector>
#include <decision_manager/TaskGoalMsg.h>

namespace decision_manager_plugin {

const std::string FindingTool::kFindShelfServiceName_ = "/laser_tool/find_shelf";  // NOLINT
const std::string FindingTool::kFindShelfSucceedCmd_ = "goto";

FindingTool::FindingTool() {
}
FindingTool::~FindingTool() {
}

std::vector<std::string>
  FindingTool::StringSplit(std::string str, std::string pattern) {
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

void FindingTool::Initialize(
  ros::NodeHandle n, std::string task_name, bool can_stop, bool can_cancel) {
  SetTaskName(task_name);
  SetTaskStateBool(can_stop, can_cancel);
  nh_ = n;
  find_shelf_service_client_ =
    nh_.serviceClient<laser_tool::FindShelf>(kFindShelfServiceName_);
  goal_pub_ = nh_.advertise<decision_manager::TaskGoalMsg>("task_goal", 2);
}

void FindingTool::Run() {
  if (find_shelf_service_client_.call(finding_tool_srv_catcher_)) {
    //~ ros::Duration(2.0).sleep();
    std::string shelf_loc = finding_tool_srv_catcher_.response.coord;
    ROS_INFO_STREAM("shelf location" << shelf_loc);
    shelf_location_ = StringSplit(shelf_loc, " ");
    if (shelf_location_[0] == kFindShelfSucceedCmd_) {
      decision_manager::TaskGoalMsg tmpose;
      std::string::size_type sz;
      tmpose.x = double(stod(shelf_location_[1], &sz) + 0.2);
      tmpose.y = double(stod(shelf_location_[2], &sz));
      double angle = stod(shelf_location_[3], &sz);
      angle = angle * 3.1415926 / 180;
      tmpose.th = angle;
      tmpose.command = "None_f";
      tmpose.task = "None_ff";
      goal_pub_.publish(tmpose);
    }

  } else {
    ROS_INFO_STREAM("!!!! Finding Shelf Service Failed !!!!");
  }
  usleep(500000);
  OnTaskEventCaller(*this, decision_manager::OnTaskCompleteID);
}

void FindingTool::Stop() {
  ROS_INFO_STREAM("!!!! Finding Shelf Stop !!!!");
}

};  // namespace decision_manager_plugin

PLUGINLIB_EXPORT_CLASS(decision_manager_plugin::FindingTool, decision_manager::Task);  // NOLINT
