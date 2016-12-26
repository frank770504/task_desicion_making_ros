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

#include <decision_manager/shelf_combiner.h>
#include <pluginlib/class_list_macros.h>

namespace decision_manager_plugin {

ShelfCombiner::ShelfCombiner() {
}
ShelfCombiner::~ShelfCombiner() {
}

void ShelfCombiner::Initialize(
  ros::NodeHandle n, std::string task_name, bool can_stop, bool can_cancel) {
  SetTaskName(task_name);
  SetTaskStateBool(can_stop, can_cancel);
  nh_ = n;
  motor_pub_ = nh_.advertise<std_msgs::UInt8>("/robot/lifter_motor", 2);
}

void ShelfCombiner::Run() {
  ROS_INFO_STREAM("SC run!");
  std_msgs::UInt8 tmp;
  tmp.data = 17;
  motor_pub_.publish(tmp);
  usleep(100000);
  OnTaskEventCaller(*this, decision_manager::OnTaskCompleteID);
}

void ShelfCombiner::Stop() {
  ROS_INFO_STREAM("SC stop!");
}

};  // namespace decision_manager_plugin


PLUGINLIB_EXPORT_CLASS(decision_manager_plugin::ShelfCombiner, decision_manager::Task);  // NOLINT
