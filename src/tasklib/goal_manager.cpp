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
#include <decision_manager/goal_manager.h>
#include <pluginlib/class_list_macros.h>
#include <queue>
#include <string>
#include <vector>
#include <decision_manager/TaskGoalMsg.h>

// maybe I have to add a namespace here
namespace decision_manager_plugin {

// add defined name here
const std::string GoalManager::kCancelGoalSubName_ = "cancel_goal";
const std::string GoalManager::kTaskGoalSubName_ = "task_goal";
const std::string GoalManager::kNewGoalSubName_ = "new_goal";
const std::string GoalManager::kNewGoalStampedSubName_ = "new_goal_stamped";
const std::string GoalManager::kGoalSequenceKey_ = "goal_sequence";
const std::string GoalManager::kActionLibServername_ = "move_base";
const std::string GoalManager::kGoalFrameId_ = "map";
const int GoalManager::kSleepTime_ = 100000;  // u sec
const std::string GoalManager::kEmptyCommandOrTask_ = "None";

GoalManager::GoalManager()
  : ind_(1), is_doing_topic_goal_(false),
  is_wating_for_reaching_goal_(false), is_task_stop_(true) {
}

  // usage functions
void GoalManager::NewGoalStampedSubCbk(const geometry_msgs::PoseStamped::ConstPtr& goal) {  // NOLINT
  bool is_goal_vector_empty = goal_vector_.empty();
  bool is_param_goal_vector_empty = param_goal_vector_.empty();
  geometry_msgs::PoseStamped pose_tmp;
  pose_tmp.header = goal->header;
  pose_tmp.pose = goal->pose;
  mtx_.lock();
  pose_tmp.header.stamp = ros::Time::now();
  goal_vector_.push(pose_tmp);
  mtx_.unlock();
  decision_manager::TaskGoalMsg tmp;
  tmp.command = kEmptyCommandOrTask_;
  tmp.task = kEmptyCommandOrTask_;
  goal_task_map_[pose_tmp.header.stamp] = tmp;
  if (is_goal_vector_empty) {
    if (is_param_goal_vector_empty && !is_doing_topic_goal_)
      cond_.notify_all();
  } else if (!is_doing_topic_goal_ && is_wating_for_reaching_goal_) {
    action_client_->cancelAllGoals();
    cond_.notify_all();
  }  // else do nothing
}

void GoalManager::TaskGoalSubCbk(const decision_manager::TaskGoalMsg::ConstPtr& goal) {
  bool is_goal_vector_empty = goal_vector_.empty();
  bool is_param_goal_vector_empty = param_goal_vector_.empty();
  geometry_msgs::PoseStamped pose_tmp;
  pose_tmp.pose.position.x = goal->x;
  pose_tmp.pose.position.y = goal->y;
  pose_tmp.pose.orientation = tf::createQuaternionMsgFromYaw(goal->th);
  mtx_.lock();
  pose_tmp.header.stamp = ros::Time::now();
  goal_vector_.push(pose_tmp);
  mtx_.unlock();
  decision_manager::TaskGoalMsg tmp;
  tmp.command = goal->command;
  tmp.task = goal->task;
  goal_task_map_[pose_tmp.header.stamp] = tmp;
  if (is_goal_vector_empty) {
    if (is_param_goal_vector_empty && !is_doing_topic_goal_)
      cond_.notify_all();
  } else if (!is_doing_topic_goal_ && is_wating_for_reaching_goal_) {
    action_client_->cancelAllGoals();
    cond_.notify_all();
  }  // else do nothing
}

void GoalManager::NewGoalSubCbk(const geometry_msgs::Pose::ConstPtr& goal) {
  bool is_goal_vector_empty = goal_vector_.empty();
  bool is_param_goal_vector_empty = param_goal_vector_.empty();
  geometry_msgs::PoseStamped pose_tmp;
  pose_tmp.pose.position = goal->position;
  pose_tmp.pose.orientation = goal->orientation;
  mtx_.lock();
  pose_tmp.header.stamp = ros::Time::now();
  goal_vector_.push(pose_tmp);
  mtx_.unlock();
  decision_manager::TaskGoalMsg tmp;
  tmp.command = kEmptyCommandOrTask_;
  tmp.task = kEmptyCommandOrTask_;
  goal_task_map_[pose_tmp.header.stamp] = tmp;
  if (is_goal_vector_empty) {
    if (is_param_goal_vector_empty && !is_doing_topic_goal_)
      cond_.notify_all();
  } else if (!is_doing_topic_goal_ && is_wating_for_reaching_goal_) {
    action_client_->cancelAllGoals();
    cond_.notify_all();
  }  // else do nothing
}

bool GoalManager::IsGoalVectorsEmpty() {
  int size = 0;
  size = param_goal_vector_.size();
  mtx_.lock();
  size += goal_vector_.size();
  mtx_.unlock();
  if (size == 0)
    return true;
  else
    return false;
}

void GoalManager::ReleaseGoalVectors() {
    // release goal vector
    param_goal_vector_.clear();
    std::queue<geometry_msgs::PoseStamped>().swap(goal_vector_);
    std::vector<decision_manager::TaskGoalMsg>().swap(param_goal_vector_);
}

void GoalManager::CancelGoalSubCbk(const std_msgs::String::ConstPtr& cancel) {
  ReleaseGoalVectors();
  action_client_->cancelAllGoals();
}

void GoalManager::GoalSending() {
  while (1) {
    boost::unique_lock<boost::mutex> task_lock{mtx_task_notify_};
    if (is_task_stop_) {
      //~ OnTaskEventCaller(*this, decision_manager::OnTaskFailedID);
      OnTaskEventCaller(*this, decision_manager::OnTaskStoppedID);
      ROS_INFO_STREAM("Waiting for running task!");
      task_cond_.wait(mtx_task_notify_);
      is_doing_topic_goal_ = false;
    }
    boost::unique_lock<boost::mutex> lock{mtx_notify_};
    if (IsGoalVectorsEmpty() || is_wating_for_reaching_goal_) {
      if (is_wating_for_reaching_goal_) {
        ROS_INFO_STREAM("Waiting for reaching this goal!");
      } else {
        ROS_INFO_STREAM("Wait for Goals!");
      }
      cond_.wait(mtx_notify_);
      is_doing_topic_goal_ = false;
    }
    if (!is_task_stop_) {
      decision_manager::TaskGoalMsg point_tmp;
      move_base_msgs::MoveBaseGoal goal_tmp;
      geometry_msgs::PoseStamped pose_tmp;
      std::string phase;
      // set goal
      if ( !goal_vector_.empty() ) {
        is_doing_topic_goal_ = true;
        pose_tmp = goal_vector_.front();
        goal_vector_.pop();
        goal_tmp.target_pose.header = pose_tmp.header;
        goal_tmp.target_pose.pose = pose_tmp.pose;
        phase = "topic";
        // for ROS_INFO_STREAM
        point_tmp.x = goal_tmp.target_pose.pose.position.x;
        point_tmp.y = goal_tmp.target_pose.pose.position.y;
        point_tmp.th = tf::getYaw(goal_tmp.target_pose.pose.orientation);
      } else {
        is_doing_topic_goal_ = false;
        if ( param_goal_vector_.size() == 0 )
          continue;
        point_tmp = param_goal_vector_.back();
        param_goal_vector_.pop_back();
        goal_tmp.target_pose.pose.position.x = point_tmp.x;
        goal_tmp.target_pose.pose.position.y = point_tmp.y;
        goal_tmp.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(point_tmp.th);  // NOLINT
        mtx_.lock();
        goal_tmp.target_pose.header.stamp = ros::Time::now();
        mtx_.unlock();
        goal_task_map_[goal_tmp.target_pose.header.stamp] = point_tmp;
        phase = "param";
      }
      // the piority of  goal_vector is higher than param_goal_vector
      // send goal
      goal_tmp.target_pose.header.frame_id = kGoalFrameId_;
      current_task_stamp_ = goal_tmp.target_pose.header.stamp;
      action_client_->sendGoal(goal_tmp, boost::bind(&GoalManager::ActionGoalDone, this, _1),
                                         boost::bind(&GoalManager::ActionActive, this),
                                         ActionClient::SimpleFeedbackCallback());
      ROS_INFO_STREAM(
        phase <<"| Sending Goal: - [" << point_tmp.x << ", "
              << point_tmp.y << ", " << point_tmp.th << "]");
      usleep(kSleepTime_);
    }
  }
}

void GoalManager::ActionActive() {
  ROS_INFO_STREAM("move_base_action_start");
  is_wating_for_reaching_goal_ = true;
}

void GoalManager::ActionGoalDone(
  const actionlib::SimpleClientGoalState& state) {
  ROS_INFO_STREAM("ActionGoalDoneCB: " << state.toString());
  is_wating_for_reaching_goal_ = false;
  // if don't cancel all the goals, the program will go to next goal after
  // reach the current goal
  decision_manager::TaskGoalMsg tmp = goal_task_map_[current_task_stamp_];
  goal_task_map_.erase(current_task_stamp_);
  ROS_INFO_STREAM(tmp.command << ", " << tmp.task);
  if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {  // NOLINT
    ROS_INFO_STREAM("Goal SUCCEEDED!!");
    if (tmp.command == decision_manager::kTaskCommandUntil) {
      OnGoalEventCaller(*this, taskCommand_.StopSelfUntil(tmp.task));
    } else if (tmp.command == decision_manager::kTaskCommandRun) {
      OnGoalEventCaller(*this, taskCommand_.RunTask(tmp.task));
    } else if (tmp.command == decision_manager::kTaskCommandWait) {
      OnGoalEventCaller(*this, taskCommand_.StopTask(tmp.task));
    } else {
      cond_.notify_all();
    }
  } else {
    ROS_INFO_STREAM("Goal State: " << state.toString());
    cond_.notify_all();
  }
}

void GoalManager::Initialize(
  ros::NodeHandle n, std::string task_name, bool can_stop, bool can_cancel) {
  SetTaskName(task_name);
  SetTaskStateBool(can_stop, can_cancel);
  nh_ = n;
  ROS_INFO_STREAM("Goal Manager Init...");
  ROS_INFO_STREAM("Start ActionLib");
  // Connect to the move_base action server
  // create a thread to handle subscriptions.
  action_client_ = new ActionClient(kActionLibServername_, true);
  ROS_INFO_STREAM("It will wait until move base open");
  action_client_->waitForServer();
  ROS_INFO("Server OK");
  current_task_stamp_ = ros::Time::now();

  task_goal_sub_ = nh_.subscribe<decision_manager::TaskGoalMsg>(
    kTaskGoalSubName_, 1, boost::bind(&GoalManager::TaskGoalSubCbk, this, _1));  // NOLINT
  new_goal_stamped_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(
    kNewGoalStampedSubName_, 1, boost::bind(&GoalManager::NewGoalStampedSubCbk, this, _1));  // NOLINT
  new_goal_sub_ = nh_.subscribe<geometry_msgs::Pose>(
    kNewGoalSubName_, 1, boost::bind(&GoalManager::NewGoalSubCbk, this, _1));
  cancel_goal_sub_ = nh_.subscribe<std_msgs::String>(
    kCancelGoalSubName_, 1, boost::bind(&GoalManager::CancelGoalSubCbk, this, _1));  // NOLINT

  GoalSendingThread_.reset(
    new boost::thread(boost::bind(&GoalManager::GoalSending, this)) );
  XmlRpc::XmlRpcValue yml;
  if (!nh_.getParam(kGoalSequenceKey_, yml)) {
    ROS_ERROR_STREAM("get " << kGoalSequenceKey_ << " error");
    ROS_INFO_STREAM("Goal Manager Init without parameter goals");
    cond_.notify_all();
    return;
  }
  // push form backward so that it can pop from the front
  for (int i = yml.size() - 1; i >= 0; i--) {
    decision_manager::TaskGoalMsg goal_tmp;
    XmlRpc::XmlRpcValue tmp = yml[i];
    goal_tmp.x = static_cast<double>(tmp[0]);
    goal_tmp.y = static_cast<double>(tmp[1]);
    goal_tmp.th = static_cast<double>(tmp[2]);
    goal_tmp.command = static_cast<std::string>(tmp[3]);
    goal_tmp.task = static_cast<std::string>(tmp[4]);
    param_goal_vector_.push_back(goal_tmp);
  }
  cond_.notify_all();
  ROS_INFO_STREAM("Goal Manager Init...OK...");
}

void GoalManager::Run() {
  is_task_stop_ = false;
  ROS_INFO_STREAM("task run!");
  task_cond_.notify_all();
}

void GoalManager::Stop() {
  is_task_stop_ = true;
  ROS_INFO_STREAM("task stop!");
  if (is_wating_for_reaching_goal_) {
    action_client_->cancelAllGoals();
  }else {
      cond_.notify_all();
  }
}
};  // namespace decision_manager_plugin

PLUGINLIB_EXPORT_CLASS(decision_manager_plugin::GoalManager, decision_manager::Task);  // NOLINT
