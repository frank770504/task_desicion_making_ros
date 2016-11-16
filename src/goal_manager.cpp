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
#include <decision_maneger/goal_manager.h>
#include <queue>
#include <string>
#include <vector>

// maybe I have to add a namespace here

// add defined name here
const std::string GoalManager::kCancelGoalSubName_ = "cancel_goal";
const std::string GoalManager::kNewGoalSubName_ = "new_goal";
const std::string GoalManager::kNewGoalStampedSubName_ = "new_goal_stamped";
const std::string GoalManager::kGoalSequenceKey_ = "goal_sequence";
const std::string GoalManager::kActionLibServername_ = "move_base";
const std::string GoalManager::kGoalFrameId_ = "map";
const int GoalManager::kSleepTime_ = 100000;  // u sec

GoalManager::GoalManager()
  : ind_(1), is_doing_topic_goal_(false),
  is_wating_for_reaching_goal_(false),
  workPtr_(new boost::asio::io_service::work(ioService_)) {
}

  // usage functions
void GoalManager::NewGoalStampedSubCbk(const geometry_msgs::PoseStamped::ConstPtr& goal) {  // NOLINT
  bool is_goal_vector_empty = goal_vector_.empty();
  bool is_param_goal_vector_empty = param_goal_vector_.empty();
  geometry_msgs::PoseStamped pose_tmp;
  pose_tmp.header = goal->header;
  pose_tmp.pose = goal->pose;
  mtx_.lock();
  goal_vector_.push(pose_tmp);
  mtx_.unlock();
  if (is_goal_vector_empty) {
    if (is_param_goal_vector_empty && !is_doing_topic_goal_)
      cond_.notify_all();
  } else if (!is_doing_topic_goal_) {
    action_client_->cancelAllGoals();
    cond_.notify_all();
  }  // else do nothing
}

void GoalManager::NewGoalSubCbk(const geometry_msgs::Pose::ConstPtr& goal) {
  bool is_goal_vector_empty = goal_vector_.empty();
  bool is_param_goal_vector_empty = param_goal_vector_.empty();
  geometry_msgs::PoseStamped pose_tmp;
  pose_tmp.header.stamp = ros::Time::now();
  pose_tmp.pose.position = goal->position;
  pose_tmp.pose.orientation = goal->orientation;
  mtx_.lock();
  goal_vector_.push(pose_tmp);
  mtx_.unlock();
  if (is_goal_vector_empty) {
    if (is_param_goal_vector_empty && !is_doing_topic_goal_)
      cond_.notify_all();
  } else if (!is_doing_topic_goal_) {
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
    std::vector<Point2D>().swap(param_goal_vector_);
}

void GoalManager::CancelGoalSubCbk(const std_msgs::String::ConstPtr& cancel) {
  ReleaseGoalVectors();
  action_client_->cancelAllGoals();
}

void GoalManager::WaitGoalReaching() {
    while (action_client_->waitForResult(ros::Duration(1, 0)) == false) {
      if (!nh_.ok())  // exit if ros node is closed. (by pressing ctrl+c)
        exit(0);
    }
    // if don't cancel all the goals, the program will go to next goal after
    // reach the current goal
    if (action_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {  // NOLINT
      ROS_INFO_STREAM("Goal SUCCEEDED!!");
    } else {
      ROS_INFO_STREAM("Goal Reaching FAILED!!");
    }
    cond_.notify_all();
}

void GoalManager::GoalSending() {
  while (1) {
    boost::unique_lock<boost::mutex> lock{mtx_notify_};
    if (IsGoalVectorsEmpty() || is_wating_for_reaching_goal_) {
      if (is_wating_for_reaching_goal_) {
        ROS_INFO_STREAM("Waiting for reaching this goal!");
      } else {
        ROS_INFO_STREAM("Wait for Goals!");
      }
      cond_.wait(mtx_notify_);
      is_wating_for_reaching_goal_ = false;
      is_doing_topic_goal_ = false;
    }
    Point2D point_tmp;
    move_base_msgs::MoveBaseGoal goal_tmp;
    geometry_msgs::PoseStamped pose_tmp;
    goal_tmp.target_pose.header.frame_id = kGoalFrameId_;
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
      point_tmp.x_ = goal_tmp.target_pose.pose.position.x;
      point_tmp.y_ = goal_tmp.target_pose.pose.position.y;
      point_tmp.th_ = tf::getYaw(goal_tmp.target_pose.pose.orientation);
    } else {
      is_doing_topic_goal_ = false;
      if ( param_goal_vector_.size() == 0 )
        continue;
      point_tmp = param_goal_vector_.back();
      param_goal_vector_.pop_back();
      goal_tmp.target_pose.pose.position.x = point_tmp.x_;
      goal_tmp.target_pose.pose.position.y = point_tmp.y_;
      goal_tmp.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(point_tmp.th_);  // NOLINT
      goal_tmp.target_pose.header.stamp = ros::Time::now();
      phase = "param";
    }
    // the piority of  goal_vector is higher than param_goal_vector

    // send goal
    action_client_->sendGoal(goal_tmp);
    is_wating_for_reaching_goal_ = true;
    ROS_INFO_STREAM(
      phase <<"| Sending Goal: - [" << point_tmp.x_ << ", "
            << point_tmp.y_ << ", " << point_tmp.th_ << "]");
    ioService_.post(boost::bind(&GoalManager::WaitGoalReaching, this));
    usleep(kSleepTime_);
  }
}

  // test functions
void GoalManager::ParamGoalVectorPrintTest() {
  for (int i = 0; i < param_goal_vector_.size(); i++) {
    Point2D _tmp;
    _tmp = param_goal_vector_[i];
    ROS_INFO_STREAM(
      "- [x: " << _tmp.x_ << ", y: " << _tmp.y_ << ", th: " << _tmp.th_ << "]");
  }
}

void GoalManager::Initialize(ros::NodeHandle n) {
  nh_ = n;
  ROS_INFO_STREAM("Goal Manager Init...");
  ROS_INFO_STREAM("Start ActionLib");
  // Connect to the move_base action server
  // create a thread to handle subscriptions.
  action_client_ = new ActionClient(kActionLibServername_, true);
  ROS_INFO_STREAM("It will wait until move base open");
  action_client_->waitForServer();
  ROS_INFO("Server OK");

  new_goal_stamped_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(
    kNewGoalStampedSubName_, 1, boost::bind(&GoalManager::NewGoalStampedSubCbk, this, _1));  // NOLINT
  new_goal_sub_ = nh_.subscribe<geometry_msgs::Pose>(
    kNewGoalSubName_, 1, boost::bind(&GoalManager::NewGoalSubCbk, this, _1));
  cancel_goal_sub_ = nh_.subscribe<std_msgs::String>(
    kCancelGoalSubName_, 1, boost::bind(&GoalManager::CancelGoalSubCbk, this, _1));  // NOLINT

  GoalSendingThread_.reset(
    new boost::thread(boost::bind(&GoalManager::GoalSending, this)) );
  AsioThread_.reset(
    new boost::thread(boost::bind(&boost::asio::io_service::run, &ioService_)));
  XmlRpc::XmlRpcValue yml;
  if (!nh_.getParam(kGoalSequenceKey_, yml)) {
    ROS_ERROR_STREAM("get " << kGoalSequenceKey_ << " error");
    ROS_INFO_STREAM("Goal Manager Init without parameter goals");
    cond_.notify_all();
    return;
  }
  // push form backward so that it can pop from the front
  for (int i = yml.size() - 1; i >= 0; i--) {
    Point2D pose_tmp(yml[i][0], yml[i][1], yml[i][2]);
    param_goal_vector_.push_back(pose_tmp);
  }
  cond_.notify_all();
  ROS_INFO_STREAM("Goal Manager Init...OK...");
}

void GoalManager::Run() {
}

void GoalManager::Stop() {
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "goal_manager");
  ros::NodeHandle nh;
  GoalManager gm;
  gm.Initialize(nh);
  ros::spin();
  return 0;
}
