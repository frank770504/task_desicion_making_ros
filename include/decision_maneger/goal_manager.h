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

#ifndef INCLUDE_DECISION_MANEGER_GOAL_MANAGER_H_
#define INCLUDE_DECISION_MANEGER_GOAL_MANAGER_H_

#include "task.h"
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <boost/thread.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/shared_ptr.hpp>
#include <queue>
#include <string>
#include <vector>


// maybe I have to add a namespace here

struct Point2D {
  Point2D() : x_(0), y_(0), th_(0) {
  }
  Point2D(double x, double y, double th)
    : x_(x), y_(y), th_(th) {
  }
  double x_;
  double y_;
  double th_;
};

class GoalManager : public decision_manager::Task {
  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
    ActionClient;
  typedef boost::shared_ptr<boost::asio::io_service::work> WorkPtr;

 public:
  GoalManager();
  ~GoalManager() {
    ReleaseGoalVectors();
    // release actionlib
    delete action_client_;
  }

  void GoalSending();

  // Fullfill Task interface
  void Initialize(ros::NodeHandle n);
  void Run();
  void Stop();

  // test functions
  void ParamGoalVectorPrintTest();

 private:
  void NewGoalStampedSubCbk(const geometry_msgs::PoseStamped::ConstPtr& goal);
  void NewGoalSubCbk(const geometry_msgs::Pose::ConstPtr& goal);
  void CancelGoalSubCbk(const std_msgs::String::ConstPtr& cancel);
  void WaitGoalReaching();
  void ReleaseGoalVectors();
  bool IsGoalVectorsEmpty();
  ros::NodeHandle nh_;

  // test
  int ind_;
  bool is_doing_topic_goal_;
  bool is_wating_for_reaching_goal_;
  bool is_task_stop_;
  std::queue<geometry_msgs::PoseStamped> goal_vector_;
  std::vector<Point2D> param_goal_vector_;
  static const int kSleepTime_;
  static const std::string kNewGoalSubName_;
  static const std::string kNewGoalStampedSubName_;
  static const std::string kCancelGoalSubName_;
  static const std::string kGoalSequenceKey_;
  static const std::string kActionLibServername_;
  static const std::string kGoalFrameId_;

  boost::shared_ptr<boost::thread> GoalSendingThread_;
  boost::shared_ptr<boost::thread> AsioThread_;
  boost::mutex mtx_;
  boost::mutex mtx_notify_;
  boost::mutex mtx_task_notify_;
  boost::condition_variable_any cond_;
  boost::condition_variable_any task_cond_;
  boost::asio::io_service ioService_;
  WorkPtr workPtr_;

  ActionClient* action_client_;
  ros::Subscriber new_goal_sub_;
  ros::Subscriber new_goal_stamped_sub_;
  ros::Subscriber cancel_goal_sub_;
};

#endif  // INCLUDE_DECISION_MANEGER_GOAL_MANAGER_H_
