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

#include <ros/ros.h>
#include <string>
#include <vector>

#ifndef INCLUDE_DECISION_MANEGER_TASK_H_
#define INCLUDE_DECISION_MANEGER_TASK_H_

namespace decision_manager {
class TaskStatus{};  // TODO(FrankChen) define structure.

class Task {
 public:
     void GetTaskState(TaskStatus& status) const {
       status = taskStatus_;
     }
     void GetTaskName(std::string& name) const  {
       name = taskName_;
     }
     virtual void Run() = 0;
     virtual void Stop() = 0;
     virtual void Initialize(ros::NodeHandle n) = 0;
     virtual ~Task() {}
 protected:
     Task() {}
     void SetTaskName(std::string name) {
       taskName_ = name;
     }
     void ExecuteInner();
 private:
     std::string taskName_;
     std::vector<std::string> actions_;
     TaskStatus taskStatus_;
};
}  // namespace decision_manager
#endif  // INCLUDE_DECISION_MANEGER_TASK_H_
