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

#ifndef INCLUDE_DECISION_MANAGER_TASK_EXECUTOR_H_
#define INCLUDE_DECISION_MANAGER_TASK_EXECUTOR_H_

#include <decision_manager/task.h>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/asio/io_service.hpp>

namespace decision_manager {
class TaskExecutor {
 public:
  TaskExecutor()
    : workPtr_(new boost::asio::io_service::work(ioService_)) {
    unsigned int nthreads = boost::thread::hardware_concurrency()
        / kCore_divide_factor;  // NOLINT
    while (nthreads--) {
      thread_group_.create_thread(
          boost::bind(&boost::asio::io_service::run, &ioService_));
    }
  }
  ~TaskExecutor() {
    workPtr_.reset();
    thread_group_.join_all();
    ioService_.stop();
  }
  void PostTask(const TaskPtr& taskPtr);
 private:
  static const int kCore_divide_factor;
  typedef boost::shared_ptr<boost::asio::io_service::work> WorkPtr;
  boost::asio::io_service ioService_;
  WorkPtr workPtr_;
  boost::thread_group thread_group_;
};
const int TaskExecutor::kCore_divide_factor = 4;
void TaskExecutor::PostTask(const TaskPtr& taskPtr) {
  ioService_.post(boost::bind(&Task::Run, taskPtr.get()));
}
};  // namespace decision_manager

#endif  // INCLUDE_DECISION_MANAGER_TASK_EXECUTOR_H_
