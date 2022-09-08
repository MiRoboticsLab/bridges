// Copyright (c) 2021 Xiaomi Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MSG_DISPATCHER_HPP_
#define MSG_DISPATCHER_HPP_
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <queue>
#include <utility>
template<typename MessageT>
class LatestMsgDispather
{
  using SharedPtrCallback = std::function<void (std::shared_ptr<MessageT>)>;

public:
  LatestMsgDispather()
  : callback_(nullptr), need_run_(true) {}
  ~LatestMsgDispather()
  {
    need_run_ = false;
    cond_.notify_all();
    if (thread_->joinable()) {
      thread_->join();
    }
  }
  void push(const MessageT & msg)
  {
    std::unique_lock<std::mutex> lock(queue_mutex_);
    if (!need_run_) {
      return;
    }
    queue_.push(std::move(msg));
    cond_.notify_one();
  }
  template<typename CallbackT>
  void setCallback(CallbackT && callback)
  {
    callback_ = std::forward<CallbackT>(callback);
    run();
  }

private:
  void run()
  {
    thread_ = std::make_shared<std::thread>(
      &LatestMsgDispather::process_thread,
      this);
  }
  void process_thread()
  {
    while (need_run_) {
      if (callback_ != nullptr) {
        auto msg = get();
        if (msg) {
          callback_(msg);
        }
      }
    }
  }
  std::shared_ptr<MessageT> get()
  {
    std::unique_lock<std::mutex> lock(queue_mutex_);
    cond_.wait(lock, [this] {return !need_run_ || !this->queue_.empty();});
    if (!need_run_) {
      return NULL;
    }
    std::shared_ptr<MessageT> val(
      std::make_shared<MessageT>(std::move(queue_.front())));
    queue_.pop();
    return val;
  }

  bool need_run_;
  std::queue<MessageT> queue_;
  std::mutex queue_mutex_;
  std::condition_variable cond_;
  SharedPtrCallback callback_;
  std::shared_ptr<std::thread> thread_;
};
#endif  // MSG_DISPATCHER_HPP_