// Copyright (c) 2022 Xiaomi Corporation
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

#ifndef ACTION_TASK_HPP_
#define ACTION_TASK_HPP_

#include <shared_mutex>
#include <functional>
#include <memory>
#include <condition_variable>
#include <map>
#include <mutex>
#include <atomic>
#include <list>
#include <utility>

#include "rclcpp_action/rclcpp_action.hpp"
#include "cyberdog_common/cyberdog_log.hpp"

namespace cyberdog
{
namespace bridges
{
template<typename GHP>
size_t getHash(GHP goal_handle_ptr)
{
  std::hash<rclcpp_action::GoalUUID> goal_id_hash_fun;
  return goal_id_hash_fun(goal_handle_ptr->get_goal_id());
}

template<typename ActionType>
using FeedbackCallback =
  std::function<
  void (
    typename rclcpp_action::ClientGoalHandle<ActionType>::SharedPtr,
    const std::shared_ptr<const typename ActionType::Feedback>)>;

template<typename ActionType>
using ResultCallback =
  std::function<
  void (
    const typename rclcpp_action::ClientGoalHandle<ActionType>::WrappedResult &)>;

class ActionTaskBase
{
public:
  virtual ~ActionTaskBase()
  {
    cv_ptr_->notify_all();
  }
  void SetConditionVariable(std::shared_ptr<std::condition_variable> & cv_ptr)
  {
    cv_ptr = cv_ptr_;
  }
  void SetResultMutex(std::shared_ptr<std::mutex> & mux)
  {
    mux = result_mutex_ptr_;
  }
  virtual void RemoveRequest() = 0;
  virtual void CallFeedbackWithLatestValue() = 0;
  virtual void CallFeedbackBeforeAcception() = 0;
  void SetGoalHash(size_t gh)
  {
    goal_hash_ = gh;
  }
  size_t GetGoalHash() const
  {
    return goal_hash_;
  }

protected:
  size_t goal_hash_ {0};
  std::mutex request_mutex_;
  std::shared_ptr<std::condition_variable> cv_ptr_ {std::make_shared<std::condition_variable>()};
  std::shared_ptr<std::mutex> result_mutex_ptr_ {std::make_shared<std::mutex>()};  // for cv_ptr_
  bool ready_ {false};  // receive goal_handle, accepted
};

template<typename ActionType>
class ActionTask : public ActionTaskBase
{
public:
  ~ActionTask() override
  {
    addFakeResult();  // force quit
  }
  void SetFeedbackCallback(
    FeedbackCallback<ActionType> feedback_cb)
  {
    std::unique_lock<std::mutex> wite_lock(request_mutex_);
    feedback_cb_ = feedback_cb;
  }
  void SetResultPtr(std::shared_ptr<std::shared_ptr<typename ActionType::Result>> & result_pp)
  {
    std::unique_lock<std::mutex> lock(*result_mutex_ptr_);
    if (!got_result_) {
      if (!action_result_ptr_ptr_) {
        action_result_ptr_ptr_ =
          std::make_shared<std::shared_ptr<typename ActionType::Result>>(nullptr);
      } else {
        action_result_ptr_ptr_->reset();
      }
    }
    result_pp = action_result_ptr_ptr_;
  }
  size_t SendGoal(
    typename rclcpp_action::Client<ActionType>::SharedPtr client_ptr,
    const typename ActionType::Goal & goal,
    FeedbackCallback<ActionType> manager_feedback_cb,
    ResultCallback<ActionType> manager_result_cb,
    bool * not_rec_acc)
  {
    typename rclcpp_action::Client<ActionType>::SendGoalOptions goal_options;
    goal_options.feedback_callback = manager_feedback_cb;
    goal_options.result_callback = manager_result_cb;
    auto goal_handle_future = client_ptr->async_send_goal(goal, goal_options);
    if (goal_handle_future.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
      WARN("Not receive action goal handle response!");
      *not_rec_acc = true;
      return 0;
    }
    goal_handle_ptr_ = goal_handle_future.get();
    if (!goal_handle_ptr_) {
      WARN("Task was rejected!");
      return 0;
    }
    goal_hash_ = getHash(goal_handle_ptr_);
    if (goal_hash_ == 0) {
      ERROR("Hash of GoalHandle is zero!");
    } else {
      INFO("Hash of GoalHandle is %lu", goal_hash_);
    }
    return goal_hash_;
  }
  void CallFeedback(
    typename rclcpp_action::ClientGoalHandle<ActionType>::SharedPtr goal_handle_ptr,
    const std::shared_ptr<const typename ActionType::Feedback> feedback_ptr)
  {
    std::unique_lock<std::mutex> write_lock(request_mutex_);
    latest_feedback_value_ = feedback_ptr;
    if (!ready_) {
      INFO("Receive feedback before acception.");
      feedback_buff_.push_back(std::make_pair(goal_handle_ptr, feedback_ptr));
      return;
    }
    if (!feedback_cb_) {  // there is no requests receiving feedback
      return;
    }
    feedback_cb_(goal_handle_ptr, feedback_ptr);
  }
  void RemoveRequest() override
  {
    removeFeedbackCallback();
    addFakeResult();
    cv_ptr_->notify_all();
  }
  void CallFeedbackWithLatestValue() override
  {
    if (!latest_feedback_value_) {
      return;
    }
    CallFeedback(goal_handle_ptr_, latest_feedback_value_);
  }
  void CallFeedbackBeforeAcception() override
  {
    std::unique_lock<std::mutex> write_lock(request_mutex_);
    ready_ = true;
    if (feedback_buff_.empty()) {
      INFO("No feedback comes before acception.");
      return;
    }
    for (auto & fb : feedback_buff_) {
      if (!feedback_cb_) {  // there is no requests receiving feedback
        return;
      }
      feedback_cb_(fb.first, fb.second);
    }
    INFO("Sending all feedbacks before acception.");
    feedback_buff_.clear();
  }
  void ResultCB(
    const typename rclcpp_action::ClientGoalHandle<ActionType>::WrappedResult & result_wrapper)
  {
    switch (result_wrapper.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        INFO("Goal was succeeded");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        WARN("Goal was aborted");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        WARN("Goal was canceled");
        break;
      default:
        ERROR("Unknown result code");
        break;
    }
    removeFeedbackCallback();
    std::unique_lock<std::mutex> lock(*result_mutex_ptr_);
    if (!result_wrapper.result || !action_result_ptr_ptr_) {
      action_result_ptr_ptr_ =
        std::make_shared<std::shared_ptr<typename ActionType::Result>>(
        new typename ActionType::Result());  // fake result
    } else {
      *action_result_ptr_ptr_ = result_wrapper.result;
    }
    got_result_ = true;
  }

private:
  void addFakeResult()
  {
    std::unique_lock<std::mutex> lock(*result_mutex_ptr_);
    if (!action_result_ptr_ptr_) {
      action_result_ptr_ptr_ =
        std::make_shared<std::shared_ptr<typename ActionType::Result>>(
        new typename ActionType::Result());  // fake result
    } else if (!(*action_result_ptr_ptr_)) {
      action_result_ptr_ptr_->reset(new typename ActionType::Result());  // fake result
    }
  }
  void removeFeedbackCallback()
  {
    std::unique_lock<std::mutex> write_lock(request_mutex_);
    feedback_cb_ = nullptr;
  }
  FeedbackCallback<ActionType> feedback_cb_ {nullptr};
  std::shared_ptr<std::shared_ptr<typename ActionType::Result>> action_result_ptr_ptr_ {nullptr};
  bool got_result_ {false};
  typename rclcpp_action::ClientGoalHandle<ActionType>::SharedPtr
    goal_handle_ptr_ {nullptr};
  std::shared_ptr<const typename ActionType::Feedback> latest_feedback_value_ {nullptr};
  std::list<std::pair<typename rclcpp_action::ClientGoalHandle<ActionType>::SharedPtr,
    const std::shared_ptr<const typename ActionType::Feedback>>> feedback_buff_;
  LOGGER_MINOR_INSTANCE("ActionTask");
};

class ActionTaskManager
{
public:
  template<typename ActionType>
  size_t StartActionTask(
    typename rclcpp_action::Client<ActionType>::SharedPtr client,
    const typename ActionType::Goal & goal,
    FeedbackCallback<ActionType> feedback_cb,
    std::shared_ptr<std::condition_variable> & cv_ptr,
    std::shared_ptr<std::shared_ptr<typename ActionType::Result>> & result_pp,
    std::shared_ptr<std::mutex> & mx)
  {
    auto action_task_ptr = std::make_shared<ActionTask<ActionType>>();
    action_task_ptr->SetFeedbackCallback(feedback_cb);
    action_task_ptr->SetConditionVariable(cv_ptr);
    action_task_ptr->SetResultPtr(result_pp);
    action_task_ptr->SetResultMutex(mx);
    std::unique_lock<std::shared_mutex> write_lock(action_map_mutex_);
    INFO("Sending action goal");
    size_t goal_hash = action_task_ptr->SendGoal(
      client, goal, std::bind(
        &ActionTaskManager::feedbackCB<ActionType>, this,
        std::placeholders::_1, std::placeholders::_2),
      std::bind(
        &ActionTaskManager::resultCB<ActionType>, this, std::placeholders::_1),
      &not_rec_acc_);
    if (goal_hash != 0 || not_rec_acc_) {
      INFO("goalhandle is available, registering...");
      action_tasks_[goal_hash] = action_task_ptr;
      if (not_rec_acc_ && goal_hash == 0) {
        auto rec_acc = [&]() {return !not_rec_acc_;};
        if (goal_handle_absent_cv_.wait_for(write_lock, std::chrono::seconds(3), rec_acc)) {
          goal_hash = action_tasks_[0]->GetGoalHash();
        } else {
          not_rec_acc_ = false;
        }
        action_tasks_.erase(0);
      }
    }
    return goal_hash;
  }
  void KillTask(size_t goal_hash)
  {
    std::unique_lock<std::shared_mutex> write_lock(action_map_mutex_);
    action_tasks_.erase(goal_hash);
  }
  bool RemoveRequest(size_t goal_hash)
  {
    std::shared_lock<std::shared_mutex> read_lock(action_map_mutex_);
    auto action_task_itr = action_tasks_.find(goal_hash);
    if (action_task_itr == action_tasks_.end()) {  // action has finished
      return false;
    }
    action_task_itr->second->RemoveRequest();
    return true;
  }
  template<typename ActionType>
  bool AccessTask(
    size_t goal_hash,
    FeedbackCallback<ActionType> feedback_cb,
    std::shared_ptr<std::condition_variable> & cv_ptr,
    std::shared_ptr<std::shared_ptr<typename ActionType::Result>> & result_pp,
    std::shared_ptr<std::mutex> & mx)
  {
    std::shared_lock<std::shared_mutex> read_lock(action_map_mutex_);
    auto action_task_itr = action_tasks_.find(goal_hash);
    if (action_task_itr == action_tasks_.end()) {  // action has finished
      return false;
    }
    std::shared_ptr<ActionTask<ActionType>> action_task_ptr =
      std::static_pointer_cast<ActionTask<ActionType>>(action_task_itr->second);
    action_task_ptr->SetFeedbackCallback(feedback_cb);
    action_task_ptr->SetConditionVariable(cv_ptr);
    action_task_ptr->SetResultPtr(result_pp);
    action_task_ptr->SetResultMutex(mx);
    return true;
  }
  void CallLatestFeedback(size_t goal_hash)
  {
    std::shared_lock<std::shared_mutex> read_lock(action_map_mutex_);
    auto action_task_itr = action_tasks_.find(goal_hash);
    if (action_task_itr == action_tasks_.end()) {  // action has finished
      return;
    }
    action_task_itr->second->CallFeedbackWithLatestValue();
  }
  void CallFeedbackBeforeAcception(size_t goal_hash)
  {
    std::shared_lock<std::shared_mutex> read_lock(action_map_mutex_);
    auto action_task_itr = action_tasks_.find(goal_hash);
    if (action_task_itr == action_tasks_.end()) {  // action has finished
      return;
    }
    action_task_itr->second->CallFeedbackBeforeAcception();
  }

private:
  template<typename ActionType>
  void feedbackCB(
    typename rclcpp_action::ClientGoalHandle<ActionType>::SharedPtr goal_handle,
    const std::shared_ptr<const typename ActionType::Feedback> fb)
  {
    std::unique_lock<std::shared_mutex> write_lock(action_map_mutex_);
    auto action_task_itr = action_tasks_.find(getHash(goal_handle));
    if (action_task_itr == action_tasks_.end()) {
      if (!not_rec_acc_) {  // action has finished
        WARN("No action required exists!");
        return;
      } else if (action_tasks_.find(0) != action_tasks_.end()) {  // GoalHandle not receive
        not_rec_acc_ = false;
        WARN("Receive a feedback to get goal hash!");
        size_t gh = getHash(goal_handle);
        action_tasks_[0]->SetGoalHash(gh);
        action_tasks_[gh] = action_tasks_[0];
        action_task_itr = action_tasks_.find(gh);
        goal_handle_absent_cv_.notify_one();
      }
    }
    std::shared_ptr<ActionTask<ActionType>> action_task_ptr =
      std::static_pointer_cast<ActionTask<ActionType>>(action_task_itr->second);
    INFO("calling feedback");
    action_task_ptr->CallFeedback(goal_handle, fb);
  }
  size_t getHashFromGoalID(rclcpp_action::GoalUUID goal_id)
  {
    std::hash<rclcpp_action::GoalUUID> goal_id_hash_fun;
    return goal_id_hash_fun(goal_id);
  }
  template<typename ActionType>
  void resultCB(
    const typename rclcpp_action::ClientGoalHandle<ActionType>::WrappedResult & result_wrapper)
  {
    std::unique_lock<std::shared_mutex> write_lock(action_map_mutex_);
    size_t gh = getHashFromGoalID(result_wrapper.goal_id);
    auto action_task_itr = action_tasks_.find(gh);
    if (action_task_itr == action_tasks_.end()) {
      WARN("No action required exists!");
      return;
    }
    std::shared_ptr<ActionTask<ActionType>> action_task_ptr =
      std::static_pointer_cast<ActionTask<ActionType>>(action_task_itr->second);
    INFO("calling result callback");
    action_task_ptr->ResultCB(result_wrapper);
    action_tasks_.erase(gh);
  }
  std::map<size_t, std::shared_ptr<ActionTaskBase>> action_tasks_;
  std::shared_mutex action_map_mutex_;
  bool not_rec_acc_ {false};
  std::condition_variable_any goal_handle_absent_cv_;
  LOGGER_MINOR_INSTANCE("ActionTaskManager");
};
}  // namespace bridges
}  // namespace cyberdog
#endif  // ACTION_TASK_HPP_
