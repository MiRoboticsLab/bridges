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
  void SetFirefunction(std::function<void(size_t)> fun)
  {
    fire_me_ = fun;
  }
  virtual void RemoveRequest() = 0;
  virtual void CallFeedbackWithLatestValue() = 0;

protected:
  void fireMe()
  {
    if (fire_me_) {
      fire_me_(goal_hash_);
    }
  }
  std::function<void(size_t)> fire_me_ {nullptr};
  size_t goal_hash_ {0};
  std::shared_mutex request_mutex_;
  std::shared_ptr<std::condition_variable> cv_ptr_ {std::make_shared<std::condition_variable>()};
  std::shared_ptr<std::mutex> result_mutex_ptr_ {std::make_shared<std::mutex>()};  // for cv_ptr_
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
    std::unique_lock<std::shared_mutex> wite_lock(request_mutex_);
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
    FeedbackCallback<ActionType> manager_feedback_cb)
  {
    typename rclcpp_action::Client<ActionType>::SendGoalOptions goal_options;
    goal_options.feedback_callback = manager_feedback_cb;
    goal_options.result_callback = std::bind(
      &ActionTask<ActionType>::resultCB, this, std::placeholders::_1);
    auto goal_handle_future = client_ptr->async_send_goal(goal, goal_options);
    if (goal_handle_future.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
      return 0;
    }
    goal_handle_ptr_ = goal_handle_future.get();
    if (!goal_handle_ptr_) {
      return 0;
    }
    goal_hash_ = getHash(goal_handle_ptr_);
    return goal_hash_;
  }
  void CallFeedback(
    typename rclcpp_action::ClientGoalHandle<ActionType>::SharedPtr goal_handle_ptr,
    const std::shared_ptr<const typename ActionType::Feedback> feedback_ptr)
  {
    std::shared_lock<std::shared_mutex> read_lock(request_mutex_);
    if (!feedback_cb_) {  // there is no requests receiving feedback
      return;
    }
    latest_feedback_value_ = feedback_ptr;
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

private:
  void resultCB(
    const typename rclcpp_action::ClientGoalHandle<ActionType>::WrappedResult & result_wrapper)
  {
    switch (result_wrapper.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
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
    {
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
    fireMe();
  }
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
    std::unique_lock<std::shared_mutex> wite_lock(request_mutex_);
    feedback_cb_ = nullptr;
  }
  FeedbackCallback<ActionType> feedback_cb_ {nullptr};
  std::shared_ptr<std::shared_ptr<typename ActionType::Result>> action_result_ptr_ptr_ {nullptr};
  std::atomic_bool got_result_ {false};
  typename rclcpp_action::ClientGoalHandle<ActionType>::SharedPtr
    goal_handle_ptr_ {nullptr};
  std::shared_ptr<const typename ActionType::Feedback> latest_feedback_value_;
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
    action_task_ptr->SetFirefunction(
      std::bind(&ActionTaskManager::KillTask, this, std::placeholders::_1));
    size_t goal_hash = action_task_ptr->SendGoal(
      client, goal, std::bind(
        &ActionTaskManager::feedbackCB<ActionType>, this,
        std::placeholders::_1, std::placeholders::_2));
    if (goal_hash != 0) {
      std::unique_lock<std::shared_mutex> write_lock(action_map_mutex_);
      action_tasks_[goal_hash] = action_task_ptr;
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

private:
  template<typename ActionType>
  void feedbackCB(
    typename rclcpp_action::ClientGoalHandle<ActionType>::SharedPtr goal_handle,
    const std::shared_ptr<const typename ActionType::Feedback> fb)
  {
    std::shared_lock<std::shared_mutex> read_lock(action_map_mutex_);
    auto action_task_itr = action_tasks_.find(getHash(goal_handle));
    if (action_task_itr == action_tasks_.end()) {  // action has finished
      return;
    }
    std::shared_ptr<ActionTask<ActionType>> action_task_ptr =
      std::static_pointer_cast<ActionTask<ActionType>>(action_task_itr->second);
    action_task_ptr->CallFeedback(goal_handle, fb);
  }
  std::map<size_t, std::shared_ptr<ActionTaskBase>> action_tasks_;
  std::shared_mutex action_map_mutex_;
};
}  // namespace bridges
}  // namespace cyberdog
#endif  // ACTION_TASK_HPP_
