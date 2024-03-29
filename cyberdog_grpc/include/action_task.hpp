// Copyright (c) 2023 Xiaomi Corporation
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
#include <unordered_map>
#include <mutex>
#include <atomic>
#include <list>
#include <utility>
#include <queue>

#include "rclcpp_action/rclcpp_action.hpp"
#include "cyberdog_common/cyberdog_log.hpp"

namespace cyberdog
{
namespace bridges
{
/**
 * @brief Get the Hash of goal handle
 * @tparam GHP Type of GoalHandle
 * @param goal_handle_ptr Goal handle
 * @return size_t Hash
 */
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

/**
 * @brief Base of ActionTask
 */
class ActionTaskBase
{
public:
  /**
   * @brief Destroy the Action Task Base
   */
  virtual ~ActionTaskBase()
  {
    cv_ptr_->notify_all();
  }
  /**
   * @brief Set the condition variable for the result
   * @param cv_ptr Condition variable pointer for the result
   */
  void SetConditionVariable(std::shared_ptr<std::condition_variable> & cv_ptr)
  {
    cv_ptr = cv_ptr_;
  }
  /**
   * @brief Set the mutex for the result
   * @param mux Mutex pointer for the result
   */
  void SetResultMutex(std::shared_ptr<std::mutex> & mux)
  {
    mux = result_mutex_ptr_;
  }
  /**
   * @brief Unbind the request (from grpc) with the task
   */
  virtual void RemoveRequest() = 0;
  /**
   * @brief Call feedback with latest received value
   */
  virtual void CallFeedbackWithLatestValue() = 0;
  /**
   * @brief Call all feedback values that came before acception received
   */
  virtual void CallFeedbackBeforeAcception() = 0;
  /**
   * @brief Set the goal handle hash
   * @param gh Hash of goal handle
   */
  void SetGoalHash(size_t gh)
  {
    goal_hash_ = gh;
  }
  /**
   * @brief Get the goal handle hash
   * @return Hash of goal handle
   */
  size_t GetGoalHash() const
  {
    return goal_hash_;
  }

protected:
  size_t goal_hash_ {0};
  std::mutex request_mutex_;
  std::shared_ptr<std::condition_variable> cv_ptr_ {std::make_shared<std::condition_variable>()};
  std::shared_ptr<std::mutex> result_mutex_ptr_ {std::make_shared<std::mutex>()};  // for cv_ptr_
};

/**
 * @brief Action task for any type of action
 * @tparam ActionType Type ot action
 */
template<typename ActionType>
class ActionTask : public ActionTaskBase
{
public:
  /**
   * @brief Destroy the ActionTask
   */
  ~ActionTask() override
  {
    addFakeResult();  // force quit
  }
  /**
   * @brief Set callback function for feedback
   * @param feedback_cb Callback function
   */
  void SetFeedbackCallback(
    FeedbackCallback<ActionType> feedback_cb, bool access = false)
  {
    std::unique_lock<std::mutex> write_lock(request_mutex_);
    feedback_cb_ = feedback_cb;
    access_but_before_sending_acception_ = access;
  }
  /**
   * @brief Set result pointer
   * @param result_pp Pointer of pointer of result
   */
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
  /**
   * @brief Send an action goal
   * @param client_ptr Action client pointer
   * @param goal Aciion goal
   * @param manager_feedback_cb Feedback callback from ActionTaskManager
   * @param manager_result_cb Result callback from ActionTaskManager
   * @param not_rec_acc If action call receives response
   * @param goal_hash Goal handle hash
   * @return true Action call is accepted
   * @return false Action call is rejected or not receives response
   */
  bool SendGoal(
    typename rclcpp_action::Client<ActionType>::SharedPtr client_ptr,
    const typename ActionType::Goal & goal,
    FeedbackCallback<ActionType> manager_feedback_cb,
    ResultCallback<ActionType> manager_result_cb,
    bool & not_rec_acc, size_t & goal_hash)
  {
    typename rclcpp_action::Client<ActionType>::SendGoalOptions goal_options;
    goal_options.feedback_callback = manager_feedback_cb;
    goal_options.result_callback = manager_result_cb;
    auto goal_handle_future = client_ptr->async_send_goal(goal, goal_options);
    if (goal_handle_future.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
      WARN("Not receive action goal handle response!");
      not_rec_acc = true;
      return false;
    }
    goal_handle_ptr_ = goal_handle_future.get();
    if (!goal_handle_ptr_) {
      WARN("Task was rejected!");
      return false;
    }
    goal_hash_ = getHash(goal_handle_ptr_);
    if (goal_hash_ == 0) {
      WARN("Hash of GoalHandle is zero!");
    } else {
      INFO("Hash of GoalHandle is %lu", goal_hash_);
    }
    goal_hash = goal_hash_;
    return true;
  }
  /**
   * @brief Call feedback
   * @param goal_handle_ptr Goal handel pointer
   * @param feedback_ptr Feedback function pointer
   */
  void CallFeedback(
    typename rclcpp_action::ClientGoalHandle<ActionType>::SharedPtr goal_handle_ptr,
    const std::shared_ptr<const typename ActionType::Feedback> feedback_ptr,
    bool latest_feedback = false)
  {
    std::unique_lock<std::mutex> write_lock(request_mutex_);
    latest_feedback_value_ = feedback_ptr;
    if (!ready_) {
      INFO("Receive feedback before acception.");
      feedback_buff_.emplace_back(goal_handle_ptr, feedback_ptr);
      return;
    }
    if (feedback_cb_) {
      if (!access_but_before_sending_acception_) {
        feedback_cb_(goal_handle_ptr, feedback_ptr);
      } else if (latest_feedback) {
        feedback_cb_(goal_handle_ptr, feedback_ptr);
        access_but_before_sending_acception_ = false;
      }
    }
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
    CallFeedback(goal_handle_ptr_, latest_feedback_value_, true);
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
  /**
   * @brief Result callback
   * @param result_wrapper Action result wrapper
   */
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
  /**
   * @brief Add a fake result when result is not received
   */
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
  /**
   * @brief Clear feedback funtion pointer
   */
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
  bool ready_ {false};  // receive goal_handle, accepted
  bool access_but_before_sending_acception_ {false};
  LOGGER_MINOR_INSTANCE("ActionTask");
};

/**
 * @brief Manager to arrange ActionTasks to response the task requests from grpc
 */
class ActionTaskManager
{
public:
  /**
   * @brief Activate an action task
   * @tparam ActionType Type of action
   * @param client Action client pointer
   * @param goal Action goal
   * @param feedback_cb Feedback function from grpc
   * @param cv_ptr Condition variable pointer for action result
   * @param result_pp Pointer of pointer of action result
   * @param mx Mutex for action result
   * @param goal_hash Goal handle hash
   * @return true The task is accepted by the action server
   * @return false The task is rejected by the action server
   */
  template<typename ActionType>
  bool StartActionTask(
    typename rclcpp_action::Client<ActionType>::SharedPtr client,
    const typename ActionType::Goal & goal,
    FeedbackCallback<ActionType> feedback_cb,
    std::shared_ptr<std::condition_variable> & cv_ptr,
    std::shared_ptr<std::shared_ptr<typename ActionType::Result>> & result_pp,
    std::shared_ptr<std::mutex> & mx, size_t & goal_hash)
  {
    auto action_task_ptr = std::make_shared<ActionTask<ActionType>>();
    action_task_ptr->SetFeedbackCallback(feedback_cb);
    action_task_ptr->SetConditionVariable(cv_ptr);
    action_task_ptr->SetResultPtr(result_pp);
    action_task_ptr->SetResultMutex(mx);
    std::unique_lock<std::shared_mutex> write_lock(action_map_mutex_);
    INFO("Sending action goal");
    bool not_rec_acc = false;
    bool acception = action_task_ptr->SendGoal(
      client, goal, std::bind(
        &ActionTaskManager::feedbackCB<ActionType>, this,
        std::placeholders::_1, std::placeholders::_2),
      std::bind(
        &ActionTaskManager::resultCB<ActionType>, this, std::placeholders::_1),
      not_rec_acc, goal_hash);
    if (acception) {
      INFO("goalhandle is available, registering hash: %zu", goal_hash);
      action_tasks_[goal_hash] = action_task_ptr;
      return true;
    } else if (not_rec_acc) {
      auto rec_acc = [&]() {
          return !absent_tasks_.empty() &&
                 absent_tasks_.front().second.get() == action_task_ptr.get() &&
                 absent_tasks_.front().first;
        };
      absent_tasks_.emplace(false, action_task_ptr);
      bool wait_result = goal_handle_absent_cv_.wait_for(
        write_lock, std::chrono::seconds(3), rec_acc);
      absent_tasks_.pop();
      if (wait_result) {
        goal_hash = action_task_ptr->GetGoalHash();
        action_tasks_[goal_hash] = action_task_ptr;
        INFO("registering task from feedback, hash: %zu", goal_hash);
        return true;
      } else {
        return false;
      }
    }
    return false;
  }
  /**
   * @brief Delete task connection to action server
   * @param goal_hash Goal handle hash
   */
  void KillTask(size_t goal_hash)
  {
    std::unique_lock<std::shared_mutex> write_lock(action_map_mutex_);
    action_tasks_.erase(goal_hash);
  }
  /**
   * @brief Unbind the request with the task
   * @param goal_hash Goal handle hash
   * @return true Found and unbinded the task
   * @return false Not found the task
   */
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
  /**
   * @brief Access an existed task
   * @tparam ActionType Type of action
   * @param goal_hash Goal handle hash
   * @param feedback_cb Feedback callback function
   * @param cv_ptr Condition variable pointer for action result
   * @param result_pp Pointer of pointer of action result
   * @param mx Mutex for action result
   * @return true Found and accessed
   * @return false Not found
   */
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
    action_task_ptr->SetFeedbackCallback(feedback_cb, true);
    action_task_ptr->SetConditionVariable(cv_ptr);
    action_task_ptr->SetResultPtr(result_pp);
    action_task_ptr->SetResultMutex(mx);
    return true;
  }
  /**
   * @brief Call feedback with latest received value
   * @param goal_hash Goal handle hash
   */
  void CallLatestFeedback(size_t goal_hash)
  {
    std::shared_lock<std::shared_mutex> read_lock(action_map_mutex_);
    auto action_task_itr = action_tasks_.find(goal_hash);
    if (action_task_itr == action_tasks_.end()) {  // action has finished
      return;
    }
    action_task_itr->second->CallFeedbackWithLatestValue();
  }
  /**
   * @brief Call all feedback values that came before acception received
   * @param goal_hash Goal handle hash
   */
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
  /**
   * @brief Feedback callback of action
   * @tparam ActionType Type of action
   * @param goal_handle Goal handle hash
   * @param fb Action feedback pointer
   */
  template<typename ActionType>
  void feedbackCB(
    typename rclcpp_action::ClientGoalHandle<ActionType>::SharedPtr goal_handle,
    const std::shared_ptr<const typename ActionType::Feedback> fb)
  {
    std::unique_lock<std::shared_mutex> write_lock(action_map_mutex_);
    auto action_task_itr = action_tasks_.find(getHash(goal_handle));
    if (action_task_itr == action_tasks_.end()) {
      if (absent_tasks_.empty()) {  // action has finished
        WARN("No action required exists!");
      } else {  // GoalHandle not receive
        WARN("Receive a feedback to get goal hash!");
        size_t goal_hash = getHash(goal_handle);
        absent_tasks_.front().second->SetGoalHash(goal_hash);
        absent_tasks_.front().first = true;
        std::shared_ptr<ActionTask<ActionType>> action_task_ptr =
          std::static_pointer_cast<ActionTask<ActionType>>(absent_tasks_.front().second);
        INFO("calling feedback without acception, hash: %zu", goal_hash);
        action_task_ptr->CallFeedback(goal_handle, fb);
        goal_handle_absent_cv_.notify_all();
      }
      return;
    }
    std::shared_ptr<ActionTask<ActionType>> action_task_ptr =
      std::static_pointer_cast<ActionTask<ActionType>>(action_task_itr->second);
    INFO("calling feedback, hash: %zu", action_task_itr->first);
    action_task_ptr->CallFeedback(goal_handle, fb);
  }
  /**
   * @brief Get goal handle hash from goal UUID
   * @param goal_id Goal UUID
   * @return Goal handle hash
   */
  size_t getHashFromGoalID(rclcpp_action::GoalUUID goal_id)
  {
    std::hash<rclcpp_action::GoalUUID> goal_id_hash_fun;
    return goal_id_hash_fun(goal_id);
  }
  /**
   * @brief Result callback of action
   * @tparam ActionType ActionType Type of action
   * @param result_wrapper Action result wrapper
   */
  template<typename ActionType>
  void resultCB(
    const typename rclcpp_action::ClientGoalHandle<ActionType>::WrappedResult & result_wrapper)
  {
    std::shared_lock<std::shared_mutex> read_lock(action_map_mutex_);
    size_t gh = getHashFromGoalID(result_wrapper.goal_id);
    auto action_task_itr = action_tasks_.find(gh);
    if (action_task_itr == action_tasks_.end()) {
      WARN("No action required exists!");
      return;
    }
    std::shared_ptr<ActionTask<ActionType>> action_task_ptr =
      std::static_pointer_cast<ActionTask<ActionType>>(action_task_itr->second);
    INFO("calling result callback, hash: %zu", gh);
    action_task_ptr->ResultCB(result_wrapper);
    action_tasks_.erase(gh);
  }
  std::unordered_map<size_t, std::shared_ptr<ActionTaskBase>> action_tasks_;
  std::shared_mutex action_map_mutex_;
  std::condition_variable_any goal_handle_absent_cv_;
  std::queue<std::pair<bool, std::shared_ptr<ActionTaskBase>>> absent_tasks_;
  LOGGER_MINOR_INSTANCE("ActionTaskManager");
};
}  // namespace bridges
}  // namespace cyberdog
#endif  // ACTION_TASK_HPP_
