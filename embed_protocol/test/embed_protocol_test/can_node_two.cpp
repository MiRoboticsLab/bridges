// Copyright (c) 2021 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
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

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "embed_protocol/embed_protocol.hpp"

#define EVM cyberdog::embed


typedef struct _can_data {
  uint64_t u64_var;
}can_data;

typedef struct _ultrasonic_can_simulator {
  uint8_t enable_on;
  uint8_t enable_off;
}ultrasonic_can_simulator;

class CanNode : public rclcpp::Node
{
public:
  CanNode()
  : Node("can_node_two"), is_enable_(false), is_stop_(false)
  {
    subscription_ = this->create_subscription<std_msgs::msg::UInt16>(
      "topic",
      10,
      [this](std_msgs::msg::UInt16::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg->data);
      });
    auto func = [this](){
      // std::string path = std::string(PASER_PATH) + "/can_protocal_toml/test_send.toml";
      // auto pro = std::make_shared<EVM::Protocol<can_data>>(path, false);
      std::string path = std::string(PASER_PATH) + "/can_protocal_toml/ultrasonic_simulator.toml";
      auto pro = std::make_shared<EVM::Protocol<ultrasonic_can_simulator>>(path, false);
      pro->LINK_VAR(pro->GetData()->enable_on);
      pro->SetDataCallback(std::bind(&CanNode::recv_ultrasonic_callback, this, pro, std::placeholders::_1, std::placeholders::_2));    
      while(!is_stop_)
      {
        // // pro->LINK_VAR(pro->GetData()->u64_var); 
        // pro->Operate("example_data");  
        // std::this_thread::sleep_for(std::chrono::microseconds(300));      
        std::unique_lock<std::mutex> lck(data_mutex_);
        data_cond_.wait(lck, [this, &pro] {return is_enable_;});
        lck.unlock();
        uint64_t i_cnt = 0;
        while(is_enable_)
        {
          std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
          std::chrono::system_clock::duration duration_since_epoch = now.time_since_epoch(); 
          int64_t mcrosec = std::chrono::duration_cast<std::chrono::microseconds>(duration_since_epoch).count();
          std::vector<uint8_t> u_data( ((uint8_t*)&i_cnt), (((uint8_t*)&i_cnt)+7) );
          std::vector<uint8_t> u_data_time( ((uint8_t*)&mcrosec), (((uint8_t*)&mcrosec)+7) );
          pro->Operate("ultrasonic_data", u_data);
          pro->Operate("ultrasonic_data_clock", u_data_time);
          ++i_cnt;
          std::this_thread::sleep_for(std::chrono::microseconds(300));
        }         
      }
    };
    thread_ = std::make_unique<std::thread>(func);      
  }

  ~CanNode()
  {
    is_stop_ = true;
    thread_->join();
  }  

private:
  void recv_ultrasonic_callback(std::shared_ptr<EVM::Protocol<ultrasonic_can_simulator>> pro, std::string& name, std::shared_ptr<ultrasonic_can_simulator> data)
  {
    ptr_ultrasonic_simulator_ = data;
    // 数据解析在这里进行
    if(name == "enable_on")
    {
      RCLCPP_INFO(this->get_logger(), "I heard name:%s", name.c_str());
      pro->BREAK_VAR(pro->GetData()->enable_on);
      pro->Operate("enable_on_ack", std::vector<uint8_t>{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00});
      is_enable_ = true;
      pro->LINK_VAR(pro->GetData()->enable_off);
    }
    else if(name == "enable_off")
    {
      RCLCPP_INFO(this->get_logger(), "I heard name:%s", name.c_str());
      pro->BREAK_VAR(pro->GetData()->enable_off);
      pro->Operate("enable_off_ack", std::vector<uint8_t>{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00});
      is_enable_ = false;
      pro->LINK_VAR(pro->GetData()->enable_on);      
    }
    // ----------------
    {
      std::lock_guard<std::mutex> lck(data_mutex_);
      data_cond_.notify_one();
    }    
  }  

private:
  rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr subscription_;
  std::unique_ptr<std::thread> thread_;
  std::shared_ptr<ultrasonic_can_simulator> ptr_ultrasonic_simulator_;
  std::mutex data_mutex_;
  std::condition_variable data_cond_;  
  volatile bool is_enable_;
  bool is_stop_;  
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CanNode>());
  rclcpp::shutdown();
  return 0;

  return 0;
}