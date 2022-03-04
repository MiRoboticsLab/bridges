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


typedef struct _test_send_data {
  uint64_t u64_var;
}test_send_data;

class TestSenor
{
public:
  TestSenor(rclcpp::Logger l):logger_(l),is_stop_(false)
  {
    auto func = [this](){
      std::string path = std::string(PASER_PATH) + "/can_protocal_toml/test_send.toml";
      auto pro = std::make_shared<EVM::Protocol<test_send_data>>(path, false);
      pro->LINK_VAR(pro->GetData()->u64_var);       
      while(!is_stop_)
      {
        pro->Operate("example_data");  
        // pro->Operate("example_data_1");  
        // pro->Operate("example_data_2");  
        // std::this_thread::sleep_for(std::chrono::microseconds(300));        
        std::this_thread::sleep_for(std::chrono::seconds(10));    
      }
    };
    thread_ = std::make_unique<std::thread>(func);
  }

  ~TestSenor()
  {
    is_stop_ = true;
    thread_->join();
  }       

private:
  rclcpp::Logger logger_;
  std::unique_ptr<std::thread> thread_;
  std::shared_ptr<EVM::Protocol<test_send_data>> test_can_;
  bool is_stop_; 
};

typedef struct _ultrasonic_can_simulator {
  uint8_t enable_on;
  uint8_t enable_off;
}ultrasonic_can_simulator;

class UltrasonicSenor
{
public:
  UltrasonicSenor(rclcpp::Logger l):logger_(l), is_enable_(false), is_stop_(false)
  {
    auto func = [this](){
      std::string path = std::string(PASER_PATH) + "/can_protocal_toml/ultrasonic_simulator.toml";
      ultrasonic_can_ = std::make_shared<EVM::Protocol<ultrasonic_can_simulator>>(path, false);
      ultrasonic_can_->LINK_VAR(ultrasonic_can_->GetData()->enable_on);
      ultrasonic_can_->SetDataCallback(std::bind(&UltrasonicSenor::recv_ultrasonic_callback, this, std::placeholders::_1, std::placeholders::_2));    
      while(!is_stop_)
      {
        // // pro->LINK_VAR(pro->GetData()->u64_var); 
        // pro->Operate("example_data");  
        // std::this_thread::sleep_for(std::chrono::microseconds(300));      
        std::unique_lock<std::mutex> lck(data_mutex_);
        data_cond_.wait(lck, [this] {return is_stop_ || is_enable_;});
        lck.unlock();
        uint64_t i_cnt = 0;
        while(is_enable_)
        {
          std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
          std::chrono::system_clock::duration duration_since_epoch = now.time_since_epoch(); 
          int64_t mcrosec = std::chrono::duration_cast<std::chrono::microseconds>(duration_since_epoch).count();
          std::vector<uint8_t> u_data( ((uint8_t*)&i_cnt), (((uint8_t*)&i_cnt)+7) );
          std::vector<uint8_t> u_data_time( ((uint8_t*)&mcrosec), (((uint8_t*)&mcrosec)+7) );
          ultrasonic_can_->Operate("ultrasonic_data", u_data);
          ultrasonic_can_->Operate("ultrasonic_data_clock", u_data_time);
          ++i_cnt;
          std::this_thread::sleep_for(std::chrono::microseconds(300));
        }         
      }
    };
    thread_ = std::make_unique<std::thread>(func);      
  }

  ~UltrasonicSenor()
  {
    is_stop_ = true;
    is_enable_ = false;
    {
      std::lock_guard<std::mutex> lck(data_mutex_);
      data_cond_.notify_one();
    }        
    thread_->join();
  }     

private:
  void recv_ultrasonic_callback(std::string& name, std::shared_ptr<ultrasonic_can_simulator> data)
  {
    ptr_ultrasonic_simulator_ = data;
    // 数据解析在这里进行
    if(name == "enable_on")
    {
      RCLCPP_INFO(logger_, "I heard name:%s", name.c_str());
      ultrasonic_can_->BREAK_VAR(ultrasonic_can_->GetData()->enable_on);
      ultrasonic_can_->Operate("enable_on_ack", std::vector<uint8_t>{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00});
      is_enable_ = true;
      ultrasonic_can_->LINK_VAR(ultrasonic_can_->GetData()->enable_off);
    }
    else if(name == "enable_off")
    {
      RCLCPP_INFO(logger_, "I heard name:%s", name.c_str());
      ultrasonic_can_->BREAK_VAR(pro->GetData()->enable_off);
      ultrasonic_can_->Operate("enable_off_ack", std::vector<uint8_t>{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00});
      is_enable_ = false;
      ultrasonic_can_->LINK_VAR(ultrasonic_can_->GetData()->enable_on);      
    }
    // ----------------
    {
      std::lock_guard<std::mutex> lck(data_mutex_);
      data_cond_.notify_one();
    }    
  }   

private:
  rclcpp::Logger logger_;
  std::unique_ptr<std::thread> thread_;
  std::shared_ptr<EVM::Protocol<ultrasonic_can_simulator>> ultrasonic_can_;
  std::shared_ptr<ultrasonic_can_simulator> ptr_ultrasonic_simulator_;
  std::mutex data_mutex_;
  std::condition_variable data_cond_;  
  volatile bool is_enable_;
  bool is_stop_;  
}; // class UltrasonicSenor

typedef struct _tof_can_simulator {
  uint8_t enable_on;
  uint8_t enable_off;
}tof_can_simulator;

class TofSenor
{
public:
  TofSenor(rclcpp::Logger l):logger_(l), is_enable_(false), is_stop_(false)
  {
    auto func = [this](){
      // std::string path = std::string(PASER_PATH) + "/can_protocal_toml/test_send.toml";
      // auto pro = std::make_shared<EVM::Protocol<can_data>>(path, false);
      std::string path = std::string(PASER_PATH) + "/can_protocal_toml/tof_simulator.toml";
      tof_can_ = std::make_shared<EVM::Protocol<tof_can_simulator>>(path, false);
      tof_can_->LINK_VAR(tof_can_->GetData()->enable_on);
      tof_can_->SetDataCallback(std::bind(&TofSenor::recv_tof_callback, this, std::placeholders::_1, std::placeholders::_2));    
      while(!is_stop_)
      {
        // // pro->LINK_VAR(pro->GetData()->u64_var); 
        // pro->Operate("example_data");  
        // std::this_thread::sleep_for(std::chrono::microseconds(300));      
        std::unique_lock<std::mutex> lck(data_mutex_);
        data_cond_.wait(lck, [this] {return is_stop_ || is_enable_;});
        lck.unlock();
        uint64_t i_cnt = 0;
        while(is_enable_)
        {
          std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
          std::chrono::system_clock::duration duration_since_epoch = now.time_since_epoch(); 
          int64_t mcrosec = std::chrono::duration_cast<std::chrono::microseconds>(duration_since_epoch).count();
          std::vector<uint8_t> u_data( ((uint8_t*)&i_cnt), (((uint8_t*)&i_cnt)+7) );
          std::vector<uint8_t> u_data_time( ((uint8_t*)&mcrosec), (((uint8_t*)&mcrosec)+7) );
          tof_can_->Operate("tof_data_one", u_data);
          tof_can_->Operate("tof_data_two", u_data);
          tof_can_->Operate("tof_data_three", u_data);
          tof_can_->Operate("tof_data_four", u_data);
          tof_can_->Operate("tof_data_five", u_data);
          tof_can_->Operate("tof_data_six", u_data);
          tof_can_->Operate("tof_data_seven", u_data);
          tof_can_->Operate("tof_data_eight", u_data);
          tof_can_->Operate("tof_data_clock", u_data_time);
          ++i_cnt;
          std::this_thread::sleep_for(std::chrono::microseconds(300));
        }         
      }
    };
    thread_ = std::make_unique<std::thread>(func);      
  }

  ~TofSenor()
  {
    is_stop_ = true;
    is_enable_ = false;
    {
      std::lock_guard<std::mutex> lck(data_mutex_);
      data_cond_.notify_one();
    }        
    thread_->join();
  }     

private:
  void recv_tof_callback(std::string& name, std::shared_ptr<tof_can_simulator> data)
  {
    ptr_tof_simulator_ = data;
    // 数据解析在这里进行
    if(name == "enable_on")
    {
      RCLCPP_INFO(logger_, "I heard name:%s", name.c_str());
      tof_can_->BREAK_VAR(tof_can_->GetData()->enable_on);
      tof_can_->Operate("enable_on_ack", std::vector<uint8_t>{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00});
      is_enable_ = true;
      tof_can_->LINK_VAR(tof_can_->GetData()->enable_off);
    }
    else if(name == "enable_off")
    {
      RCLCPP_INFO(logger_, "I heard name:%s", name.c_str());
      tof_can_->BREAK_VAR(pro->GetData()->enable_off);
      tof_can_->Operate("enable_off_ack", std::vector<uint8_t>{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00});
      is_enable_ = false;
      tof_can_->LINK_VAR(tof_can_->GetData()->enable_on);      
    }
    // ----------------
    {
      std::lock_guard<std::mutex> lck(data_mutex_);
      data_cond_.notify_one();
    }    
  }   

private:
  rclcpp::Logger logger_;
  std::unique_ptr<std::thread> thread_;
  std::shared_ptr<EVM::Protocol<tof_can_simulator>> tof_can_;
  std::shared_ptr<tof_can_simulator> ptr_tof_simulator_;
  std::mutex data_mutex_;
  std::condition_variable data_cond_;  
  volatile bool is_enable_;
  bool is_stop_;  
}; // class TofSenor

class CanNode : public rclcpp::Node
{
public:
  CanNode()
  : Node("sensor_simulator_node")
  {
    test_ = std::make_unique<TestSenor>(this->get_logger());
    // ultrasonic_ = std::make_unique<UltrasonicSenor>(this->get_logger());
    // tof_ = std::make_unique<TofSenor>(this->get_logger());
    // subscription_ = this->create_subscription<std_msgs::msg::UInt16>(
    //   "sensor_simulator",
    //   10,
    //   [this](std_msgs::msg::UInt16::SharedPtr msg) {
    //     RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg->data);
    //     if(msg->data == 0)
    //     {
    //     }
    //   });
  }

private:
  // rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr subscription_;
  std::unique_ptr<TestSenor> test_;
  std::unique_ptr<UltrasonicSenor> ultrasonic_;
  std::unique_ptr<TofSenor> tof_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CanNode>());
  rclcpp::shutdown();
  return 0;
}