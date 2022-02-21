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
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "embed_protocol/embed_protocol.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"


#define EVM cyberdog::embed


typedef struct _can_data {
  uint64_t example_data;
}can_data;

typedef struct _ultrasonic_can {
  uint8_t ultrasonic_data_array[16];
  uint8_t enable_on_ack;
  uint8_t enable_off_ack;
  uint64_t ultrasonic_clock;
}ultrasonic_can;

class UltrasonicControl {
public:
  UltrasonicControl(rclcpp::Logger l):logger_(l)/*, is_stop_(false) */
  {
    // std::string path = std::string(PASER_PATH) + "/can_protocal_toml/ultrasonic.toml";
    std::string path = ament_index_cpp::get_package_share_directory("params") + "/toml_config/sensors/ultrasonic.toml";
    ptr_ultrasonic_protocol = std::make_shared<EVM::Protocol<ultrasonic_can>>(path, false);
    // ptr_ultrasonic_protocol->Operate("enable_on", std::vector<uint8_t>{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00});
    // // pro->Operate("enable_off", std::vector<uint8_t>{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00});   
    ptr_ultrasonic_protocol->LINK_VAR(ptr_ultrasonic_protocol->GetData()->enable_on_ack);
    ptr_ultrasonic_protocol->SetDataCallback(std::bind(&UltrasonicControl::recv_ultrasonic_callback, this, std::placeholders::_1, std::placeholders::_2));     
    // auto func = [this](){
    //   // std::string path = std::string(PASER_PATH) + "/can_protocal_toml/test_receive.toml";
    //   // auto pro = std::make_shared<EVM::Protocol<can_data>>(path, false);
    //   // pro->LINK_VAR(pro->GetData()->example_data);
    //   // pro->SetDataCallback(std::bind(&CanNode::recv_callback, this, std::placeholders::_1, std::placeholders::_2));      
    //   while(!is_stop_)
    //   {
    //     std::unique_lock<std::mutex> lck(data_mutex_);
    //     data_cond_.wait(lck, [this] {return (is_stop_ || (!(ptr_ultrasonic_protocol->IsRxTimeout() || ptr_ultrasonic_protocol->IsRxError())) );});
    //     lck.unlock();
    //     if(ptr_ultrasonic_protocol->IsRxTimeout() || ptr_ultrasonic_protocol->IsRxError())
    //       continue;        
    //     // RCLCPP_INFO(this->get_logger(), "I heard: '%016lx'", ptr_data_->example_data);
    //   }
    // };
    // thread_ = std::make_unique<std::thread>(func);
  }

  void Enable_On()
  {
    ptr_ultrasonic_protocol->Operate("enable_on", std::vector<uint8_t>{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00});
    ptr_ultrasonic_protocol->LINK_VAR(ptr_ultrasonic_protocol->GetData()->enable_on_ack);
  }

  void Enable_Off()
  {
    ptr_ultrasonic_protocol->BREAK_VAR(ptr_ultrasonic_protocol->GetData()->ultrasonic_data_array);
    ptr_ultrasonic_protocol->Operate("enable_off", std::vector<uint8_t>{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00});
    ptr_ultrasonic_protocol->LINK_VAR(ptr_ultrasonic_protocol->GetData()->enable_off_ack);
  }  

private:
  // void recv_callback(std::string& name, std::string& name, std::shared_ptr<can_data> data)
  // {
  //   ptr_data_ = data;
  //   {
  //     std::lock_guard<std::mutex> lck(data_mutex_);
  //     data_cond_.notify_one();
  //   }
  //   // 数据解析在这里进行
  //   RCLCPP_INFO(this->get_logger(), "I heard: '%016lx'", ptr_data_->example_data);
  // }
  void recv_ultrasonic_callback(std::string& name, std::shared_ptr<ultrasonic_can> data)
  {
    ptr_ultrasonic_data_ = data;
    // {
    //   std::lock_guard<std::mutex> lck(data_mutex_);
    //   data_cond_.notify_one();
    // }
    // 数据解析在这里进行
    if(name == "enable_on_ack")
    {
      RCLCPP_INFO(logger_, "[%s]canid name:%s, data: '%02x'", CONTROLNAME.c_str(), name.c_str(), ptr_ultrasonic_data_->enable_on_ack);
      ptr_ultrasonic_protocol->BREAK_VAR(ptr_ultrasonic_protocol->GetData()->enable_on_ack);
      ptr_ultrasonic_protocol->LINK_VAR(ptr_ultrasonic_protocol->GetData()->ultrasonic_data_array);
      // pro->LINK_VAR(pro->GetData()->ultrasonic_clock);
      // pro->LINK_VAR(pro->GetData()->enable_off_ack);
    }
    else if(name == "ultrasonic_data_array")
    {
      std::string out_put("");
      char one_data[8] = { 0 };
      for(size_t i=0 ; i<sizeof(ptr_ultrasonic_data_->ultrasonic_data_array); ++i)
      {
        snprintf(one_data, sizeof(one_data), "%02x", ptr_ultrasonic_data_->ultrasonic_data_array[i]);
        out_put += one_data;
      }
      RCLCPP_INFO(logger_, "[%s]canid name:%s, data: '%s'", CONTROLNAME.c_str(), name.c_str(), out_put.c_str());
      ptr_ultrasonic_protocol->BREAK_VAR(ptr_ultrasonic_protocol->GetData()->ultrasonic_data_array);
      ptr_ultrasonic_protocol->LINK_VAR(ptr_ultrasonic_protocol->GetData()->ultrasonic_data_array);      
    }
    else if(name == "enable_off_ack")
    {
      RCLCPP_INFO(logger_, "[%s]canid name:%s, data: '%02x'", CONTROLNAME.c_str(), name.c_str(), ptr_ultrasonic_data_->enable_off_ack);
      ptr_ultrasonic_protocol->BREAK_VAR(ptr_ultrasonic_protocol->GetData()->enable_off_ack);
    }
  }    

private:
  rclcpp::Logger logger_;
  // std::unique_ptr<std::thread> thread_;
  std::shared_ptr<EVM::Protocol<ultrasonic_can>> ptr_ultrasonic_protocol;
  std::shared_ptr<ultrasonic_can> ptr_ultrasonic_data_;
  // std::mutex data_mutex_;
  // std::condition_variable data_cond_;  
  // bool is_stop_;
  const std::string CONTROLNAME = "Ultrasonic";
};

typedef struct _tof_can {
  uint8_t tof_data_array[72];
  uint8_t enable_on_ack;
  uint8_t enable_off_ack;
  uint64_t tof_clock;
}tof_can;

class TofControl {
public:
  TofControl(rclcpp::Logger l):logger_(l)
  {
    std::string path = ament_index_cpp::get_package_share_directory("params") + "/toml_config/sensors/tof.toml";
    ptr_tof_protocol = std::make_shared<EVM::Protocol<tof_can>>(path, false);
    ptr_tof_protocol->LINK_VAR(ptr_tof_protocol->GetData()->enable_on_ack);
    ptr_tof_protocol->SetDataCallback(std::bind(&TofControl::recv_tof_callback, this, std::placeholders::_1, std::placeholders::_2));   
  }

  void Enable_On()
  {
    ptr_tof_protocol->Operate("enable_on", std::vector<uint8_t>{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00});
    ptr_tof_protocol->LINK_VAR(ptr_tof_protocol->GetData()->enable_on_ack);
  }

  void Enable_Off()
  {
    ptr_tof_protocol->BREAK_VAR(ptr_tof_protocol->GetData()->tof_data_array);
    ptr_tof_protocol->Operate("enable_off", std::vector<uint8_t>{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00});
    ptr_tof_protocol->LINK_VAR(ptr_tof_protocol->GetData()->enable_off_ack);
  }  

private:
  void recv_tof_callback(std::string& name, std::shared_ptr<tof_can> data)
  {
    ptr_tof_data_ = data;
    // 数据解析在这里进行
    if(name == "enable_on_ack")
    {
      RCLCPP_INFO(logger_, "[%s]canid name:%s, data: '%02x'", CONTROLNAME.c_str(), name.c_str(), ptr_tof_data_->enable_on_ack);
      ptr_tof_protocol->BREAK_VAR(ptr_tof_protocol->GetData()->enable_on_ack);
      ptr_tof_protocol->LINK_VAR(ptr_tof_protocol->GetData()->tof_data_array);
    }
    else if(name == "tof_data_array")
    {
      std::string out_put("");
      char one_data[8] = { 0 };
      for(size_t i=0 ; i<sizeof(ptr_tof_data_->tof_data_array); ++i)
      {
        snprintf(one_data, sizeof(one_data), "%02x", ptr_tof_data_->tof_data_array[i]);
        out_put += one_data;
      }
      RCLCPP_INFO(logger_, "[%s]canid name:%s, data: '%s'", CONTROLNAME.c_str(), name.c_str(), out_put.c_str());
      ptr_tof_protocol->BREAK_VAR(ptr_tof_protocol->GetData()->tof_data_array);
      ptr_tof_protocol->LINK_VAR(ptr_tof_protocol->GetData()->tof_data_array);      
    }
    else if(name == "enable_off_ack")
    {
      RCLCPP_INFO(logger_, "[%s]canid name:%s, data: '%02x'", CONTROLNAME.c_str(), name.c_str(), ptr_tof_data_->enable_off_ack);
      ptr_tof_protocol->BREAK_VAR(ptr_tof_protocol->GetData()->enable_off_ack);
    }
  }    

private:
  rclcpp::Logger logger_;
  std::shared_ptr<EVM::Protocol<tof_can>> ptr_tof_protocol;
  std::shared_ptr<tof_can> ptr_tof_data_;
  const std::string CONTROLNAME = "Tof";
};

class CanNode : public rclcpp::Node
{
public:
  CanNode()
  : Node("can_node_test")
  {
    subscription_ = this->create_subscription<std_msgs::msg::UInt16>(
      "can_protocol_test",
      10,
      [this](std_msgs::msg::UInt16::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg->data);
      });
    ultrasonic_control_ = std::make_unique<UltrasonicControl>(this->get_logger());
    tof_control_ = std::make_unique<TofControl>(this->get_logger());
  }

private:
  rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr subscription_;
  std::unique_ptr<UltrasonicControl> ultrasonic_control_;
  std::unique_ptr<TofControl> tof_control_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CanNode>());
  rclcpp::shutdown();
  return 0;

  return 0;
}