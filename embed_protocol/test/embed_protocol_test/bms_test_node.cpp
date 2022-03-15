#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "embed_protocol/embed_protocol.hpp"


namespace cyberdog {
namespace test {


typedef struct CanProtocolBmsType 
{
    uint16_t  batt_volt;
    int16_t   batt_curr;
    uint8_t   batt_soc;
    int16_t   batt_temp;
    uint8_t  batt_st;
    uint8_t  key_val;
    uint8_t  disable_charge;
    uint8_t  power_supply;
    uint8_t  buzze;
    uint8_t  status;
    int8_t   batt_health;
    int16_t   batt_loop_number;
    int8_t   powerboard_status;
    uint8_t enable_on;
    uint8_t enable_off;
} CanProtocolBmsType; 

enum Command
{
    kBuzze,
    kPowerSupply,
    kDisableCharge
};

class BMSProcessor
{
public:
    BMSProcessor(rclcpp::Logger l) : logger_(l)
    {
        auto func = [this]() 
        {
            std::string path = std::string(PASER_PATH) + "/can_protocal_toml/battery_simulator.toml";
            bms_can_ = std::make_shared<embed::Protocol<CanProtocolBmsType>>(path, false);
            bms_can_->LINK_VAR(bms_can_->GetData()->enable_on);
            bms_can_->SetDataCallback(std::bind(&BMSProcessor::HandleBMSMessages, this, 
                std::placeholders::_1, std::placeholders::_2));

            while (!stop_) 
            {
                std::unique_lock<std::mutex> lock(data_mutex_);
                data_cond_.wait(lock, [this] {return stop_ || enable_;});
                lock.unlock();

                while (enable_) 
                {
                    // bms_can_->Operate("ultrasonic_data_clock", u_data_time);
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                }
            }
        };

        thread_ = std::make_unique<std::thread>(func);
    }


    bool SendCommand(const Command& command)
    {
        bool commmand_send_success = false;
        if (command == Command::kBuzze) {
             RCLCPP_INFO(logger_, "[BmsProcessor]: %s", "command type = Command::kBuzze");
            bms_can_->BREAK_VAR(bms_can_->GetData()->buzze);
            commmand_send_success = bms_can_->Operate("cmd_buzze", std::vector<uint8_t>{0x00});
            bms_can_->LINK_VAR(bms_can_->GetData()->buzze);
        } 
        else if (command == Command::kPowerSupply) {
             RCLCPP_INFO(logger_,"[BmsProcessor]: %s", "Send command type : Command::kPowerSupply");
            bms_can_->BREAK_VAR(bms_can_->GetData()->power_supply);
            commmand_send_success = bms_can_->Operate("cmd_power_supply", std::vector<uint8_t>{0x00});
            bms_can_->LINK_VAR(bms_can_->GetData()->power_supply);
        } 
        else if (command == Command::kDisableCharge) {
             RCLCPP_INFO(logger_,"[BmsProcessor]: %s", "Send command type : Command::kDisableCharge");
            bms_can_->BREAK_VAR(bms_can_->GetData()->disable_charge);
            commmand_send_success = bms_can_->Operate("cmd_disable_charge", std::vector<uint8_t>{0x00});
            bms_can_->LINK_VAR(bms_can_->GetData()->disable_charge);
        } 
        return commmand_send_success;
    }

    ~BMSProcessor()
    {
        stop_ = true;
        enable_ = false;
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            data_cond_.notify_one();
        }
        thread_->join();
    }
    
private:
    void HandleBMSMessages(std::string& name, std::shared_ptr<CanProtocolBmsType> data)
    {
        bms_data_ptr_ = data;

        // 数据解析在这里进行
        if (name == "enable_on") {
            RCLCPP_INFO(logger_, "I heard name:%s", name.c_str());
            bms_can_->BREAK_VAR(bms_can_->GetData()->enable_on);
            bms_can_->Operate("enable_on_ack", std::vector<uint8_t>{0x00});
            enable_ = true;
            bms_can_->LINK_VAR(bms_can_->GetData()->enable_off);
        } 
        else if (name == "enable_off") {
            RCLCPP_INFO(logger_, "I heard name:%s", name.c_str());
            bms_can_->BREAK_VAR(bms_can_->GetData()->enable_off);
            bms_can_->Operate("enable_off_ack", std::vector<uint8_t>{0x00});
            enable_ = false;
            bms_can_->LINK_VAR(bms_can_->GetData()->enable_on);
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
    std::shared_ptr<embed::Protocol<CanProtocolBmsType>> bms_can_;
    std::shared_ptr<CanProtocolBmsType> bms_data_ptr_;


    std::mutex data_mutex_;
    std::condition_variable data_cond_;
    volatile bool enable_;
    bool stop_;
}; 

class BMSCanNode : public rclcpp::Node
{
public:
    BMSCanNode() : Node("bms_sensor_simulator_node")
    {
        bms_processor_ = std::make_shared<BMSProcessor>(this->get_logger());
        simulation_subscription_ = this->create_subscription<std_msgs::msg::UInt16>("bms_processor_simulation", 10,
            [this](std_msgs::msg::UInt16::SharedPtr msg) 
            {
                RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg->data);
            });

        command_subscription_ = this->create_subscription<std_msgs::msg::UInt16>("bms_processor_simulation", 10,
            [this](std_msgs::msg::UInt16::SharedPtr msg) 
            {
                RCLCPP_INFO(this->get_logger(), "BMSProcessor command send : '%d'", msg->data);
                if (msg->data == 1) {
                    bms_processor_->SendCommand(Command::kBuzze);
                }
                else if (msg->data == 2) {
                    bms_processor_->SendCommand(Command::kPowerSupply);
                }
                else if (msg->data == 3) {
                    bms_processor_->SendCommand(Command::kDisableCharge);
                }
            });
    }

private:
    rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr simulation_subscription_;
    rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr command_subscription_;
    std::shared_ptr<BMSProcessor> bms_processor_;
};

}
}


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<cyberdog::test::BMSCanNode>());
    rclcpp::shutdown();
    return 0;
}