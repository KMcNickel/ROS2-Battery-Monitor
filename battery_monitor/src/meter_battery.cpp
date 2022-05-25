#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <iostream>
#include <memory>
#include <thread>
#include <type_traits>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/int16.hpp"

#include "battery_monitor/rs232.h"

#define TX_BUFFER_SIZE 64
#define RX_BUFFER_SIZE 512
#define COM_PORT_POLL_TIMER_DURATION 100
#define FLOW_CONTROL_ENABLED 0
#define START_BYTE 0x1C

class BatteryMeter : public rclcpp_lifecycle::LifecycleNode
{
    public:
        explicit BatteryMeter()
        : LifecycleNode("BatteryMeter")
        {
            RCLCPP_INFO(rclcpp::get_logger("Constructor"), "Node Created");
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State &)
        {
            RCLCPP_INFO(rclcpp::get_logger("on_configure"), "Configuring...");

            this->declare_parameter<std::string>("port_name");
            this->declare_parameter<std::int32_t>("baud_rate", 115200);
            this->declare_parameter<std::string>("port_mode", "8N1");

            this->get_parameter("port_name", portName);
            this->get_parameter("baud_rate", baudRate);
            this->get_parameter("port_mode", portMode);

            portNum = RS232_GetPortnr(portName.c_str());

            if(portNum == -1)
            {
                RCLCPP_FATAL(rclcpp::get_logger("on_configure"), "Unable to find serial port");
                return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
            }

            RCLCPP_INFO(rclcpp::get_logger("on_configure"), "Attempting to open port: %s\n\tNumber: %d", portName.c_str(), portNum);
            if(RS232_OpenComport(portNum, baudRate, portMode.c_str(), FLOW_CONTROL_ENABLED))
            {
                RCLCPP_FATAL(rclcpp::get_logger("on_configure"), "Failed to open serial port");
                return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
            }
            RCLCPP_INFO(rclcpp::get_logger("on_configure"), "Successfully opened serial port");

            createInterfaces();

            comPortPollTimer = rclcpp::create_timer(this, get_clock(), std::chrono::milliseconds(COM_PORT_POLL_TIMER_DURATION),
                    std::bind(&BatteryMeter::comPortTimerCallback, this));

            RCLCPP_INFO(rclcpp::get_logger("on_configure"), "Configuration Complete");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State &)
        {
            RCLCPP_INFO(rclcpp::get_logger("on_activate"), "Activating...");

            SoCPublisher->on_activate();
            voltagePublisher->on_activate();

            RCLCPP_INFO(rclcpp::get_logger("on_activate"), "Activation completed successfully");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State &)
        {
            RCLCPP_INFO(rclcpp::get_logger("on_deactivate"), "Deactivating...");

            SoCPublisher->on_deactivate();
            voltagePublisher->on_deactivate();

            RCLCPP_INFO(rclcpp::get_logger("on_deactivate"), "Deactivation completed successfully");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_cleanup(const rclcpp_lifecycle::State &)
        {
            RCLCPP_INFO(rclcpp::get_logger("on_cleanup"), "Cleaning Up...");

            resetVariables();
            
            RCLCPP_INFO(rclcpp::get_logger("on_cleanup"), "Cleanup completed successfully");

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_shutdown(const rclcpp_lifecycle::State &)
        {
            RCLCPP_INFO(rclcpp::get_logger("on_shutdown"), "Shutting Down...");

            resetVariables();

            RCLCPP_INFO(rclcpp::get_logger("on_shutdown"), "Shut down completed successfully");

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        ~BatteryMeter()
        {
            RCLCPP_INFO(rclcpp::get_logger("Destructor"), "Shutting Down");

            resetVariables();
            
            RCLCPP_INFO(rclcpp::get_logger("Destructor"), "Shutdown complete. Goodbye!");
        }

    private:
        void createInterfaces()
        {
            SoCPublisher = this->create_publisher<std_msgs::msg::Int16>("battery/output/soc", 10);
            voltagePublisher = this->create_publisher<std_msgs::msg::Int16>("battery/output/voltage", 10);
        }

        void resetVariables()
        {
            comPortPollTimer.reset();
            SoCPublisher.reset();
            voltagePublisher.reset();

            if(portNum != -1)
            {
                RCLCPP_INFO(rclcpp::get_logger("resetVariables"), "Closing port");
                RS232_CloseComport(portNum);
            }
        }

        void comPortTimerCallback()
        {
            std_msgs::msg::Int16 voltageMsg, socMsg;

            count = RS232_PollComport(portNum, receiveBuffer + count, 1);

            if(receiveBuffer[count] == START_BYTE)
                count = 0;
            if(count == 4)
            {
                voltageMsg.data = receiveBuffer[1] << 8 & receiveBuffer[2];
                socMsg.data = receiveBuffer[3] << 8 & receiveBuffer[4];

                SoCPublisher->publish(socMsg);
                voltagePublisher->publish(voltageMsg);

                count = 0;
            }
        }

        rclcpp::TimerBase::SharedPtr comPortPollTimer;
        rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Int16>::SharedPtr SoCPublisher;
        rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Int16>::SharedPtr voltagePublisher;

        std::string portName;
        int portNum;
        int baudRate;
        std::string portMode;
        unsigned char receiveBuffer[5];
        int count;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor exe;

    std::shared_ptr<BatteryMeter> lc_node =
        std::make_shared<BatteryMeter>();

    exe.add_node(lc_node->get_node_base_interface());

    exe.spin();
  rclcpp::shutdown();
    return 0;
}