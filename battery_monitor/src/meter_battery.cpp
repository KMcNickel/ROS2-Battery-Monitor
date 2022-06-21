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
#include "std_msgs/msg/u_int16.hpp"
#include "can_interface/msg/can_frame.hpp"

using std::placeholders::_1;

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

            this->declare_parameter<std::int32_t>("can_id", 1);

            this->get_parameter("can_id", canId);

            RCLCPP_INFO(rclcpp::get_logger("on_configure"), "Using Can Id: %d", canId);

            createInterfaces();

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
            canDataSubscription = this->create_subscription<can_interface::msg::CanFrame>(
                "battery/input/can", 50, std::bind(&BatteryMeter::canDataReceived, this, _1));
            SoCPublisher = this->create_publisher<std_msgs::msg::UInt16>("battery/output/soc", 10);
            voltagePublisher = this->create_publisher<std_msgs::msg::UInt16>("battery/output/voltage", 10);
        }

        void resetVariables()
        {
            canDataSubscription.reset();
            SoCPublisher.reset();
            voltagePublisher.reset();
        }

        enum messageTypes
        {
            BATTERY_CAN_MESSAGE_TYPE_NONE = 0,
            BATTERY_CAN_MESSAGE_TYPE_VOLTAGE_MV = 1,
            BATTERY_CAN_MESSAGE_TYPE_SOC_MILLIPERCENT = 2
        };

        void canDataReceived(const can_interface::msg::CanFrame & message)
        {
            uint32_t messageType;
            std_msgs::msg::UInt16 outgoingMsg;

            if(this->get_current_state().id() != 3)     //3 is the "active" lifecycle state
            {
                RCLCPP_DEBUG(rclcpp::get_logger("canDataReceived"), "Ignoring data while in current state");
                return;
            }

            if(message.can_id >> 5 != (uint32_t) canId)
            {
                RCLCPP_DEBUG(rclcpp::get_logger("canDataReceived"), "Ignoring data that is not for this node");
                return;
            }

            messageType = message.can_id & 0b00011111;

            switch((messageTypes) messageType)
            {
                case BATTERY_CAN_MESSAGE_TYPE_VOLTAGE_MV:
                    if(message.dlc != 0)
                    {
                        RCLCPP_WARN(rclcpp::get_logger("canDataReceived"), "DLC of %d is not correct for BATTERY_CAN_MESSAGE_TYPE_VOLTAGE_MV", message.dlc);
                        return;
                    }
                    outgoingMsg.data = (uint16_t) message.data[0] | ((uint16_t) message.data[1] << 8);
                    voltagePublisher->publish(outgoingMsg);
                    break;
                case BATTERY_CAN_MESSAGE_TYPE_SOC_MILLIPERCENT:
                    if(message.dlc != 0)
                    {
                        RCLCPP_WARN(rclcpp::get_logger("canDataReceived"), "DLC of %d is not correct for BATTERY_CAN_MESSAGE_TYPE_SOC_MILLIPERCENT", message.dlc);
                        return;
                    }
                    outgoingMsg.data = (uint16_t) message.data[0] | ((uint16_t) message.data[1] << 8);
                    SoCPublisher->publish(outgoingMsg);
                    break;
                default:
                    break;
            }
        }

        rclcpp::Subscription<can_interface::msg::CanFrame>::SharedPtr canDataSubscription;

        rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::UInt16>::SharedPtr SoCPublisher;
        rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::UInt16>::SharedPtr voltagePublisher;

        uint16_t voltage, soc;
        int32_t canId;
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