#include "rclcpp/rclcpp.hpp"
#include "unitree_api/msg/request.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "sport_model.hpp"
#include "nlohmann/json.hpp"

#include <chrono>
#include <memory>
#include <sstream>
#include <string>

using namespace std::placeholders;
using namespace std::chrono_literals;

class TwistBridge : public rclcpp::Node
{
public:
    TwistBridge() : Node("twist_bridge_node_cpp")
    {
        require_localization_valid_ = this->declare_parameter<bool>(
            "require_localization_valid", false);
        localization_valid_topic_ = this->declare_parameter<std::string>(
            "localization_valid_topic", "/localization/valid");
        localization_state_topic_ = this->declare_parameter<std::string>(
            "localization_state_topic", "/localization/state");
        required_localization_states_ = this->declare_parameter<std::string>(
            "required_localization_states",
            "IN_MAP_TRACKING,GLOBAL_TRACKING");
        localization_timeout_sec_ = this->declare_parameter<double>(
            "localization_timeout_sec", 1.0);
        cmd_vel_timeout_sec_ = this->declare_parameter<double>(
            "cmd_vel_timeout_sec", 0.5);
        stop_on_gate_close_ = this->declare_parameter<bool>(
            "stop_on_gate_close", true);

        RCLCPP_INFO(this->get_logger(), "TwistBridge创建，可以将geometry_msgs/msg/twist消息转换成unitree_api/msg/request消息!");
        request_pub_ = this->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);
        twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&TwistBridge::twist_cb, this, _1));

        if (require_localization_valid_) {
            auto localized_qos = rclcpp::QoS(rclcpp::KeepLast(1));
            localized_qos.transient_local().reliable();
            valid_sub_ = this->create_subscription<std_msgs::msg::Bool>(
                localization_valid_topic_, localized_qos,
                std::bind(&TwistBridge::valid_cb, this, _1));
            state_sub_ = this->create_subscription<std_msgs::msg::String>(
                localization_state_topic_, localized_qos,
                std::bind(&TwistBridge::state_cb, this, _1));
        }

        timer_ = this->create_wall_timer(
            50ms, std::bind(&TwistBridge::timer_cb, this));
    }

private:
    enum class GateResult
    {
        OPEN,
        CLOSED
    };

    void twist_cb(const geometry_msgs::msg::Twist::SharedPtr twist)
    {
        latest_twist_ = *twist;
        latest_cmd_time_ = now();
        has_cmd_vel_ = true;
    }

    void valid_cb(const std_msgs::msg::Bool::SharedPtr msg)
    {
        localization_valid_ = msg->data;
        last_valid_receive_time_ = now();
    }

    void state_cb(const std_msgs::msg::String::SharedPtr msg)
    {
        localization_state_ = msg->data;
        last_state_receive_time_ = now();
    }

    GateResult gate_result() const
    {
        if (!require_localization_valid_) {
            return GateResult::OPEN;
        }
        const auto current_time = now();
        const auto valid_age = (current_time - last_valid_receive_time_).seconds();
        const auto state_age = (current_time - last_state_receive_time_).seconds();
        if (!localization_valid_ ||
            !localization_state_allowed() ||
            valid_age > localization_timeout_sec_ ||
            state_age > localization_timeout_sec_) {
            return GateResult::CLOSED;
        }
        return GateResult::OPEN;
    }

    bool localization_state_allowed() const
    {
        std::stringstream stream(required_localization_states_);
        std::string state;
        while (std::getline(stream, state, ',')) {
            const auto first = state.find_first_not_of(" \t");
            const auto last = state.find_last_not_of(" \t");
            if (first == std::string::npos) {
                continue;
            }
            if (state.substr(first, last - first + 1) == localization_state_) {
                return true;
            }
        }
        return false;
    }

    void publish_stop()
    {
        unitree_api::msg::Request request;
        request.header.identity.api_id = ROBOT_SPORT_API_ID_BALANCESTAND;
        request_pub_->publish(request);
    }

    void publish_twist(const geometry_msgs::msg::Twist &twist)
    {
        unitree_api::msg::Request request;
        const double x = twist.linear.x;
        const double y = twist.linear.y;
        const double z = twist.angular.z;
        auto api_id = ROBOT_SPORT_API_ID_BALANCESTAND;

        if (x != 0.0 || y != 0.0 || z != 0.0)
        {
            api_id = ROBOT_SPORT_API_ID_MOVE;
            nlohmann::json js;
            js["x"] = x;
            js["y"] = y;
            js["z"] = z;
            request.parameter = js.dump();
        }
        request.header.identity.api_id = api_id;
        request_pub_->publish(request);
    }

    void timer_cb()
    {
        const auto gate = gate_result();
        if (gate != gate_open_) {
            gate_open_ = gate;
            RCLCPP_WARN(
                this->get_logger(),
                gate == GateResult::OPEN
                    ? "Localization gate opened: forwarding motion commands"
                    : "Localization gate closed: robot motion commands blocked");
        }

        if (gate == GateResult::CLOSED) {
            if (stop_on_gate_close_) {
                publish_stop();
            }
            return;
        }

        if (!has_cmd_vel_) {
            publish_stop();
            return;
        }

        const auto cmd_age = (now() - latest_cmd_time_).seconds();
        if (cmd_age > cmd_vel_timeout_sec_) {
            publish_stop();
            return;
        }
        publish_twist(latest_twist_);
    }

    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr request_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr valid_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool require_localization_valid_{false};
    bool localization_valid_{false};
    bool stop_on_gate_close_{true};
    bool has_cmd_vel_{false};
    GateResult gate_open_{GateResult::OPEN};
    std::string localization_valid_topic_;
    std::string localization_state_topic_;
    std::string required_localization_states_;
    std::string localization_state_;
    double localization_timeout_sec_{1.0};
    double cmd_vel_timeout_sec_{0.5};
    geometry_msgs::msg::Twist latest_twist_;
    rclcpp::Time latest_cmd_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_valid_receive_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_state_receive_time_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TwistBridge>());
    rclcpp::shutdown();
    return 0;
}
