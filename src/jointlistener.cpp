#include <chrono>
#include <memory>
#include <vector>
#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

class JointStateListener : public rclcpp::Node
{
public:
    JointStateListener()
    : Node("joint_state_listener")
    {
        // サブスクライバの作成
        subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/lbr/joint_states",
            10,
            std::bind(&JointStateListener::topic_callback, this, std::placeholders::_1)
        );

        // タイマーの作成（100Hz = 10ms周期）
        timer_ = this->create_wall_timer(
            10ms,
            std::bind(&JointStateListener::timer_callback, this)
        );
    }

private:
    void topic_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // 最新のposition値を保存
        last_position_ = msg->position;
    }

    void timer_callback()
    {
        // position配列を表示
        if (!last_position_.empty()) {
            std::cout << "Joint positions: ";
            for (auto v : last_position_) {
                std::cout << v << " ";
            }
            std::cout << std::endl;
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<double> last_position_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointStateListener>());
    rclcpp::shutdown();
    return 0;
}
