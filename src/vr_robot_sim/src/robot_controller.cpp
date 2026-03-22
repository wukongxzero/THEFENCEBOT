#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mutex>

class RobotController : public rclcpp::Node {
public:
    RobotController() : Node("robot_controller") {
        auto qos = rclcpp::QoS(10).best_effort();
        subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/vr_pose", qos,
            std::bind(&RobotController::poseCallback, this, std::placeholders::_1));

        // 500 Hz control loop
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(2),
            std::bind(&RobotController::controlLoop, this));

        RCLCPP_INFO(this->get_logger(), "Robot controller ready");
    }

private:
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_pose_ = msg;
    }

    void controlLoop() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!latest_pose_) return;

        auto& pos = latest_pose_->pose.position;
        auto& ori = latest_pose_->pose.orientation;

        // Placeholder — IK goes here later
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "EE target: [%.3f, %.3f, %.3f]", pos.x, pos.y, pos.z);
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::PoseStamped::SharedPtr latest_pose_;
    std::mutex mutex_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotController>());
    rclcpp::shutdown();
}