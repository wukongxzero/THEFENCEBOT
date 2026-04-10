#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <mutex>
#include <cstring>

class RobotController : public rclcpp::Node {
public:
    RobotController() : Node("robot_controller") {
        auto qos = rclcpp::QoS(10).best_effort();
        subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/vr_pose", qos,
            std::bind(&RobotController::poseCallback, this, std::placeholders::_1));

        sock_ = socket(AF_INET, SOCK_DGRAM, 0);
        memset(&isaac_addr_, 0, sizeof(isaac_addr_));
        isaac_addr_.sin_family      = AF_INET;
        isaac_addr_.sin_port        = htons(5006);
        inet_pton(AF_INET, "127.0.0.1", &isaac_addr_.sin_addr);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&RobotController::controlLoop, this));

        RCLCPP_INFO(this->get_logger(), "Robot controller ready, forwarding to Isaac Lab :5006");
    }

    ~RobotController() { close(sock_); }

private:
    struct Packet { float x, y, z, qx, qy, qz, qw; };

    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_pose_ = msg;
        new_data_ = true;
    }

    void controlLoop() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!latest_pose_ || !new_data_) return;
        new_data_ = false;

        auto& pos = latest_pose_->pose.position;
        auto& ori = latest_pose_->pose.orientation;

        Packet pkt{
            static_cast<float>(pos.x), static_cast<float>(pos.y), static_cast<float>(pos.z),
            static_cast<float>(ori.x), static_cast<float>(ori.y),
            static_cast<float>(ori.z), static_cast<float>(ori.w)
        };

        sendto(sock_, &pkt, sizeof(pkt), 0, (sockaddr*)&isaac_addr_, sizeof(isaac_addr_));

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Forwarding EE target: [%.3f, %.3f, %.3f]", pos.x, pos.y, pos.z);
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::PoseStamped::SharedPtr latest_pose_;
    std::mutex mutex_;
    bool new_data_ = false;
    int sock_;
    sockaddr_in isaac_addr_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotController>());
    rclcpp::shutdown();
}