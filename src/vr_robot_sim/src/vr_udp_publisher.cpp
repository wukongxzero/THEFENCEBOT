#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <thread>
#include <atomic>
#include <cstring>

class VRUDPPublisher : public rclcpp::Node {
public:
    VRUDPPublisher() : Node("vr_udp_publisher") {
        auto qos = rclcpp::QoS(10).best_effort();
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/vr_pose", qos);

        sock_ = socket(AF_INET, SOCK_DGRAM, 0);
        sockaddr_in addr{};
        addr.sin_family      = AF_INET;
        addr.sin_port        = htons(5005);
        addr.sin_addr.s_addr = INADDR_ANY;
        bind(sock_, (sockaddr*)&addr, sizeof(addr));

        RCLCPP_INFO(this->get_logger(), "Listening for VR data on UDP port 5005");
        udp_thread_ = std::thread(&VRUDPPublisher::udpLoop, this);
    }

    ~VRUDPPublisher() {
        running_ = false;
        close(sock_);
        if (udp_thread_.joinable()) udp_thread_.join();
    }

private:
    struct Packet { float x, y, z, qx, qy, qz, qw; };

    void udpLoop() {
        Packet pkt;
        while (running_) {
            ssize_t n = recv(sock_, &pkt, sizeof(pkt), 0);
            if (n != sizeof(pkt)) continue;

            auto msg = geometry_msgs::msg::PoseStamped();
            msg.header.stamp    = this->get_clock()->now();
            msg.header.frame_id = "world";
            msg.pose.position.x    = pkt.x;
            msg.pose.position.y    = pkt.y;
            msg.pose.position.z    = pkt.z;
            msg.pose.orientation.x = pkt.qx;
            msg.pose.orientation.y = pkt.qy;
            msg.pose.orientation.z = pkt.qz;
            msg.pose.orientation.w = pkt.qw;

            publisher_->publish(msg);
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    std::thread udp_thread_;
    std::atomic<bool> running_{true};
    int sock_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VRUDPPublisher>());
    rclcpp::shutdown();
}
