#include <chrono>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/header.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

class FakeImuPublisher : public rclcpp::Node
{
  private:
    static constexpr const char* NODE_NAME = "fake_imu_publisher";
    static constexpr const char* CMD_VEL_TOPIC = "/cmd_vel";
    static constexpr const char* IMU_TOPIC = "/imu/data";
    static constexpr const char* FRAME_ID = "ego_racecar/base_link";
    static constexpr int QOS_DEPTH = 10;
    static constexpr auto IMU_PUBLISH_PERIOD = 20ms;  // 50 Hz

  public:
    FakeImuPublisher():
        Node(NODE_NAME),
        _prev_vx(0.0),
        _prev_vy(0.0),
        _yaw(0.0)
    {
        _subscription
            = this->create_subscription<geometry_msgs::msg::Twist>(CMD_VEL_TOPIC,
                                                                   QOS_DEPTH,
                                                                   [this](const geometry_msgs::msg::Twist::SharedPtr msg)
                                                                   {
                                                                       this->twist_callback(msg);
                                                                   });

        _imu_publisher = this->create_publisher<sensor_msgs::msg::Imu>(IMU_TOPIC, QOS_DEPTH);

        _prev_time = this->now();

        // Timer to publish IMU data periodically
        _timer = this->create_wall_timer(IMU_PUBLISH_PERIOD,
                                         [this]()
                                         {
                                             this->publish_imu();
                                         });
    }

  private:
    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        _last_twist = *msg;
        _last_twist_time = this->now();
    }

    void publish_imu()
    {
        rclcpp::Time now = this->now();
        double dt = (now - _prev_time).seconds();
        if (dt <= 0.0)
            return;

        double vx = _last_twist.linear.x;
        double vy = _last_twist.linear.y;
        double wz = _last_twist.angular.z;

        // Finite difference acceleration
        double ax = (vx - _prev_vx) / dt;
        double ay = (vy - _prev_vy) / dt;

        // Integrate yaw
        _yaw += wz * dt;

        // Convert to quaternion
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, _yaw);
        q.normalize();

        // IMU message
        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header.stamp = now;
        imu_msg.header.frame_id = FRAME_ID;

        // Orientation
        imu_msg.orientation = tf2::toMsg(q);
        imu_msg.orientation_covariance = {0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001};

        // Angular velocity
        imu_msg.angular_velocity.x = 0.0;
        imu_msg.angular_velocity.y = 0.0;
        imu_msg.angular_velocity.z = wz;
        imu_msg.angular_velocity_covariance = {0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001};

        // Linear acceleration
        imu_msg.linear_acceleration.x = ax;
        imu_msg.linear_acceleration.y = ay;
        imu_msg.linear_acceleration.z = 0.0;
        imu_msg.linear_acceleration_covariance = {0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001};

        _imu_publisher->publish(imu_msg);

        // Update previous values
        _prev_vx = vx;
        _prev_vy = vy;
        _prev_time = now;
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _subscription;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr _imu_publisher;
    rclcpp::TimerBase::SharedPtr _timer;

    geometry_msgs::msg::Twist _last_twist;
    rclcpp::Time _last_twist_time;

    double _prev_vx, _prev_vy, _yaw;
    rclcpp::Time _prev_time;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FakeImuPublisher>());
    rclcpp::shutdown();
    return 0;
}
