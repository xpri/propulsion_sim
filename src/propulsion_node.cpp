#include <chrono>
#include <functional>
#include <memory>
#include <string>

// Include the ROS 2 C++ client library
#include "rclcpp/rclcpp.hpp"
// Include the standard string message type library
#include "std_msgs/msg/string.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"

// To use s, ms, etc
using namespace std::chrono_literals;

class PropulsionSimNode : public rclcpp::Node
{
public:
    PropulsionSimNode()
        : Node("propulsion_sim"),
        count_(0)     //initializes node with the name "propulsion_sim"
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("rocket_pose", 10);

        this->declare_parameter("thrust_N", 15.0);        // Thrust is in newtons
        this->declare_parameter("burn_rate_kg/s", 1.0);  // burn rate is in kg/s
        this->declare_parameter("dry_mass_kg", 0.5);     // The dry mass is in kg
        this->declare_parameter("fuel_mass_kg", 1.0);    // The fual mass is in kg
        
        current_mass_ = this->get_parameter("dry_mass_kg").as_double() + this->get_parameter("fuel_mass_kg").as_double();
        altitude_ = 0.0;
        velocity_ = 0.0;

        timer_= this->create_wall_timer(10ms, std::bind(&PropulsionSimNode::update_physics, this));

        RCLCPP_INFO(this->get_logger(), "Propulsion simulation made with ROS2 nodes!!!");    
    }
private:

    void update_physics()
    {
        double dt = 0.01;
        // double g = 9.81;
        double dry_mass = this->get_parameter("dry_mass_kg").as_double();
        double thrust_force = this->get_parameter("thrust_N").as_double();
        double m_dot = this->get_parameter("burn_rate_kg/s").as_double();        // mass flow rate

        // Check if there is fuel
        double current_thrust = 0.0;
        if (current_mass_ > dry_mass)
        {
            current_thrust = thrust_force;
            current_mass_ -= m_dot * dt;     // Consume fuel
        }
        else
        {
            current_mass_ = dry_mass;       // Out of fuel
        }

        // F = m*a => a = F/m
        double acceleration = current_thrust / current_mass_;

        // Numerical integration using euler (oiler)
        velocity_ += acceleration * dt;
        altitude_ += velocity_ * dt;

        if (altitude_ < 0.0)
        {
            altitude_ = 0.0;
            velocity_ = 0.0;
        }

        publish_state();
    }

    void publish_state()
    {
        auto message = geometry_msgs::msg::PoseStamped();
        message.header.stamp = this->get_clock()->now();
        message.header.frame_id = "map";

        message.pose.position.z = altitude_;
        publisher_->publish(message);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    double altitude_, velocity_, current_mass_;
    size_t count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PropulsionSimNode>());
    rclcpp::shutdown();
    return 0;
}
