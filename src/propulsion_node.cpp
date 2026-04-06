#include <chrono>
#include <functional>
#include <memory>
#include <string>

// Include the ROS 2 C++ client library
#include "rclcpp/rclcpp.hpp"
// Include the standard string message type library
#include "std_msgs/msg/string.hpp"
// Include the library for data representations of movement 
#include "geometry_msgs/msg/pose_stamped.hpp"
// Include the library to handle boolean arithmetic
#include "std_msgs/msg/string.hpp"

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
            parachute_subscriber_ = this->create_subscription<std_msgs::msg::Bool>
            (
                "cmd_parachute",
                10,
                [this](const std_msgs::msg::Bool::SharedPtr msg)
                {
                    this->parachute_deployed = msg->data;

                    if (this->parachute_dedployed == true)
                    {
                        RCLCPP_INFO(this->get_logger(), "Recieved parachute deployment status from flight computer!");
                    }
                    
                }
            );

            this->declare_parameter("thrust_N", 200.0);          // Thrust is in newtons
            this->declare_parameter("burn_rate_kg/s", 1.0);     // burn rate is in kg/s
            this->declare_parameter("dry_mass_kg", 5.0);        // The dry mass is in kg
            this->declare_parameter("fuel_mass_kg", 10.0);       // The fual mass is in kg

            // The thrust MUST support the gravitational force (m*g) for any movement to occur.

            // Adding varaibles to introduce drag; F = (1/2)(rho)(velocity^2)(C_d)(Area)
            // As velocity increases the force to counteract thrust also increases.
            this->declare_parameter("air_density", 1.225);      // Rho, 1.225 kg/m^3
            this->declare_parameter("drag_coefficient", 0.5);   // C_d
            this->declare_parameter("area", 0.01);              // This is area in meters^2

            
            current_mass_ = this->get_parameter("dry_mass_kg").as_double() + this->get_parameter("fuel_mass_kg").as_double();
            altitude_ = 0.0;
            velocity_ = 0.0;

            timer_= this->create_wall_timer(10ms, std::bind(&PropulsionSimNode::update_physics, this));

            RCLCPP_INFO(this->get_logger(), "Propulsion simulation made with ROS2 nodes!!!");    
        }
    private:
        bool parachute_deployed = false;
        rclcpp::Subcription<std_msgs::msg::Bool>::SharedPtr parachute_subscriber_;




        void update_physics()
        {
            double dt = 0.01;       // b/c of 10ms increments
            double g = 9.81;
            double dry_mass = this->get_parameter("dry_mass_kg").as_double();
            double thrust_force = this->get_parameter("thrust_N").as_double();
            double m_dot = this->get_parameter("burn_rate_kg/s").as_double();        // mass flow rate
            
            // Adding variables to include drag force
            double rho = this->get_parameter("air_density").as_double();
            double C_d = this->get_parameter("drag_coefficient").as_double();
            double A = this->get_parameter("area").as_double();

            // Check if there is fuel
            double current_thrust = 0.0;
            double drag_force = 0.0;
            double net_force = 0.0;
            double current_C_d = C_d;
            if (parachute_deployed_ == true)
            {
                current_C_d = 2.5;       // Higher drag coefficient b/c parachute is now deployed
            }
            
            if (current_mass_ > dry_mass)
            {
                current_thrust = thrust_force;



                current_mass_ -= m_dot * dt;     // Consume fuel
            }
            else        // Where engine runes out of fuel
            {
                current_thrust = 0.0;
                current_mass_ = dry_mass;       // Out of fuel
            }

            // F = m*a => a = F/m
            // Also Acceleration = thrust / mass - g
            // Adding net force to inlcude other sources of force. i.e. drag force
            drag_force = (0.5)*rho*(velocity_)*(velocity_)*(current_C_d)*A;
            net_force = current_thrust - drag_force - (current_mass_ * g);
            
            double acceleration = net_force / current_mass_;

            // Numerical integration using euler (oiler)
            //v = v + a dt and y = y + v dt
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
            message.header.frame_id = "TestFrame";

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
