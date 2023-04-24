#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std;

// Global variables
float cmd_speed[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float last_cmd_speed[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float speed_sp[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float speed_pv[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float speed_last_sp[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float speed_error[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float speed_error_max = 0.0;

float sprofile = 1000.0;
float profile_step = 1.0;
float sprofile_t = sprofile;
float sprofile_T = sprofile;

float time_out = 1.0;

geometry_msgs::msg::Twist last_cmd;

rclcpp::Time current_time, last_time, time_last_cmd = rclcpp::Clock().now();

class ScurveTwistNode : public rclcpp::Node
{
public:
    ScurveTwistNode() : Node("scurve_twist_node")
    {
        RCLCPP_INFO(this->get_logger(), "scurve_twist_node started");

        // Parameters for the node
        float frequency = this->declare_parameter<float>("frequency", 60.0);
        profile_step = this->declare_parameter<float>("profile_step", 1.0);
        sprofile = this->declare_parameter<float>("sprofile", 1000.0);

        RCLCPP_INFO(this->get_logger(), "frequency: %f", frequency);
        RCLCPP_INFO(this->get_logger(), "profile_step: %f", profile_step);
        RCLCPP_INFO(this->get_logger(), "sprofile: %f", sprofile);

        // Create a publisher
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("scurve_cmd_vel", 1);

        // Create a subscriber
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel_accel_desel", 10,
            std::bind(&ScurveTwistNode::topic_twist_callback, this, std::placeholders::_1));

        // Create a timer binding to a callback function
        timer_ = this->create_wall_timer(
            std::chrono::duration<float>(1 / frequency),
            std::bind(&ScurveTwistNode::frequency_update, this));
        
        // checker_interval_ = this->create_wall_timer(
        //     std::chrono::duration<float>(1 / 30),
        //     std::bind(&ScurveTwistNode::checker_update, this));
    }

private:
    // Callback function for the subscriber twist
    void topic_twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const
    {
        current_time = last_time = rclcpp::Clock().now();
        if (last_cmd != *msg)
        {
            last_cmd = *msg;
            sprofile_t = sprofile_T;
            cmd_speed[0] = msg->linear.x;
            cmd_speed[1] = msg->linear.y;
            cmd_speed[2] = msg->linear.z;
            cmd_speed[3] = msg->angular.x;
            cmd_speed[4] = msg->angular.y;
            cmd_speed[5] = msg->angular.z;

            for (int i = 0; i < 6; i++)
            {
                speed_last_sp[i] = speed_pv[i];
                speed_sp[i] = cmd_speed[i];
                speed_error[i] = abs(speed_sp[i] - speed_pv[i]);
                if (speed_error[i] > speed_error_max)
                {
                    speed_error_max = speed_error[i];
                }
            }
            sprofile_T = (speed_error_max / sprofile) * 100.0;
            if (sprofile_T < 100)
            {
                sprofile_T = 100.0;
            }
            if (sprofile_t >= sprofile_T)
            {
                sprofile_t = 0.0;
            }
        }
    }

    float sCurves_accel_decel(float velocity, float time)
    {
        float velocity_smooth = velocity * (2 * M_PI * time / sprofile_T - sin(2 * M_PI * time / sprofile_T)) / 2 / M_PI;
        return velocity_smooth;
    }

    // Callback function for the timer
    void frequency_update()
    {
        auto scurve_vel = geometry_msgs::msg::Twist();
        current_time = rclcpp::Clock().now();
        if (sprofile_t <= sprofile_T)
        {
            for (int i = 0; i < 6; i++)
            {
                if (speed_sp[i] > speed_pv[i])
                {
                    speed_pv[i] = speed_last_sp[i] + sCurves_accel_decel(speed_error[i], sprofile_t);
                }
                else if (speed_sp[i] < speed_pv[i])
                {
                    speed_pv[i] = speed_last_sp[i] - sCurves_accel_decel(speed_error[i], sprofile_t);
                }
            }
            if (sprofile_t >= sprofile_T)
            {
                for (int i = 0; i < 6; i++)
                {
                    speed_pv[i] = speed_sp[i];
                }
            }
        }
        scurve_vel.linear.x = speed_pv[0];
        scurve_vel.linear.y = speed_pv[1];
        scurve_vel.linear.z = speed_pv[2];
        scurve_vel.angular.x = speed_pv[3];
        scurve_vel.angular.y = speed_pv[4];
        scurve_vel.angular.z = speed_pv[5];
        sprofile_t += profile_step;
        publisher_->publish(scurve_vel);
        if (current_time.seconds() - last_time.seconds() > time_out)
        {
            geometry_msgs::msg::Twist::SharedPtr vel_timeout = std::make_shared<geometry_msgs::msg::Twist>();
            topic_twist_callback(vel_timeout);
        }
    }

    // subscription_ is a shared pointer to a subscription
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    // publisher_ is a shared pointer to a publisher
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    // timer_ is a shared pointer to a timer
    rclcpp::TimerBase::SharedPtr timer_;
    // checker_interval_ is a shared pointer to a timer
    // rclcpp::TimerBase::SharedPtr checker_interval_;
};

int main(int argc, char **argv)
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);
    // Create a node
    rclcpp::spin(std::make_shared<ScurveTwistNode>());
    // Shutdown
    rclcpp::shutdown();
    return 0;
}