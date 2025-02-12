#ifndef __SUPERVISER_NODE_H_
#define __SUPERVISER_NODE_H_

// Includes
#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32.hpp>

using namespace std;

class superviser_class : public rclcpp::Node
{
    public:
        superviser_class();
        ~superviser_class();

    private:
        float user_weight, min_force_z;
        float force_left_z, force_right_z; 
        bool use_distance;
        float max_distance, distance;

        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr Cmd_Vel_Sub;
        void Cmd_Vel_Callback(const geometry_msgs::msg::Twist::SharedPtr twist_aux);

        rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr Force_Sub;
        void Force_Callback(const geometry_msgs::msg::WrenchStamped::SharedPtr force_aux);

        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr Distance_Sub;
        void Distance_Callback(const std_msgs::msg::Float32::SharedPtr distance_aux);

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;

};


#endif