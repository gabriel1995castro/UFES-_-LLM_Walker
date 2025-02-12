#ifndef __FORCE_FILTER_NODE_H_
#define __FORCE_FILTER_NODE_H_

// Includes
#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/wrench_stamped.hpp>

#define K1 1
#define K2 1
#define DIST 0.46 // Distancia entre sensores

using namespace std;

class force_filter_class : public rclcpp::Node
{
    public:
        force_filter_class();
        ~force_filter_class();

    private:
        bool assymetric_force;
        float force_left  = 0;
        float force_right = 0;

        rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr Force_Sub;
        void Force_Callback(const geometry_msgs::msg::WrenchStamped::SharedPtr force_aux);

        rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr force_filter_publisher;
        void Publish_Force_Filter();

        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_subscription_;
        rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters);
};

#endif