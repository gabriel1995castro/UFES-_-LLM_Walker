#ifndef __ADMITTANCE_CONTROLLER_NODE_H_
#define __ADMITTANCE_CONTROLLER_NODE_H_

// Includes
#include "rclcpp/rclcpp.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include <message_filters/time_synchronizer.h>    

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32.hpp>

#define WHEELS_SEPARATION 0.814f
#define MAX_LIN_VEL 0.6 // m/s
#define MIN_LIN_VEL 0.0  // m/s
#define MAX_ANG_VEL ((2*MAX_LIN_VEL)/WHEELS_SEPARATION)  // rad/s 
#define MIN_ANG_VEL -((2*MAX_LIN_VEL)/WHEELS_SEPARATION) // rad/s 

using namespace std;

class admittance_controller_class : public rclcpp::Node
{
    public:
        admittance_controller_class();
        ~admittance_controller_class();

    private:
        float mv, mw, dv, dw;
        float lin_acc, lin_vel_last, ang_acc, ang_vel_last = 0;
        float lin_acc_filter, ang_acc_filter = 0;
        float v_current, w_current = 0;

        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr dv_Sub;
        void dv_Callback(const std_msgs::msg::Float32::SharedPtr dv_aux);

        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr dw_Sub;
        void dw_Callback(const std_msgs::msg::Float32::SharedPtr dw_aux);

        message_filters::Subscriber<geometry_msgs::msg::WrenchStamped> sub1_;
        message_filters::Subscriber<nav_msgs::msg::Odometry> sub2_;

        // std::shared_ptr<message_filters::TimeSynchronizer<geometry_msgs::msg::WrenchStamped, nav_msgs::msg::Odometry>> sync_;
        std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<geometry_msgs::msg::WrenchStamped, nav_msgs::msg::Odometry>>> sync_sub_;

        void callback(const geometry_msgs::msg::WrenchStamped::ConstSharedPtr& msg1, const nav_msgs::msg::Odometry::ConstSharedPtr& msg2) ;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;

        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_subscription_;
        rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters);
};

#endif