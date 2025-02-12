// Header guard to prevent multiple inclusions of this file
#ifndef __MULTI_ADMITANCE_CONTROLLER_NODE_HPP__
#define __MULTI_ADMITANCE_CONTROLLER_NODE_HPP__

// Include necessary ROS2 libraries for node functionality
#include "rclcpp/rclcpp.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include <message_filters/time_synchronizer.h>   
// Include message types used for communication with other ROS nodes
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
// funciones para ejecucion de servicios
#include <std_srvs/srv/set_bool.hpp> 
// Include standard C++ libraries for math operations and I/O
#include <stdio.h>
#include <math.h>

// constanst for node class
#define WHEELS_SEPARATION 0.814f
#define MAX_LIN_VEL 0.6 // m/s
#define MIN_LIN_VEL 0.0  // m/s
#define MAX_ANG_VEL ((2*MAX_LIN_VEL)/WHEELS_SEPARATION)  // rad/s 
#define MIN_ANG_VEL -((2*MAX_LIN_VEL)/WHEELS_SEPARATION) // rad/s 
#define DIST 0.46

// Use standard namespace (avoid using `using namespace std;` in header files to prevent polluting the global namespace)
using namespace std;
using namespace std::chrono_literals;

// Definition of the ROS2 node class
class AdmittanceControllerClass :
    public rclcpp::Node
    {
    public:
        // Constructor: Initializes the node
        AdmittanceControllerClass();
        // Destructor: Cleans up resources when the node is destroyed
        ~AdmittanceControllerClass();

    private:
        // Variables for storing system states and calculations
        float mv, mw, dv, dw, theta, VirtualTorque;
        float lin_acc, lin_vel_last, ang_acc, ang_vel_last = 0;
        float lin_acc_filter, ang_acc_filter = 0;
        float v_current, w_current = 0;
        float k = 30; // Gain factor for admittance control
        bool validation_flag_ = false; // Variable de validacion de configuracion

        // Publishers for sending processed data to other ROS nodes
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr virtual_torque_publisher;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr validation_publisher_; // Publicador de configuracion actual de controlador

        // Subscribers for receiving input data
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr dv_Sub;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr dw_Sub;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr theta_Sub;

        // Callback functions to handle incoming subscriber messages
        void dv_Callback(const std_msgs::msg::Float32::SharedPtr dv_aux);
        void dw_Callback(const std_msgs::msg::Float32::SharedPtr dw_aux);
        void theta_Callback(const std_msgs::msg::Float32::SharedPtr theta_Sub_aux);
        void callback(const geometry_msgs::msg::WrenchStamped::ConstSharedPtr& msg1, const nav_msgs::msg::Odometry::ConstSharedPtr& msg2);

        // Message filters for synchronizing sensor data
        message_filters::Subscriber<geometry_msgs::msg::WrenchStamped> sub1_;
        message_filters::Subscriber<nav_msgs::msg::Odometry> sub2_;
        // Synchronizer to process sensor messages together based on approximate timestamps
        std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<geometry_msgs::msg::WrenchStamped, nav_msgs::msg::Odometry>>> sync_sub_;

        // Seccion de definicion de servicios
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr validation_service_server_;

        // Timer callback function for periodic execution
        void timer_callback();
        void flag_timer_callback();

        // Parameter handling for dynamic configuration
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_subscription_;
        rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters);
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::TimerBase::SharedPtr flag_timer_;

        // Creacion de callback de servicio
        void set_validation_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
            std::shared_ptr<std_srvs::srv::SetBool::Response> response);
};

#endif