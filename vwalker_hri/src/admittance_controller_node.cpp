#include "vwalker_hri/admittance_controller_node.h"

using std::placeholders::_1;
using std::placeholders::_2;

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto admittance_controller = std::make_shared<admittance_controller_class>();
    rclcpp::spin(admittance_controller->get_node_base_interface());

    rclcpp::shutdown();
    return 0;
}

admittance_controller_class::admittance_controller_class()
: rclcpp::Node ("admittance_controller")
{
    this->declare_parameter<float>("mv", 3);
    this->declare_parameter<float>("mw", 1);
    this->declare_parameter<float>("dv", 90);
    this->declare_parameter<float>("dw", 25);

    this->get_parameter("mv", mv);
    this->get_parameter("mw", mw);
    this->get_parameter("dv", dv);
    this->get_parameter("dw", dw);

    sub1_.subscribe(this, "force/decomposed");
    sub2_.subscribe(this, "odom");

    sync_sub_ = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<geometry_msgs::msg::WrenchStamped, nav_msgs::msg::Odometry>>>(10);
    sync_sub_->connectInput(sub1_, sub2_);
    sync_sub_->registerCallback(std::bind(&admittance_controller_class::callback, this, _1, _2));

    pub_ = create_publisher<geometry_msgs::msg::Twist>("admittance_control/cmd_vel", 10);

    dv_Sub = create_subscription<std_msgs::msg::Float32>(
      "admittance_modulator/dv_admittance", 100, std::bind(&admittance_controller_class::dv_Callback, this, _1));

    dw_Sub = create_subscription<std_msgs::msg::Float32>(
      "admittance_modulator/dw_admittance", 100, std::bind(&admittance_controller_class::dw_Callback, this, _1));

    parameter_subscription_ = this->add_on_set_parameters_callback(std::bind(&admittance_controller_class::parametersCallback, this, _1));
}    

admittance_controller_class::~admittance_controller_class()
{
    RCLCPP_INFO(this->get_logger(),"Shutting down");
}

void admittance_controller_class::callback(const geometry_msgs::msg::WrenchStamped::ConstSharedPtr& msg1, const nav_msgs::msg::Odometry::ConstSharedPtr& msg2) 
{
    float dt = 0.01;

    lin_acc = (msg2->twist.twist.linear.x - lin_vel_last)/dt;
    lin_vel_last = msg2->twist.twist.linear.x;

    ang_acc = (msg2->twist.twist.angular.z - ang_vel_last)/dt;
    ang_vel_last = msg2->twist.twist.angular.z;

    lin_acc_filter = lin_acc;
    ang_acc_filter = ang_acc;

    v_current = (msg1->wrench.force.x*9.8 - mv*lin_acc_filter)/dv;
    if(v_current >= MAX_LIN_VEL){ 
        v_current = MAX_LIN_VEL;
    }else if(v_current <= MIN_LIN_VEL){
        v_current = MIN_LIN_VEL;
    } 

    w_current = (msg1->wrench.torque.z*9.8 - mw*ang_acc_filter)/dw;    
    if(w_current >= MAX_ANG_VEL){  
        w_current = MAX_ANG_VEL;
    }else if(w_current <= MIN_ANG_VEL){
        w_current = MIN_ANG_VEL;
    }

    // 
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = v_current;
    cmd_vel.angular.z = -w_current;
    pub_->publish(cmd_vel); 
}

void admittance_controller_class::dv_Callback(const std_msgs::msg::Float32::SharedPtr dv_aux){
    dv = dv_aux->data;
}

void admittance_controller_class::dw_Callback(const std_msgs::msg::Float32::SharedPtr dw_aux){
    dw = dw_aux->data;
}

rcl_interfaces::msg::SetParametersResult admittance_controller_class::parametersCallback(const std::vector<rclcpp::Parameter> &parameters)
{
    auto result = rcl_interfaces::msg::SetParametersResult();
    result.successful = true;

    for (const auto &parameter : parameters){
        if (parameter.get_name() == "mv"){
            mv = parameter.as_double();
        }else if(parameter.get_name() == "mw"){
            mw = parameter.as_double();
        }
    }

    return result;
}