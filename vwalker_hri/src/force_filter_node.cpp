#include "vwalker_hri/force_filter_node.h"
using std::placeholders::_1;

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto force_filter = std::make_shared<force_filter_class>();
    rclcpp::spin(force_filter->get_node_base_interface());

    rclcpp::shutdown();
    return 0;
}
 

force_filter_class::force_filter_class()
: rclcpp::Node ("force_filter")
{
    this->declare_parameter<bool>("assymetric_force", false);
    this->get_parameter("assymetric_force", assymetric_force);

    force_filter_publisher = create_publisher<geometry_msgs::msg::WrenchStamped>("force/decomposed", 10);
    Force_Sub = create_subscription<geometry_msgs::msg::WrenchStamped>(
      "force/raw", 100, std::bind(&force_filter_class::Force_Callback, this, _1));

    parameter_subscription_ = this->add_on_set_parameters_callback(std::bind(&force_filter_class::parametersCallback, this, _1));
}    

force_filter_class::~force_filter_class()
{
    RCLCPP_INFO(this->get_logger(),"Shutting down");
}

void force_filter_class::Force_Callback(const geometry_msgs::msg::WrenchStamped::SharedPtr force_aux)
{
    geometry_msgs::msg::WrenchStamped result;
    // result.header.stamp = rclcpp::Node::now();
    result.header.stamp = force_aux->header.stamp;

    

    // Normalization Fx/Fz
    if(assymetric_force){
        force_left  = force_aux->wrench.force.x/force_aux->wrench.force.z;
        force_right = force_aux->wrench.torque.x/force_aux->wrench.torque.z;
    }else{
        force_left  = force_aux->wrench.force.x;
        force_right = force_aux->wrench.torque.x;
    }

    // Convertendo valores de forças (direita e esquerda) no eixo x para força e torque resultante 
    result.wrench.force.x  = K1*(force_left + force_right)/2;       // Força resultante
    result.wrench.torque.z = K2*(force_left - force_right)*DIST/2;  // Torque resultante

    force_filter_publisher->publish(result);
}

rcl_interfaces::msg::SetParametersResult force_filter_class::parametersCallback(const std::vector<rclcpp::Parameter> &parameters)
{
    auto result = rcl_interfaces::msg::SetParametersResult();
    result.successful = true;

    for (const auto &parameter : parameters){
        if (parameter.get_name() == "assymetric_force"){
            assymetric_force = parameter.as_bool();
        }
    }

    return result;
}