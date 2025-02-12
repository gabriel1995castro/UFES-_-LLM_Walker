// Llamado de encabezado de progama cpp
#include "vwalker_hri/multi_admitance_controller_node.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

// Inicializacion de main de ejecucion de nodo
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto admittance_controller = std::make_shared<AdmittanceControllerClass>();
    rclcpp::spin(admittance_controller->get_node_base_interface());

    rclcpp::shutdown();
    return 0;
}

// Inicializacion de clase de ejcucion de nodo
AdmittanceControllerClass::AdmittanceControllerClass()
: rclcpp::Node ("admittance_controller")
{
    // Declaracion de parametros de variables
    this->declare_parameter<float>("mv", 3);
    this->declare_parameter<float>("mw", 1);
    this->declare_parameter<float>("dv", 90);
    this->declare_parameter<float>("dw", 25);
    this->declare_parameter<float>("dw", 25);
    this->declare_parameter<bool>("validation_flag", false);

    // Definciion de valores parametros
    this->get_parameter("mv", mv);
    this->get_parameter("mw", mw);
    this->get_parameter("dv", dv);
    this->get_parameter("dw", dw);
    this->get_parameter("validation_flag", validation_flag_);

    // Seccion de asignacion de subscriptores configurados
    sub1_.subscribe(this, "force/decomposed");
    sub2_.subscribe(this, "odom");

    // Seccion de subscriptores adicionales
    dv_Sub = create_subscription<std_msgs::msg::Float32>(
        "admittance_modulator/dv_admittance", 100, std::bind(&AdmittanceControllerClass::dv_Callback, this, _1));
    dw_Sub = create_subscription<std_msgs::msg::Float32>(
        "admittance_modulator/dw_admittance", 100, std::bind(&AdmittanceControllerClass::dw_Callback, this, _1));
    theta_Sub = create_subscription<std_msgs::msg::Float32>(
        "orientation_error", 100, std::bind(&AdmittanceControllerClass::theta_Callback, this, _1));
    parameter_subscription_ = this->add_on_set_parameters_callback(std::bind(&AdmittanceControllerClass::parametersCallback, this, _1));

    // asignacion de publicadores adicionales
    pub_ = create_publisher<geometry_msgs::msg::Twist>("admittance_control/cmd_vel", 10);
    virtual_torque_publisher  = create_publisher<std_msgs::msg::Float32>("VirtualTorque", 10);
    validation_publisher_ = create_publisher<std_msgs::msg::Bool>("/admittance_modulator/validation", 10);

    // Asignacion de sincronizacion entre subscriptores principales
    sync_sub_ = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<geometry_msgs::msg::WrenchStamped, nav_msgs::msg::Odometry>>>(10);
    sync_sub_->connectInput(sub1_, sub2_);
    sync_sub_->registerCallback(std::bind(&AdmittanceControllerClass::callback, this, _1, _2));

    // Creacion de servicio
    validation_service_server_ = create_service<std_srvs::srv::SetBool>(
        "/admittance_modulator/set_validation",
        std::bind(&AdmittanceControllerClass::set_validation_callback, this, std::placeholders::_1, std::placeholders::_2));

    // asignacion de variables adicionales
    timer_ = this->create_wall_timer(20ms, std::bind(&AdmittanceControllerClass::timer_callback, this));
    flag_timer_ = create_wall_timer(50ms, std::bind(&AdmittanceControllerClass::flag_timer_callback, this));
}

// Destructor de clase 
AdmittanceControllerClass::~AdmittanceControllerClass()
{
    RCLCPP_INFO(this->get_logger(),"Shutting down");
}

// Clase de definicion de control de admitancia 
void AdmittanceControllerClass::theta_Callback(const std_msgs::msg::Float32::SharedPtr theta_aux){
	theta = theta_aux->data;
}
// Clase de defincion de timer de control de admitancia
void AdmittanceControllerClass::timer_callback()
{
    std_msgs::msg::Float32 VT_msg;
    VT_msg.data = VirtualTorque;
    virtual_torque_publisher->publish(VT_msg);
}

// Ejecucion de publicacion deestado actual de bandera de validaicon
void AdmittanceControllerClass::flag_timer_callback()
{
    std_msgs::msg::Bool msg;
    msg.data = validation_flag_;
    validation_publisher_->publish(msg);
}

// Callback de service client apra actualizacion de valor de servicio
void AdmittanceControllerClass::set_validation_callback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    // Actualiza el flag con el valor solicitado en la petición
    validation_flag_ = request->data;
    
    response->success = true;
    response->message = "Validation flag set to " + std::string(validation_flag_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "Validation flag updated: %s", (validation_flag_ ? "true" : "false"));
}

void AdmittanceControllerClass::callback(const geometry_msgs::msg::WrenchStamped::ConstSharedPtr& msg1, const nav_msgs::msg::Odometry::ConstSharedPtr& msg2) 
{
    float dt = 0.01;
    if (!(validation_flag_)){
        VirtualTorque = msg1->wrench.torque.z*9.8; 
    } else{
        VirtualTorque = -(2.0*k*tanh(theta)*DIST)/2;
    }

    // Aceleração linear
    lin_acc = (msg2->twist.twist.linear.x - lin_vel_last)/dt;
    lin_vel_last = msg2->twist.twist.linear.x;

    // Aceleracao angular
    ang_acc = (msg2->twist.twist.angular.z - ang_vel_last)/dt;
    ang_vel_last = msg2->twist.twist.angular.z;

    lin_acc_filter = lin_acc;
    ang_acc_filter = ang_acc;

    // Admittance Response Linear
    v_current = (msg1->wrench.force.x*9.8 - mv*lin_acc_filter)/dv;
    if(v_current >= MAX_LIN_VEL){ 
        v_current = MAX_LIN_VEL;
    }else if(v_current <= MIN_LIN_VEL){
        v_current = MIN_LIN_VEL;
    } 

    w_current = (VirtualTorque - mw*ang_acc_filter)/dw;    
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

void AdmittanceControllerClass::dv_Callback(const std_msgs::msg::Float32::SharedPtr dv_aux){
    dv = dv_aux->data;
}

void AdmittanceControllerClass::dw_Callback(const std_msgs::msg::Float32::SharedPtr dw_aux){
    dw = dw_aux->data;
}

rcl_interfaces::msg::SetParametersResult AdmittanceControllerClass::parametersCallback(const std::vector<rclcpp::Parameter> &parameters)
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