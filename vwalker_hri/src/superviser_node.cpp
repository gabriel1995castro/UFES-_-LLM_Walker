  #include "vwalker_hri/superviser_node.h"

  using std::placeholders::_1;

  int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto superviser = std::make_shared<superviser_class>();
    rclcpp::spin(superviser->get_node_base_interface());

    rclcpp::shutdown();
    return 0;
}

superviser_class::superviser_class()
: rclcpp::Node ("superviser")
{
	this->declare_parameter<float>("user/weight", 60);
	this->declare_parameter<bool>("use_distance", false);
	this->declare_parameter<float>("max_distance", 0.8);

	this->get_parameter("user/weight", user_weight);
	this->get_parameter("use_distance", use_distance);
	this->get_parameter("max_distance", max_distance);

	min_force_z = user_weight*0.05;

	Cmd_Vel_Sub = create_subscription<geometry_msgs::msg::Twist>(
      "admittance_control/cmd_vel", 100, std::bind(&superviser_class::Cmd_Vel_Callback, this, _1));

	Force_Sub = create_subscription<geometry_msgs::msg::WrenchStamped>(
      "force/raw", 100, std::bind(&superviser_class::Force_Callback, this, _1));

	Distance_Sub = create_subscription<std_msgs::msg::Float32>(
      "leg_monitoring/distance", 100, std::bind(&superviser_class::Distance_Callback, this, _1));

	pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

}    

superviser_class::~superviser_class()
{
    RCLCPP_INFO(this->get_logger(),"Shutting down");
}

void superviser_class::Force_Callback(const geometry_msgs::msg::WrenchStamped::SharedPtr force_aux){
	force_left_z  = force_aux->wrench.force.z;
	force_right_z = force_aux->wrench.torque.z;
}

void superviser_class::Distance_Callback(const std_msgs::msg::Float32::SharedPtr distance_aux){
	distance = distance_aux->data;
}

void superviser_class::Cmd_Vel_Callback(const geometry_msgs::msg::Twist::SharedPtr twist_aux){
	// 
    geometry_msgs::msg::Twist cmd_vel;
    
	// Stop conditions
	if(force_left_z < min_force_z || force_right_z < min_force_z || (use_distance && (distance > max_distance))){
		cmd_vel.linear.x = 0;
    	cmd_vel.angular.z = 0;
	}else{
		cmd_vel.linear.x  = twist_aux->linear.x;
    	cmd_vel.angular.z = twist_aux->angular.z;
	}

	pub_->publish(cmd_vel); 
}