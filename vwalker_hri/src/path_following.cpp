#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <vector>

using namespace std::chrono_literals;


using std::placeholders::_1;

class PathFollowing : public rclcpp::Node
{
public:
    PathFollowing() : Node("PathFollowing")
    {
        RCLCPP_INFO(this->get_logger(), "Starting Path Following - Virtual Trail");
        this->declare_parameter<std::string>("path_topic", "/reference_path");
        this->declare_parameter<std::string>("odom_topic", "/odom_fusion");
        this->declare_parameter<std::string>("theta_topic", "/orientation_error");
        this->declare_parameter<int>("control_rate", 20);

        this->get_parameter("path_topic", pathTopic);
        this->get_parameter("odom_topic", odomTopic);
        this->get_parameter("theta_topic", orientationTopic);
        this->get_parameter("control_rate", controlRate);


        subPath = this->create_subscription<nav_msgs::msg::Path>(
            "/reference_path", 10, std::bind(&PathFollowing::callbackPath, this, _1));
        subOdom = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom_amcl", 10, std::bind(&PathFollowing::callbackOdom, this, _1));

        pubTheta = this->create_publisher<std_msgs::msg::Float32>(orientationTopic, 10);

        changeOdom = false;
        changePath = false;
        vr = 0.3;
        lx = 3.0;
        kx = 0.7;
        ly = 3.0;
        ky = 0.7;
        stepPathIndex = 10;

        // mainControl();


        // Timer para publicar mensagens regularmente
        timer_ = this->create_wall_timer(
            20ms, std::bind(&PathFollowing::timer_callback, this));



    }


    void timer_callback()
    {
        if (changePath && changeOdom)
        {
            findNearestPosition();
            DesiredOrientationCalc();
            makeOrientationMsg();
        }
    }


    

    void callbackPath(const nav_msgs::msg::Path::SharedPtr msg)
    {

        xPath.clear();
        yPath.clear();
        zOriPath.clear();
        wOriPath.clear();

        for (const auto &pose : msg->poses)
        {
            xPath.push_back(pose.pose.position.x);
            yPath.push_back(pose.pose.position.y);
            zOriPath.push_back(pose.pose.orientation.z);
            wOriPath.push_back(pose.pose.orientation.w);
        }

        lengthPathMSG = msg->poses.size();
        changePath = true;
    }

    void callbackOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        xSW = msg->pose.pose.position.x;
        ySW = msg->pose.pose.position.y;
        zOriSW = msg->pose.pose.orientation.z;
        wOriSW = msg->pose.pose.orientation.w;

        tf2::Quaternion q(0, 0, zOriSW, wOriSW);
        tf2::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, thetaSW);


        changeOdom = true;
    }

    void findNearestPosition()
    {
        std::vector<double> dist;
        for (size_t i = 0; i < xPath.size(); ++i)
        {
            dist.push_back(sqrt(pow(xPath[i] - xSW, 2) + pow(yPath[i] - ySW, 2)));
        }

        xPath_min_index = std::distance(dist.begin(), std::min_element(dist.begin(), dist.end()));

        xPath_ = xPath[xPath_min_index];
        yPath_ = yPath[xPath_min_index];

        if ((xPath_min_index + stepPathIndex) < lengthPathMSG)
        {
            xPathNext_ = xPath[xPath_min_index + stepPathIndex];
            yPathNext_ = yPath[xPath_min_index + stepPathIndex];
        }
        else
        {
            xPathNext_ = xPath[lengthPathMSG - 1];
            yPathNext_ = yPath[lengthPathMSG - 1];
        }
    }

    void DesiredOrientationCalc()
    {
        xErr = xPath_ - xSW;
        yErr = yPath_ - ySW;

        if ((xPathNext_ - xPath_) == 0 && yPathNext_ > 0)
        {
            thetaPath = 1.5708;
        }
        else if ((xPathNext_ - xPath_) == 0 && yPathNext_ < 0)
        {
            thetaPath = -1.5708;
        }
        else if ((yPathNext_ - yPath_) == 0 && xPathNext_ > 0)
        {
            thetaPath = 0;
        }
        else if ((yPathNext_ - yPath_) == 0 && xPathNext_ < 0)
        {
            thetaPath = -3.14159;
        }
        else
        {
            thetaPath = atan2((yPathNext_ - yPath_), (xPathNext_ - xPath_));
        }

        xDot = vr * cos(thetaPath) + lx * tanh((kx / lx) * xErr);
        yDot = vr * sin(thetaPath) + ly * tanh((ky / ly) * yErr);

        thetad = atan2(yDot, xDot);

        thetaerr = thetad - thetaSW;
        if (thetaerr < -3.14)
        {
            thetaerr += 6.28;
        }
        else if (thetaerr > 3.14)
        {
            thetaerr -= 6.28;
        }
    }

    void makeOrientationMsg()
    {
        std_msgs::msg::Float32 float_msg;
        float_msg.data = thetaerr;
        pubTheta->publish(float_msg);

        changeOdom = false;
        changePath = false;
    }

    void mainControl()
    {
        rclcpp::Rate rate(controlRate);
        while (rclcpp::ok())
        {
            if (changePath && changeOdom)
            {
                findNearestPosition();
                DesiredOrientationCalc();
                makeOrientationMsg();
            }
            rate.sleep();
        }
    }

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subPath;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdom;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pubTheta;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr Distance_Sub;


    rclcpp::TimerBase::SharedPtr timer_;


    std::string pathTopic, odomTopic, orientationTopic;
    int controlRate;

    std::vector<double> xPath, yPath, zOriPath, wOriPath;
    double xSW, ySW, zOriSW, wOriSW, thetaSW;
    double xPath_, yPath_, xPathNext_, yPathNext_;
    double vr, lx, kx, ly, ky;
    int stepPathIndex, lengthPathMSG, xPath_min_index;
    bool changeOdom, changePath;
    double xErr, yErr, thetaPath, xDot, yDot, thetad, thetaerr;

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathFollowing>());
    rclcpp::shutdown();
    return 0;
}
