#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <iostream>
class HzMonitor
{

public:
    HzMonitor(ros::NodeHandle &nh_)
    {
        ROS_INFO("Constructor initialized");
        std::string current_topic_ = "/robot_pose";
        ros::Subscriber topic_sub_ = nh_.subscribe(current_topic_, 5000, &HzMonitor::msg_callback, this);
        ros::Subscriber robotPose  = nh_.subscribe("/robot_pose", 1, &HzMonitor::robot_cb, this);
    }
    class DummyMsg
    {
    public:
        static std::string md5;
        static std::string data_type;
        static std::string const &__s_getMD5Sum() { return md5; }
        static std::string const &__s_getDataType() { return data_type; }
        void deserialize(void *) {}
    };
    void controlFlow();
    void msg_callback(const DummyMsg &data);
    void robot_cb(const geometry_msgs::Pose &robot_msg);
    double pose_x;
    double pose_y;
  
};
std::string HzMonitor::DummyMsg::md5 = "*";

std::string HzMonitor::DummyMsg::data_type = "/";

void HzMonitor::msg_callback(const DummyMsg &data)
{
  ROS_INFO("call back");
}
void HzMonitor::robot_cb(const geometry_msgs::Pose &robot_msg)  
{
  ROS_INFO("ROS call back");
  pose_y = pose_x  = robot_msg.position.x;
}


void HzMonitor::controlFlow()
{
    ROS_WARN("Pose updated : %f   %f",pose_x,pose_y);
}

// usage: pass the name of the file as command line argument
int main(int argc, char** argv)
{
    ros::init(argc, argv, "universal_subscriber");
    ros::NodeHandle nh;
    ros::Rate rate(20);
    HzMonitor hzmonitor(nh);

    
   
    while(ros::ok())
    {
        hzmonitor.controlFlow();
        //controller.visualization();
        rate.sleep();
        ros::spinOnce();

    }
}
