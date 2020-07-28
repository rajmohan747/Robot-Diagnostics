#include <ros/ros.h>
#include <iostream>
#include "monitoring_core/monitor.h"
#include "monitoring_msgs/MonitoringArray.h"
#include "monitoring_msgs/KeyValue.h"
int main(int argc, char **argv)
{

    ros::init(argc, argv, "dummyImu");
    ros::NodeHandle n;
    ros::Publisher firstPub = n.advertise<monitoring_msgs::MonitoringArray>("imu", 1000);

    ros::Rate loop_rate(1);

    ROS_INFO("Started");
    monitoring_msgs::MonitoringArray sensor1;
    monitoring_msgs::MonitoringInfo sen1;
    

    monitoring_msgs::KeyValue keyValue1;
    keyValue1.key = "imu";
    keyValue1.value ="22.5";
    keyValue1.unit ="%";
    keyValue1.errorlevel=0.5;
    sen1.values.push_back(keyValue1);
    sensor1.info.push_back(sen1);



    ROS_INFO("Reached here");
    while(ros::ok())
    {
       // ROS_INFO("In");
        firstPub.publish(sensor1);

        ros::spinOnce();

        loop_rate.sleep();
    }
return 0;
}
