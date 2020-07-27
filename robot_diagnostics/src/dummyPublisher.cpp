#include <ros/ros.h>
#include <iostream>
#include "monitoring_core/monitor.h"
#include "monitoring_msgs/MonitoringArray.h"
#include "monitoring_msgs/KeyValue.h"
int main(int argc, char **argv)
{

    ros::init(argc, argv, "dummyPublisher");
    ros::NodeHandle n;
    ros::Publisher firstPub = n.advertise<monitoring_msgs::MonitoringArray>("imu", 1000);
    ros::Publisher secondPub = n.advertise<monitoring_msgs::MonitoringArray>("battery", 1000);
    ros::Rate loop_rate(10);

    ROS_INFO("Started");
    monitoring_msgs::MonitoringArray sensor1,sensor2;
    monitoring_msgs::MonitoringInfo sen1,sen2;
    

    monitoring_msgs::KeyValue keyValue1,keyValue2;
    keyValue1.key = "battery";
    keyValue1.value ="12.5";
    keyValue1.unit ="%";
    keyValue1.errorlevel=0.5;
    sen1.values.push_back(keyValue1);
    sensor1.info.push_back(sen1);

    keyValue2.key = "imu";
    keyValue2.value ="12.5";
    keyValue2.unit ="%";
    keyValue2.errorlevel=0.8;
    sen2.values.push_back(keyValue2);
    sensor2.info.push_back(sen2);
    //sensor1.info[0].values[0].key = "sensor1";
    // sensor1.info[0].values[0].value = "12.3";
    // sensor1.info[0].values[0].unit = "%";
    // sensor1.info[0].values[0].errorlevel = 0.5;

    // sensor2.info[0].values[0].key = "sensor2";
    // sensor2.info[0].values[0].value = "76.3";
    // sensor2.info[0].values[0].unit = "%";
    // sensor2.info[0].values[0].errorlevel = 0.8;
    ROS_INFO("Reached here");
    while(ros::ok())
    {
        ROS_INFO("In");
        firstPub.publish(sensor1);
        secondPub.publish(sensor2);
        ros::spinOnce();

        loop_rate.sleep();
    }
return 0;
}