#include <ros/ros.h>
#include <iostream>
#include "monitoring_core/monitor.h"
#include "monitoring_msgs/MonitoringArray.h"
#include "monitoring_msgs/KeyValue.h"
#include "Math_Func.h"
int main(int argc, char **argv)
{

    ros::init(argc, argv, "dummyBattery");
    ros::NodeHandle n;

    ros::Publisher secondPub = n.advertise<monitoring_msgs::MonitoringArray>("battery", 1000);
ros::Publisher second = n.advertise<monitoring_msgs::MonitoringArray>("dummybattery", 1000);
    ros::Rate loop_rate(1);

    ROS_INFO("Started");
    monitoring_msgs::MonitoringArray sensor2;
    monitoring_msgs::MonitoringInfo sen2;
    

    monitoring_msgs::KeyValue keyValue2;


    keyValue2.key = "battery";
    keyValue2.value ="12.5";
    keyValue2.unit ="%";
    keyValue2.errorlevel=0.8;
    sen2.values.push_back(keyValue2);
    sensor2.info.push_back(sen2);

    //MathFunc::dummyfun<void>();

	//MathFunc::dummyfun();

    while(ros::ok())
    {

        secondPub.publish(sensor2);
        ros::spinOnce();

        loop_rate.sleep();
    }
return 0;
}
