#ifndef SENSOR_STATISTICS_H
#define SENSOR_STATISTICS_H
#include <ros/ros.h>
#include <iostream>
#include "xmlrpcpp/XmlRpcValue.h"
#include <unordered_map>
#include <ros/package.h>
#include <memory>
#include "monitoring_core/monitor.h"
#include "monitoring_msgs/MonitoringArray.h"
/**
* @class SensorStatistics
*/



class SensorStatistics
{
public:
/**
 * @brief  Constructor for the SensorStatistics
 */



    SensorStatistics(ros::NodeHandle &nh,std::string topicName,std::shared_ptr<Monitor> monitor);

    /**
    * @brief  Destructor for the SensorStatistics
    */
    ~SensorStatistics();
    


private:

    ros::NodeHandle nh;

    /**
     * @brief ROS Subscribers
    */

    ros::Subscriber sensorSub;


    /*Member functions*/

    void sensorMessageCallback(const monitoring_msgs::MonitoringArrayConstPtr &msg);


    /*Member variables*/

    std::shared_ptr<Monitor> m_monitor;
};
#endif
