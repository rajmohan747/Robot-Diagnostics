#include <ros/console.h> 
#include "sensor_statistics.h"


SensorStatistics::SensorStatistics(ros::NodeHandle &nh,std::string topicName,std::shared_ptr<Monitor> monitor)
{
    m_monitor= monitor;
    ROS_WARN("SensorStatistics constructor called  for topic : %s",topicName.c_str());

    /*Subscribers*/

    sensorSub = nh.subscribe(topicName, 1, &SensorStatistics::sensorMessageCallback, this);
    /*Timer*/
    
    /*Initialization of time variables*/


}

/**
* @brief  Destructor for the TopicStatistics
*/

SensorStatistics::~SensorStatistics()
{

}


void SensorStatistics::sensorMessageCallback(const monitoring_msgs::MonitoringArrayConstPtr &msg)
{
    auto errors = msg->info[0];
    for(auto error:errors.values)
    {
        m_monitor->addValue(error.key, error.value,error.unit,error.errorlevel, AggregationStrategies::FIRST);
        //ROS_INFO("Size of message : %d",errors.values.size());
    }
}
