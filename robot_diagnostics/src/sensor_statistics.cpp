#include <ros/console.h> 
#include "sensor_statistics.h"


SensorStatistics::SensorStatistics(ros::NodeHandle &nh,std::string topicName,std::string nodeName,std::shared_ptr<Monitor> monitor)
{
    m_topic  = topicName;
    m_monitor= monitor;
    ROS_INFO("SensorStatistics constructor called  for topic : %s",m_topic.c_str());

    /*Subscribers*/

    sensorSub = nh.subscribe(m_topic, 1, &SensorStatistics::sensorMessageCallback, this);
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
        ROS_WARN("Topic : %s  has error level :%f  size  %d",m_topic.c_str(),error.errorlevel,errors.values.size());
        m_monitor->addValue(error.key, error.value,error.unit,error.errorlevel, AggregationStrategies::FIRST);
    }
}
