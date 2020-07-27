#include <ros/console.h> 
#include "sensor_monitor.h"

/**
* @brief  Constructor for the NodeMonitor
*/
SensorMonitor::SensorMonitor():nh("~") 
{
  getAllTopics();
  validTopicList(m_sensorTopicList);
  m_sensorMonitor =std::make_shared<Monitor>(nh, "Sensor Monitor", true);
  for(int i=0;i < m_sensorTopicList.size();i++)
  {
    std::shared_ptr<SensorStatistics>sensorStatistics(new SensorStatistics(nh,m_sensorTopicList[i],m_sensorMonitor));
    sensorMonitorList.push_back(sensorStatistics);
  }

  ROS_INFO("SensorMonitor constructor called");
}

/**
* @brief  Destructor for the NodeMonitor
*/

SensorMonitor::~SensorMonitor()
{
}



/**
* @brief  Getting all the topics registered with the ROS master
*/
void SensorMonitor::getAllTopics()
{
  ros::master::V_TopicInfo master_topics;
  ros::master::getTopics(master_topics);
  
  for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++) 
  {
    m_topicListOriginal.push_back((*it).name);
  }
}


/**
* @brief  Collecting the  valid topic lists from the yaml provided by user
*/
void SensorMonitor::validTopicList(std::vector<std::string> &validTopicList)
{

  std::string path = ros::package::getPath("robot_diagnostics");
  std::vector<std::string> topicList;
  nh.getParam("/sensors", topicList);

  int invalidTopicCount = 0;
  int totalTopicCount = topicList.size();
  for(int i=0;i < totalTopicCount;i++)
  {
    bool isValid = isValidTopic(topicList[i]);
    if(isValid == false)
    {
      invalidTopicCount = invalidTopicCount + 1;
      if(invalidTopicCount == totalTopicCount) 
      {
        ROS_ERROR("Please re-check the topics provided in the yaml file %s/config/topic_monitor.yaml",path.c_str());
        exit(0);
      }
    }
    else
    {
      validTopicList.push_back(topicList[i]);
    }   
  }

}



/**
* @brief  Verifying whether the given topic is registered with the  ROS master
*/
bool SensorMonitor::isValidTopic(std::string &topic_name)
{

  for(auto topic : m_topicListOriginal)
  {
    if(topic == topic_name)
    {
      return true;
    }
  }
  return false;

}




/**
 * @brief Main function
*/
int main(int argc, char** argv)
{
  ros::init(argc, argv, "sensor_monitoring");
  SensorMonitor sensor_monitor;
  ros::spin();
  return 0;
}