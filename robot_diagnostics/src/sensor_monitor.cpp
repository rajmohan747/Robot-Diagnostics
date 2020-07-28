#include <ros/console.h> 
#include "sensor_monitor.h"

/**
* @brief  Constructor for the SensorMonitor
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
  double sensorTimerUpdateFrequency;
  nh.getParam("/sensorTimerUpdateFrequency", sensorTimerUpdateFrequency);
  topicStatusTimer = nh.createTimer(ros::Duration(1.0 / sensorTimerUpdateFrequency), &SensorMonitor::timerCallback, this);
  ROS_INFO("SensorMonitor constructor called");
}

/**
* @brief  Destructor for the SensorMonitor
*/

SensorMonitor::~SensorMonitor()
{
}


/**
* @brief  Getting all the topics registered with the ROS master
*/
void SensorMonitor::getAllTopics()
{
  m_topicListOriginal.clear();
  m_topicListOriginal.resize(0);
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
      m_invalidTopics  = true;
      invalidTopicCount = invalidTopicCount + 1;
      m_invalidTopicList.push_back(topicList[i]);
      if(invalidTopicCount == totalTopicCount) 
      {
        ROS_ERROR("Please re-check the topics provided in the yaml file %s/config/topic_monitor.yaml",path.c_str());
        //exit(0);
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
* @brief  Timer call back for updating statistics,if all the sensor topics are not available in the first time
*/

void SensorMonitor::timerCallback(const ros::TimerEvent &e)
{
  /*Gets all the topics registered with the ROS master*/
  getAllTopics();

  /*incase if there is atleast a single sensor topic not available initially,it will be checked at fixed time intervals*/
  if(m_invalidTopics)
  {
    for(int i=0; i < m_invalidTopicList.size();i++)
    {
      std::vector<std::string>::iterator it; 
      it = std::find(m_topicListOriginal.begin(),m_topicListOriginal.end(),m_invalidTopicList[i]);
      if (it != m_topicListOriginal.end()) 
      {
        std::shared_ptr<SensorStatistics>sensorStatistics(new SensorStatistics(nh,m_invalidTopicList[i],m_sensorMonitor));
        sensorMonitorList.push_back(sensorStatistics);
        m_invalidTopicList.erase(std::remove(m_invalidTopicList.begin(), m_invalidTopicList.end(), m_invalidTopicList[i]), m_invalidTopicList.end());        //= std::remove(m_topicListOriginal.begin(),m_topicListOriginal.end(),m_invalidTopicList[i]);
        if(m_invalidTopicList.size() == 0)
        {
          ROS_WARN_ONCE("All the sensor topics are available now..Hurrey");
          m_invalidTopics = false;
        }
      }
      else
      {
        ROS_INFO("Sensor topic %s is not found",m_invalidTopicList[i].c_str());
      }
    }
    
  }
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