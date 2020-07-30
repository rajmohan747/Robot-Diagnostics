#include <ros/console.h> 
#include "sensor_monitor.h"

/**
* @brief  Constructor for the SensorMonitor
*/
SensorMonitor::SensorMonitor():nh("~") 
{
  double sensorTimerUpdateFrequency;
  nh.getParam("/sensorTimerUpdateFrequency", sensorTimerUpdateFrequency);


  Utilities::getAllTopics<void,std::string>(m_topicListOriginal);
  validTopicList(m_sensorTopicList);
  m_sensorMonitor =std::make_shared<Monitor>(nh, "Sensor Monitor", true);
  
  for(int i=0;i < m_sensorTopicList.size();i++)
  {
    std::shared_ptr<SensorStatistics>sensorStatistics(new SensorStatistics(nh,m_sensorTopicList[i],m_sensorMonitor));
    sensorMonitorList.push_back(sensorStatistics);
  }
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
    bool isValid = Utilities::isValidTopic<bool,std::string>(topicList[i],m_topicListOriginal);
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
* @brief  Timer call back for updating statistics,if all the sensor topics are not available in the first time
*/

void SensorMonitor::timerCallback(const ros::TimerEvent &e)
{
  /*Gets all the topics registered with the ROS master*/
  Utilities::getAllTopics<void,std::string>(m_topicListOriginal);

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