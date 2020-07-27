#include <ros/console.h> 
#include "sensor_monitor.h"

/**
* @brief  Constructor for the NodeMonitor
*/
SensorMonitor::SensorMonitor():nh("~") 
{
  
  ROS_INFO("SensorMonitor constructor called");
  getAllTopics();
  validTopicList(m_validTopicMap);

  for(auto &x:m_validTopicMap)
  {
    std::cout << "Node "<<x.second << "Topic  "<< x.first<< std::endl;
  }
  m_topicMonitor =std::make_shared<Monitor>(nh, "Sensor Monitor", true);
  for (auto& x: m_validTopicMap) 
  {
    //std::cout << x.first << "  : " << x.second << std::endl;
    std::shared_ptr<SensorStatistics>sensorStatistics(new SensorStatistics(nh,x.first,x.second,m_topicMonitor));
    monitor_list_.push_back(sensorStatistics);
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
void SensorMonitor::validTopicList(std::unordered_map<std::string ,std::string> &validTopicMap)
{

  std::string path = ros::package::getPath("robot_diagnostics");
  std::unordered_map<std::string ,std::string> topicMap;
  nh.getParam("/sensors", m_topicList);
  //std::cout << "Here " << m_topicList << std::endl;
  /*Getting all the topics from the yaml file*/ 
  if (m_topicList.getType() == XmlRpc::XmlRpcValue::TypeArray)
  {
    //std::cout << "I am here   "<< m_topicList.size() << std::endl;
    for(int i=0; i < m_topicList.size(); i++)
    {
      //std::cout << "Got in" << std::endl;
      XmlRpc::XmlRpcValue topicObject = m_topicList[i];
      //topicMap[topicObject["name"]] = topicObject["frequency"];
      //std::cout <<"Mei yahaan" <<topicObject["node"] << " :  " << topicObject["topic"];
      topicMap[std::string(topicObject["node"])] = std::string(topicObject["topic"]);
    }

    /*Verifying whether the topics mentioned in the yaml file are valid or not*/
    int totalTopicCount   = m_topicList.size();
    int invalidTopicCount = 0;
    for (auto& x: topicMap) 
    {
      std::string topic = x.second;
      bool isValid = isValidTopic(topic);
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
        validTopicMap[topic] =  x.first;
      }   
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
  ros::Rate rate(20);
  ros::spin();
  return 0;
}