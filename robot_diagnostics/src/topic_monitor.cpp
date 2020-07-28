#include <ros/console.h> 
#include "topic_monitor.h"
#include <ros/package.h>

/**
* @brief  Constructor for the TopicMonitor
*/
TopicMonitor::TopicMonitor():nh("~") 
{
  ROS_INFO("TopicStatistics constructor called");

  getAllTopics();
  validTopicList(m_validTopicMap);

  m_topicMonitor =std::make_shared<Monitor>(nh, "Topic Monitor", true);
  for (auto& x: m_validTopicMap) 
  {
    std::shared_ptr<TopicStatistics>topicStatistics(new TopicStatistics(nh,x.first,x.second,m_topicMonitor));
    topicMonitorList.push_back(topicStatistics);
  }

  topicUpdateTimer = nh.createTimer(ros::Duration(1.0), &TopicMonitor::topicTimerCallback, this);
  ROS_INFO("TopicMonitor constructor called");
  
}

/**
* @brief  Destructor for the TopicMonitor
*/

TopicMonitor::~TopicMonitor()
{

}


/**
* @brief  Collecting the  valid topic lists from the yaml provided by user
*/
void TopicMonitor::validTopicList(std::unordered_map<std::string ,double> &validTopicMap)
{

  std::string path = ros::package::getPath("robot_diagnostics");
  std::unordered_map<std::string ,double> topicMap;
  nh.getParam("/topics", m_topicList);
  
  /*Getting all the topics from the yaml file*/ 
  if (m_topicList.getType() == XmlRpc::XmlRpcValue::TypeArray)
  {
 
    for(int i=0; i < m_topicList.size(); i++)
    {
      XmlRpc::XmlRpcValue topicObject = m_topicList[i];
      topicMap[topicObject["name"]] = topicObject["frequency"];
    }

    /*Verifying whether the topics mentioned in the yaml file are valid or not*/
    int totalTopicCount   = m_topicList.size();
    int invalidTopicCount = 0;
    for (auto& x: topicMap) 
    {
      std::string topic = x.first;
      bool isValid = isValidTopic(topic);
      if(isValid == false)
      {
        m_invalidTopics  = true;
        m_invalidTopicMap[x.first] =  x.second;
        m_invalidTopicList.push_back(x.first);
        invalidTopicCount = invalidTopicCount + 1;
        if(invalidTopicCount == totalTopicCount) 
        {
          ROS_ERROR("Please re-check the topics provided in the yaml file %s/config/topic_monitor.yaml",path.c_str());
          //exit(0);
        }
      }
      else
      {
        validTopicMap[topic] =  x.second;
      }   
    }
  }
}

/**
* @brief  Getting all the topics registered with the ROS master
*/
void TopicMonitor::getAllTopics()
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
* @brief  Verifying whether the given topic is registered with the  ROS master
*/
bool TopicMonitor::isValidTopic(std::string &topic_name)
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


void TopicMonitor::topicTimerCallback(const ros::TimerEvent &e)
{
/*Gets all the topics registered with the ROS master*/
  if(m_invalidTopics)
  {
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
          std::shared_ptr<TopicStatistics>topicStatistics(new TopicStatistics(nh,m_invalidTopicList[i],m_invalidTopicMap[m_invalidTopicList[i]],m_topicMonitor));
          topicMonitorList.push_back(topicStatistics);   
          m_invalidTopicList.erase(std::remove(m_invalidTopicList.begin(), m_invalidTopicList.end(), m_invalidTopicList[i]), m_invalidTopicList.end());        //= std::remove(m_topicListOriginal.begin(),m_topicListOriginal.end(),m_invalidTopicList[i]);

          ROS_INFO("Found %s . Remaining topics : %d",m_invalidTopicList[i].c_str(),m_invalidTopicList.size());
          //m_invalidTopicMap.erase(x.first);   
          if(m_invalidTopicList.size() == 0)
          {
            ROS_WARN_ONCE("All the  topics are available now..Hurrey");
            m_invalidTopics = false;
          }

        }
        else
        {
          ROS_INFO(" topic %s is not found",m_invalidTopicList[i].c_str());
        }
 
      }
      
    }
  }
}


/**
 * @brief Main function
*/
int main(int argc, char** argv)
{
    ros::init(argc, argv, "topic_monitoring");
    TopicMonitor topic_monitor;
    ros::spin();
    return 0;
}
