#include <ros/console.h> 
#include "topic_monitor.h"
#include <ros/package.h>

/**
* @brief  Constructor for the Statistics
*/
TopicMonitor::TopicMonitor()
{
  ROS_INFO("TopicStatistics constructor called");

  getAllTopics();
  validTopicList(m_validTopicMap);

  
  for (auto& x: m_validTopicMap) 
  {
    std::cout << x.first << "  : " << x.second << std::endl;
    std::shared_ptr<TopicStatistics>monitor(new TopicStatistics(nh,x.first,x.second));
    monitor_list_.push_back(monitor);
  }

}

/**
* @brief  Destructor for the Statistics
*/

TopicMonitor::~TopicMonitor()
{

}

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
        invalidTopicCount = invalidTopicCount + 1;
        if(invalidTopicCount == totalTopicCount) 
        {
          ROS_ERROR("Please re-check the topics provided in the yaml file %s/config/topic_monitor.yaml",path.c_str());
          exit(0);
        }
      }
      else
      {
        validTopicMap[topic] =  x.second;
      }   
    }
  }
}


void TopicMonitor::getAllTopics()
{
  ros::master::V_TopicInfo master_topics;
  ros::master::getTopics(master_topics);
  
  for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++) 
  {
    m_topicListOriginal.push_back((*it).name);
  }
}

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
 * @brief Main function
*/
int main(int argc, char** argv)
{
    ros::init(argc, argv, "topic_monitoring");
    TopicMonitor topic_monitor;
    //std::unique_ptr<TopicStatistics> statisticsPtr(new TopicStatistics);
    //TopicMonitor topic_monitor(std::move(statisticsPtr));
    ros::Rate rate(20);
    ros::spin();

    // while(ros::ok())
    // {
    //     //topic_monitor.topicMonitoring();
    //     rate.sleep();
    //     ros::spinOnce();
    // }
    return 0;
}