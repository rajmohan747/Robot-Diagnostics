#include <ros/console.h> 
#include "topic_statistics.h"
#include <ros/package.h>

/**
* @brief  Constructor for the Statistics
*/
TopicStatistics::TopicStatistics()
{
   ROS_INFO("TopicStatistics constructor called");
   std::vector<std::string> initialTopicList;
   nh.getParam("/topics", initialTopicList);
   nh.getParam("/filter_type", m_topicFilterType);

  validTopicList(initialTopicList,m_validTopicList);
  applyTopicFilter();
   /*Gets all the nodes registered in the ROS master and stores it in a vector m_nodeListOriginal*/
   //ros::master::getNodes(m_nodeListOriginal);

   //ROS_ERROR("Initial list size : %d",m_initialNodeList.size());
}

/**
* @brief  Destructor for the Statistics
*/

TopicStatistics::~TopicStatistics()
{

}

void TopicStatistics::validTopicList(const std::vector<std::string> &initialTopicList,std::vector<std::string> &validTopicList)
{

  getAllTopics();
  std::string path = ros::package::getPath("robot_diagnostics");
  int invalidTopicCount = 0;
  int totalTopicCount   = initialTopicList.size();
  for(auto topic:initialTopicList)
  {
    bool isValid = isValidTopic(topic);
    if(isValid == false)
    {
      invalidTopicCount = invalidTopicCount + 1;
      if((invalidTopicCount == totalTopicCount) && (m_topicFilterType == FilterType::ADD))
      {
        ROS_ERROR("Please re-check the topics provided in the yaml file %s/config/topic_monitor.yaml",path.c_str());
        exit(0);
      }

    }
    else
    {
      validTopicList.push_back(topic);
    }
  }

}


void TopicStatistics::getAllTopics()
{

  ros::master::V_TopicInfo master_topics;
  ros::master::getTopics(master_topics);

  for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++) 
  {
    m_topicListOriginal.push_back((*it).name);
  }

}
bool TopicStatistics::isValidTopic(std::string &topic_name)
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

void TopicStatistics::applyTopicFilter()
{

  m_topicListCopy.clear();
  m_topicListCopy.resize(0);
    //ros::master::getNodes(m_nodeListOriginal);
  m_topicListCopy = m_topicListOriginal;
    //ROS_WARN("Node list size original : %d  copy : %d  inital  %d",nodeListOriginal.size(),nodeListCopy.size(),initialNodeList.size());

  if(m_topicFilterType == FilterType::DEFAULT)
  {
    ROS_WARN("Default filter type");
    m_topicListCopy = m_topicListOriginal;
  }
  else if (m_topicFilterType == FilterType::ADD)
  {
    ROS_WARN("ADD filter type");
    m_topicListCopy = m_validTopicList;
  }
  else
  {
    ROS_WARN("Remove filter type");
    for (std::vector<std::string>::iterator it(m_validTopicList.begin()); it != m_validTopicList.end(); ++it)
    {
      m_topicListCopy.erase(std::remove(begin(m_topicListCopy), end(m_topicListCopy), *it), end(m_topicListCopy));
    }

  }

}

/**
* @brief Updates the node related statistics
*/
void TopicStatistics::updateTopicStatistics()
{
  getAllTopics();
  for(std::vector<std::string>::iterator i(m_topicListCopy.begin());i != m_topicListCopy.end();++i)
  {
    std::cout <<"Topics after filtering are : "<< *i << std::endl; 
  }

}

