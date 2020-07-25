#include <ros/console.h> 
#include "node_monitor.h"

/**
* @brief  Constructor for the NodeMonitor
*/
NodeMonitor::NodeMonitor():nh("~") 
{
  
   ROS_INFO("NodeMonitor constructor called");
   nh.getParam("/nodes", m_initialNodeList);
   nh.getParam("/nodeFilterType", m_nodeFilterType);

   /*Gets all the nodes registered in the ROS master and stores it in a vector m_nodeListOriginal*/
   ros::master::getNodes(m_nodeListOriginal);
  
   /*Verifying that the mentioned nodes in the yaml file are valid*/
   updateValidNodeList(m_validNodeList);



   applyNodeFilter(m_nodeListFiltered);
   double timerUpdateFrequency = 1.0;   
   m_nodeMonitor =std::make_shared<Monitor>(nh, "Node Monitor", true);
   for(int i=0; i< m_nodeListFiltered.size();i++)
   {
       //std::cout << "Filtered nodes are " <<m_nodeListFiltered[i] << std::endl;
       std::shared_ptr<NodeStatistics>nodeStatistics(new NodeStatistics(nh,m_nodeListFiltered[i],m_nodeMonitor));
       m_nodeList.push_back(nodeStatistics);
   }
}

/**
* @brief  Destructor for the NodeMonitor
*/

NodeMonitor::~NodeMonitor()
{
}



void NodeMonitor::updateValidNodeList(std::vector<std::string> &validNodeList)
{
    validNodeList.clear();
    validNodeList.resize(0);
    for(int i=0;i< m_initialNodeList.size();i++)
    {
        if(isValidNode(m_initialNodeList[i]))
        {
            validNodeList.push_back(m_initialNodeList[i]);
        }
    }
}


/**
* @brief checks whehter the given node name is currently registered with ros master
* @returns true if the node is available,else false;
*/
bool NodeMonitor::isValidNode(std::string &node_name)
{
  std::vector<std::string> currentNodeList;
  ros::master::getNodes(currentNodeList);
  for(auto node : currentNodeList)
  {
    if(node == node_name)
    {
      return true;
    }
  }
  return false;
}


/**
* @brief Applies the node filters on the nodes mentioned 
*/
void NodeMonitor::applyNodeFilter(std::vector<std::string> &nodeListFiltered)
{
   std::vector<std::string> nodeListCopy;
   nodeListCopy.clear();
   nodeListCopy.resize(0);
   nodeListCopy = m_nodeListOriginal;

  if(m_nodeFilterType == NodeFilter::DEFAULT)
  {
    ROS_WARN("Default filter type");
    nodeListCopy = m_nodeListOriginal;
  }
  else if (m_nodeFilterType == NodeFilter::ADD)
  {
    ROS_WARN("ADD filter type");
    nodeListCopy = m_validNodeList;
  }
  else
  {
    ROS_WARN("Remove filter type");
    for (std::vector<std::string>::iterator it(m_validNodeList.begin()); it != m_validNodeList.end(); ++it)
    {
      nodeListCopy.erase(std::remove(begin(nodeListCopy), end(nodeListCopy), *it), end(nodeListCopy));
    }
  }
  nodeListFiltered = nodeListCopy;
}









/**
 * @brief Main function
*/
int main(int argc, char** argv)
{
  ros::init(argc, argv, "node_monitoring");
  NodeMonitor node_monitor;
  ros::Rate rate(20);
  ros::spin();
  return 0;
}