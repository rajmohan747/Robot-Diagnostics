#include <ros/console.h> 
#include "node_monitor.h"

/**
* @brief  Constructor for the NodeMonitor
*/
NodeMonitor::NodeMonitor():nh("~") 
{
  
   /*Info from the parmaeter server*/
   nh.getParam("/nodes", m_initialNodeList);
   nh.getParam("/nodeFilterType", m_nodeFilterType); 
   nh.getParam("/nodeTimeOut", m_nodeTimeOut); 


   /*Gets all the nodes registered in the ROS master and stores it in a vector m_nodeListOriginal*/
   ros::master::getNodes(m_nodeListOriginal);
   m_lastTime = Utilities::millis<uint64_t>();

   /*Applying the node filters to the entire node list registered with the ROS master*/
   applyNodeFilter(m_nodeListFiltered);
   
   /*Verifying that the mentioned nodes in the yaml file are valid*/ 
   updateValidNodeList(m_validNodeList);

   m_nodeMonitor =std::make_shared<Monitor>(nh, "Node Monitor", true);
   
   
   //m_alreadyCreatedNodeList.clear();
   for(int i=0; i< m_validNodeList.size();i++)
   {
    //std::cout << "Filtered nodes are " <<m_validNodeList[i] << std::endl;
    std::shared_ptr<NodeStatistics>nodeStatistics(new NodeStatistics(nh,m_validNodeList[i],m_nodeMonitor));
    m_nodeList.push_back(nodeStatistics);
    //m_alreadyCreatedNodeList.push_back(m_validNodeList[i]);
   }

   
   
   
   for(int i=0; i< m_invalidNodeList.size();i++)
   {
      std::cout << "Invalid nodes are " <<m_invalidNodeList[i] << std::endl;
   }

  nodeUpdateTimer = nh.createTimer(ros::Duration(1.0), &NodeMonitor::nodeTimerCallback, this);
   ROS_INFO("NodeMonitor constructor called");
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
    for(int i=0;i< m_nodeListFiltered.size();i++)
    {
        if(Utilities::isValidNode<bool,std::string>(m_nodeListFiltered[i]))
        {
          validNodeList.push_back(m_nodeListFiltered[i]);
        }
        else
        {
          m_invalidNodes = true;
          m_invalidNodeList.push_back(m_nodeListFiltered[i]);
        }
    }

    // if(m_nodeFilterType == Utilities::NodeFilter::REMOVE)
    // {
    //   m_invalidNodes = true;
    // }
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

  if(m_nodeFilterType == Utilities::NodeFilter::DEFAULT)
  {
    ROS_WARN("Default filter type");
    nodeListCopy = m_nodeListOriginal;
  }
  else if (m_nodeFilterType == Utilities::NodeFilter::ADD)
  {
    ROS_WARN("ADD filter type");
    nodeListCopy = m_initialNodeList;
  }
  else
  {
    ROS_WARN("Remove filter type");
    for (std::vector<std::string>::iterator it(m_initialNodeList.begin()); it != m_initialNodeList.end(); ++it)
    {
      nodeListCopy.erase(std::remove(begin(nodeListCopy), end(nodeListCopy), *it), end(nodeListCopy));
    }
  }
  nodeListFiltered = nodeListCopy;
}


/*Note: Adding up nodes in middle of process works for FilterType:ADD only as of now*/
/*TO DO: integrate the same functionality for other two filter types*/
void NodeMonitor::nodeTimerCallback(const ros::TimerEvent &e)
{

  if(m_invalidNodes) //|| (m_nodeFilterType == Utilities::NodeFilter::REMOVE))
  {
       /*Gets all the nodes registered with the ROS master*/
      m_nodeListOriginal.clear();
      m_nodeListOriginal.resize(0);
      ros::master::getNodes(m_nodeListOriginal);
      uint64_t timeoutTime = Utilities::millis<uint64_t>();
      uint64_t timeoutDelta =timeoutTime - m_lastTime;

      // for(int i =0; i< m_nodeListOriginal.size();i++)
      // {
      //   std::vector<std::string>::iterator it;
      //   it = std::find(m_initialNodeList.begin(),m_initialNodeList.end(),m_nodeListOriginal[i]);
      //   std::vector<std::string>::iterator itr;
      //   itr = std::find(m_alreadyCreatedNodeList.begin(),m_alreadyCreatedNodeList.end(),m_nodeListOriginal[i]);          
      //   if ((it == m_initialNodeList.end()) && (itr == m_alreadyCreatedNodeList.end()) && (m_nodeFilterType == Utilities::NodeFilter::REMOVE))      
      //   {
      //     //ROS_WARN("Filter in mode 2 ,found node : %s",m_nodeListOriginal[i].c_str());
      //     m_invalidNodeList.push_back(m_nodeListOriginal[i]);
      //   }
      // }



      /*Keeps on checking for the required nodes in the invalidNodeList for a timeout duration
      If not found within timeout, the constructor willl be called,which eventually will throws the error*/
      for(int i=0; i < m_invalidNodeList.size();i++)
      {
          std::vector<std::string>::iterator it; 
          it = std::find(m_nodeListOriginal.begin(),m_nodeListOriginal.end(),m_invalidNodeList[i]);
          if ((it != m_nodeListOriginal.end()) || (timeoutDelta > (m_nodeTimeOut*1000)) )
          {
            std::shared_ptr<NodeStatistics>nodeStatistics(new NodeStatistics(nh,m_invalidNodeList[i],m_nodeMonitor));
            m_nodeList.push_back(nodeStatistics);
            //m_alreadyCreatedNodeList.push_back(m_invalidNodeList[i]);
            m_invalidNodeList.erase(std::remove(m_invalidNodeList.begin(), m_invalidNodeList.end(), m_invalidNodeList[i]), m_invalidNodeList.end());        //= std::remove(m_topicListOriginal.begin(),m_topicListOriginal.end(),m_invalidTopicList[i]);

            ROS_INFO("Found %s . Remaining nodes : %d",m_invalidNodeList[i].c_str(),m_invalidNodeList.size());
            
            if(m_invalidNodeList.size() == 0)
            {
              ROS_WARN_ONCE("All the  nodes are available now..Hurrey");
              m_invalidNodes = false;
            }

          }
          else
          {
            ROS_INFO(" nodes %s is not found",m_invalidNodeList[i].c_str());
          }
  
      }
   
    } 
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