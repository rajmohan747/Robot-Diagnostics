#include <ros/console.h> 
#include "node_monitor.h"


/**
* @brief  Constructor for the NodeMonitor
*/
NodeMonitor::NodeMonitor():nh("~") 
{
  
  ROS_INFO("NodeMonitor constructor called");

   /*Info from the parmaeter server*/
   readParams(nh);
  
   /*Aggregates all the parameters to a common structure which would be used in NodeStatistics class*/
   nodeParamMapping();

   /*Gets all the nodes registered in the ROS master and stores it in a vector m_nodeListOriginal*/
   ros::master::getNodes(m_nodeListOriginal);
   nodeProcessing = std::make_shared<NodeProcessing>();
   /*Applying the node filters to the entire node list registered with the ROS master*/
   nodeProcessing->applyNodeFilter(m_nodeFilterType,m_nodeListFiltered,m_nodeListOriginal,m_initialNodeList);
   
   /*Verifying that the mentioned nodes in the yaml file are valid*/ 
   nodeProcessing->updateValidNodeList(m_validNodeList,m_invalidNodeList,m_nodeListFiltered);

   m_nodeMonitor = std::make_shared<Monitor>(nh, "Node Monitor", true);
   
   


   /*Create instances of NodeStatistics for monitoring the valid node infos*/
   m_alreadyCreatedNodeList.clear();
   
   for(int i=0; i< m_validNodeList.size();i++)
   {
      /*Confirming right before creating instance of node statistics that,
      node to be verified is not stray*/
      std::string strayString = "rostopic";
      if (m_validNodeList[i].find(strayString) != std::string::npos) 
      {
        ROS_ERROR("Stray node found : %s  and ignored",m_validNodeList[i].c_str());
        continue;
      }
      else
      {
        std::unique_lock<std::mutex> defaultlock (m_instanceMutex);
        std::shared_ptr<NodeStatistics>nodeStatistics(new NodeStatistics(nh,m_validNodeList[i],m_nodeMonitor,m_nodeParams));
        m_nodeList.push_back(nodeStatistics);
        m_alreadyCreatedNodeList.push_back(m_validNodeList[i]);
      }

   }


  m_nodeWaiting = true;
  nodeUpdateTimer = nh.createTimer(ros::Duration(1.0), &NodeMonitor::nodeTimerCallback, this);
  
}

/**
* @brief  Destructor for the NodeMonitor
*/

NodeMonitor::~NodeMonitor()
{
}

void NodeMonitor::readParams(ros::NodeHandle &nh)
{   
   nh.getParam("/nodes", m_initialNodeList);
   nh.getParam("/nodeFilterType", m_nodeFilterType); 
   nh.getParam("/nodeTimeOut", m_nodeTimeOut); 
   nh.getParam("/nodeErrors", m_nodeErrors);

   nh.getParam("/maxPermissibleNodeRestart", m_maxPermissibleNodeRestart);
   nh.getParam("/maxPermissibleCpuUsage", m_maxPermissibleCpuUsage);
   nh.getParam("/maxPermissibleMemoryUsage", m_maxPermissibleMemoryUsage);
   nh.getParam("/timerUpdateFrequency", m_timerUpdateFrequency);
}

void NodeMonitor::nodeParamMapping()
{
  m_nodeParams.nodeFilterType            = m_nodeFilterType;
  m_nodeParams.nodeTimeOut               = m_nodeTimeOut;
  m_nodeParams.maxPermissibleCpuUsage    = m_maxPermissibleCpuUsage;
  m_nodeParams.maxPermissibleMemoryUsage = m_maxPermissibleMemoryUsage;
  m_nodeParams.maxPermissibleNodeRestart = m_maxPermissibleNodeRestart;
  m_nodeParams.timerUpdateFrequency      = m_timerUpdateFrequency;



    /*Getting all the topics from the yaml file*/ 
  if (m_nodeErrors.getType() == XmlRpc::XmlRpcValue::TypeArray)
  {
 
    for(int i=0; i < m_nodeErrors.size(); i++)
    {
      XmlRpc::XmlRpcValue errorObject = m_nodeErrors[i];
      m_nodeParams.nodeErrorMap[errorObject["key"]] = errorObject["error_level"];
      //std::cout << errorObject["key"] << " : " << errorObject["error_level"] << std::endl;
    }
  }
}






/*Note: Adding up nodes in middle of process works for FilterType:ADD only as of now*/
/*TO DO: integrate the same functionality for other two filter types*/
void NodeMonitor::nodeTimerCallback(const ros::TimerEvent &e)
{

  if(m_nodeWaiting) 
  {
   
    m_newNodeList.clear();
    m_nodeListOriginal.clear();
    m_nodeListOriginal.resize(0);
    /*Gets all the nodes registered with the ROS master*/
    ros::master::getNodes(m_nodeListOriginal);


    
    /*Till timeout happens,accepts nodes subjected to the filter types provided*/
    
    if(m_nodeFilterType == Utilities::NodeFilter::DEFAULT)
    {
      nodeProcessing->defaultFilterProcessing(m_nodeListOriginal,m_alreadyCreatedNodeList,m_newNodeList,m_nodeTimeOut,m_nodeWaiting);
    }
    else if(m_nodeFilterType == Utilities::NodeFilter::ADD)
    { 
      nodeProcessing->addFilterProcessing(m_nodeListOriginal, m_invalidNodeList,m_newNodeList,m_nodeTimeOut,m_nodeWaiting);
    }
    else
    {
      nodeProcessing->removeFilterProcessing(m_nodeListOriginal,m_initialNodeList,m_alreadyCreatedNodeList,m_newNodeList,m_nodeTimeOut,m_nodeWaiting);
    }

    for(int i =0; i < m_newNodeList.size(); i++)
    {
      /*Confirming right before creating instance of node statistics that,
      node to be verified is not stray*/
      
      std::string strayString = "rostopic";
      if (m_newNodeList[i].find(strayString) != std::string::npos) 
      {
        ROS_ERROR("Stray node found : %s  and ignored",m_newNodeList[i].c_str());
        continue;
      }
      else
      {
        std::unique_lock<std::mutex> defaultlock (m_instanceMutex);
        std::shared_ptr<NodeStatistics>nodeStatistics(new NodeStatistics(nh,m_newNodeList[i],m_nodeMonitor,m_nodeParams));
        m_nodeList.push_back(nodeStatistics); 
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
