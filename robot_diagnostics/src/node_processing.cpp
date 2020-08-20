#include "node_processing.h"

/**
* @brief  Constructor for the NodeProcessing
*/

NodeProcessing::NodeProcessing()
{
  ROS_INFO("Constructor Node Processing");
  
  m_lastTime = Utilities::millis<uint64_t>();
    
}

/**
* @brief  Destructor for the NodeProcessing
*/

NodeProcessing::~NodeProcessing()
{

}

/**
* @brief Applies the node filters on the nodes mentioned 
*/
void NodeProcessing::applyNodeFilter(int nodeFilterType,std::vector<std::string> &nodeListFiltered,const std::vector<std::string> &nodeListOriginal,std::vector<std::string> &initialNodeList)
{
   std::vector<std::string> nodeListCopy;
   nodeListCopy.clear();
   nodeListCopy.resize(0);
   nodeListCopy = nodeListOriginal;

  if(nodeFilterType == Utilities::NodeFilter::DEFAULT)
  {
    ROS_WARN("Default filter type");
    nodeListCopy = nodeListOriginal;
  }
  else if (nodeFilterType == Utilities::NodeFilter::ADD)
  {
    ROS_WARN("ADD filter type");
    nodeListCopy = initialNodeList;
  }
  else
  {
    ROS_WARN("Remove filter type");
    for (std::vector<std::string>::iterator it(initialNodeList.begin()); it != initialNodeList.end(); ++it)
    {
      nodeListCopy.erase(std::remove(begin(nodeListCopy), end(nodeListCopy), *it), end(nodeListCopy));
    }
  }
  nodeListFiltered = nodeListCopy;
}

/**
* @brief Updates the valid nodelist 
*/

void NodeProcessing::updateValidNodeList(std::vector<std::string> &validNodeList,std::vector<std::string> &invalidNodeList,std::vector<std::string> nodeListFiltered)
{
    validNodeList.clear();
    validNodeList.resize(0);
    for(int i=0;i< nodeListFiltered.size();i++)
    {
        if(Utilities::isValidNode<bool,std::string>(nodeListFiltered[i]))
        {
          validNodeList.push_back(nodeListFiltered[i]);
        }
        else
        {
          //m_invalidNodes = true;
          invalidNodeList.push_back(nodeListFiltered[i]);
        }
    }
}

/**
* @brief Applies the DEFAULT filter on the nodes available
*/
void NodeProcessing::defaultFilterProcessing(const std::vector<std::string> &nodeListOriginal,std::vector<std::string> &alreadyCreatedNodeList,std::vector<std::string> &newNodeList,double nodeTimeOut,bool &nodeWaiting)
{
  
  uint64_t timeoutTime = Utilities::millis<uint64_t>();
  uint64_t timeoutDelta =timeoutTime - m_lastTime;
  
  for(int i =0; i< nodeListOriginal.size();i++)
  {

    std::vector<std::string>::iterator itr;
    itr = std::find(alreadyCreatedNodeList.begin(),alreadyCreatedNodeList.end(),nodeListOriginal[i]);          
    if (itr == alreadyCreatedNodeList.end())    
    {
      ROS_WARN("Default mode -  Found node : %s",nodeListOriginal[i].c_str());
      newNodeList.push_back(nodeListOriginal[i]);
      alreadyCreatedNodeList.push_back(nodeListOriginal[i]);
    }

  }  
  
  if (timeoutDelta > (nodeTimeOut*1000))
  {
    ROS_INFO("Time out,Default mode");
    nodeWaiting = false;
  }
}

/**
* @brief Applies the ADD filter on the nodes available
*/
void NodeProcessing::addFilterProcessing( std::vector<std::string> nodeListOriginal, std::vector<std::string> &invalidNodeList,std::vector<std::string> &newNodeList,double nodeTimeOut,bool &nodeWaiting)
{
  uint64_t timeoutTime = Utilities::millis<uint64_t>();
  uint64_t timeoutDelta =timeoutTime - m_lastTime;
  /*Keeps on checking for the required nodes in the invalidNodeList for a timeout duration
  If not found within timeout, the constructor willl be called,which eventually will throws the error*/
  for(int i=0; i < invalidNodeList.size();i++)
  {
    std::vector<std::string>::iterator it; 
    it = std::find(nodeListOriginal.begin(),nodeListOriginal.end(),invalidNodeList[i]);
    
    if ( ((it != nodeListOriginal.end()) || (timeoutDelta > (nodeTimeOut*1000)) )  && (invalidNodeList.size() > 0))
    {
      ROS_INFO("Found %s . Remaining nodes : %d",invalidNodeList[i].c_str(),invalidNodeList.size());
      newNodeList.push_back(invalidNodeList[i]);
      invalidNodeList.erase(std::remove(invalidNodeList.begin(), invalidNodeList.end(), invalidNodeList[i]), invalidNodeList.end());        
      
              
      if(invalidNodeList.size() == 0)
      {
        ROS_WARN_ONCE("All the  nodes are available now..Hurrey");
        nodeWaiting = false;
      }
    }
    // else
    // {
    //   ROS_INFO(" node %s is not found",invalidNodeList[i].c_str());
    // }
  }

}

/**
* @brief Applies the REMOVE filter on the nodes available
*/
void NodeProcessing::removeFilterProcessing(const std::vector<std::string> &nodeListOriginal,std::vector<std::string> initialNodeList,std::vector<std::string> &alreadyCreatedNodeList,std::vector<std::string> &newNodeList,double nodeTimeOut,bool &nodeWaiting)
{
  uint64_t timeoutTime = Utilities::millis<uint64_t>();
  uint64_t timeoutDelta =timeoutTime - m_lastTime;

  for(int i =0; i< nodeListOriginal.size();i++)
  {
    std::vector<std::string>::iterator it;
    it = std::find(initialNodeList.begin(),initialNodeList.end(),nodeListOriginal[i]);
    std::vector<std::string>::iterator itr;
    itr = std::find(alreadyCreatedNodeList.begin(),alreadyCreatedNodeList.end(),nodeListOriginal[i]);          
    
    if ((itr == alreadyCreatedNodeList.end())  && (it == initialNodeList.end()))  
    {
      ROS_WARN("Remove mode -  Found node : %s",nodeListOriginal[i].c_str());
      newNodeList.push_back(nodeListOriginal[i]);
      alreadyCreatedNodeList.push_back(nodeListOriginal[i]);
    }
  }

  if (timeoutDelta > (nodeTimeOut*1000))
  {
    ROS_INFO("Time out,Remove mode");
    nodeWaiting = false;
  }
}