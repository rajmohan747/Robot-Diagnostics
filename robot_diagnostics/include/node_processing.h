#ifndef NODE_PROCESSING_H
#define NODE_PROCESSING_H
#include <ros/ros.h>
#include <iostream>
#include <vector>
#include "utilities.h"
#include <unistd.h>
/**
* @class NodeProcessing
* @brief Collects the statistics data related to a node
*/




class NodeProcessing
{
    public:
        /**
        * @brief  Constructor for the NodeProcessing
        */
        NodeProcessing();

        /**
        * @brief  Destructor for the NodeProcessing
        */
        ~NodeProcessing();

        void applyNodeFilter(int nodeFilterType,std::vector<std::string> &nodeListFiltered,const std::vector<std::string> &nodeListOriginal, std::vector<std::string> &initialNodeList);
        void updateValidNodeList(std::vector<std::string> &validNodeList,std::vector<std::string> &invalidNodeList, std::vector<std::string> nodeListFiltered);
        void defaultFilterProcessing(const std::vector<std::string> &nodeListOriginal,std::vector<std::string> &alreadyCreatedNodeList,std::vector<std::string> &newNodeList,double nodeTimeOut,bool &nodeWaiting);
        void addFilterProcessing( std::vector<std::string> nodeListOriginal, std::vector<std::string> &invalidNodeList,std::vector<std::string> &newNodeList,double nodeTimeOut,bool &nodeWaiting);
        void removeFilterProcessing(const std::vector<std::string> &nodeListOriginal,std::vector<std::string> initialNodeList,std::vector<std::string> &alreadyCreatedNodeList,std::vector<std::string> &newNodeList,double nodeTimeOut,bool &nodeWaiting);

    private:
        /*Member functions*/
        uint64_t m_lastTime;
       
};
#endif
