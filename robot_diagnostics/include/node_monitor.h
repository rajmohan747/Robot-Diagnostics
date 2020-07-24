#ifndef NODE_MONITOR_H
#define NODE_MONITOR_H

#include <iostream>
#include <ros/ros.h>
#include <vector>

#include <unistd.h>
#include "XmlRpc.h"

#include <memory>


#include "utils.h"
#include "node_statistics.h"



class NodeMonitor
{
public:

    /* Constructor for the NodeMonitor class */ 
    NodeMonitor();

     /* Destructor for the NodeMonitor class */
    ~NodeMonitor();


private:

    ros::NodeHandle nh;
    /*Member functions*/

    bool isValidNode(std::string &nodeName);;
    void updateValidNodeList(std::vector<std::string> &validNodeList);
    void applyNodeFilter(std::vector<std::string> &nodeListFiltered);


    /*Member variables*/

    int m_nodeFilterType = 0;
    std::vector<std::shared_ptr<NodeStatistics> >m_nodeList;
    std::vector<std::string> m_initialNodeList,m_validNodeList;
    std::vector<std::string> m_nodeListOriginal,m_nodeListFiltered;

    //Monitor *monitor_;
    std::shared_ptr<Monitor> m_nodeMonitor;

};


#endif