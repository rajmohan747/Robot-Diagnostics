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

    /* Constructor for the Ros class */ 
    NodeMonitor();

     /* Destructor for the Ros class */
    ~NodeMonitor();

    void updateNodeStatistics();
private:
    ros::NodeHandle nh;

    /*Member Functions*/
    long getRamSize();
    long UpTime();
    long UpTime(std::string pid);
    long ActiveJiffies(std::string pid);
    double computeNodeCPUPercentage(std::string pid); 
    double computeNodeMemoryPercentage(std::string pid);


    std::string getNodeXmlrpcURI(std::string &node_name);
    std::string ElapsedTime(long elapsedSeconds); 
    bool isValidNode(std::string &nodeName);;

    void updateCpuStatus(std::string &node_name);
    void updateMemoryStatus(std::string &node_name);
    void updateTimeStatus(std::string &node_name);
    void updateNodeStatus(std::string &node_name);
    void updateNodePingStatus(std::string &node_name);

    void getErrorValueFromState(std::string &node_name,std::string &value, double &error_level);




    void updateValidNodeList(std::vector<std::string> &validNodeList);
    void applyNodeFilter(std::vector<std::string> &nodeListFiltered);




    int m_nodeFilterType = 0;
    std::vector<std::shared_ptr<NodeStatistics> >m_nodeList;



    bool m_setup = false;
    // double m_memPercentage,m_cpuPercentage;
    // long m_upTime,m_ramSize;

    std::vector<std::string> m_initialNodeList,m_validNodeList;
    std::vector<std::string> m_nodeListOriginal,m_nodeListFiltered;




    //std::unique_ptr<Monitor> m_monitor;
    Monitor *m_monitor; ///< An object to monitor class
};


#endif