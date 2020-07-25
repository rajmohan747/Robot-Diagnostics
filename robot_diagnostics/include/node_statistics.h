#ifndef NODE_STATISTICS_H
#define NODE_STATISTICS_H
#include <ros/ros.h>
#include <iostream>
#include "utils.h"
#include <fstream>
#include <mutex>
#include <memory>
#include <vector>
#include <unistd.h>
#include "unordered_map"
#include "monitoring_core/monitor.h"

/**
* @class NodeStatistics
* @brief Collects the statistics data related to a node
*/



class NodeStatistics
{
    public:
        /**
        * @brief  Constructor for the NodeStatistics
        */
        NodeStatistics(ros::NodeHandle &nh,std::string topicName,std::shared_ptr<Monitor> m_monitor);

        /**
        * @brief  Destructor for the NodeStatistics
        */
        ~NodeStatistics();
    
    private:
        /*Member functions*/
        void timerCallback(const ros::TimerEvent &e);
        void monitorNodeStatistics();
        void updateNodePingStatus();
        
        void publishNodeStatistics();
        void publishNodeCpuUsage();
        void publishNodeMemoryUsage();
        void publishNodeUnavailableInfo(double error_level);
        void publishNodeStatus();
        void getErrorValueFromState(std::string &value, double &error_level);

        std::string getPid();
        std::string getNodeXmlrpcURI();
        double computeNodeCPUPercentage(std::string pid);
        double computeNodeMemoryPercentage(std::string pid);
        long ActiveJiffies(std::string pid);
        long UpTime(std::string pid);
        long UpTime(); 
        long getRamSize();

        bool isNodeAvailable(); 
        /*Member variables*/
        ros::Timer nodeStatusTimer; 
        std::string m_nodeName,m_lastPid;

        int m_maxPermissibleNodeRestart;
        int m_nodeRestartCount = 0;
        double m_memPercentage =0.0;
        double m_cpuPercentage = 0.0;
        double m_maxPermissibleCPUUsage,m_maxPermissibleMemoryUsage;
        long m_ramSize,m_upTime;
        char m_nodeState;

        bool m_nodeSetup = false;
        bool m_isAvailable= false;
        bool m_nodeRestart = false;
        std::unordered_map<std::string, std::string> m_nodeLog; 
        std::mutex m_mutex;
        std::shared_ptr<Monitor> m_monitor;
};
#endif
