#ifndef NODE_STATISTICS_H
#define NODE_STATISTICS_H
#include <ros/ros.h>
#include <iostream>
#include "utils.h"
#include <fstream>
#include <mutex>
#include <memory>
#include "unordered_map"
#include "monitoring_core/monitor.h"
  /**
   * @class TrajectoryController
   * @brief A controller that follows the trajectory provided by a planner.
   */



class NodeStatistics
{
    public:
        /**
        * @brief  Constructor for the TrajectoryController
        */
        NodeStatistics(ros::NodeHandle &nh,std::string topicName);

        /**
        * @brief  Destructor for the TrajectoryController
        */
        ~NodeStatistics();
    
    private:
        /*Member functions*/
        void timerCallback(const ros::TimerEvent &e);
        void monitorNodeStatistics();
        
        void updateNodePingStatus(std::string &node_name);
        
        void publishNodeStatistics();
        void publishNodeCpuUsage(std::string &node_name);
        void publishNodeMemoryUsage(std::string &node_name);
        void publishNodeUnavailableInfo();

        std::string getPid(std::string nodeName);
        std::string getNodeXmlrpcURI(std::string &node_name);
        double computeNodeCPUPercentage(std::string pid);
        double computeNodeMemoryPercentage(std::string pid);
        long ActiveJiffies(std::string pid);
        long UpTime(std::string pid);
        long UpTime(); 
        long getRamSize();

        bool isNodeAvailable(std::string &node_name); 
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
        std::unordered_map<std::string, std::string> m_nodeLog; 
        std::mutex m_mutex;
        std::shared_ptr<Monitor> m_monitor;
        //Monitor *m_monitor; ///< An object to monitor class
};
#endif
