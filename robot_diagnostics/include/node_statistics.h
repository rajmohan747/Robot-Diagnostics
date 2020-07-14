#ifndef STATISTICS_H
#define STATISTICS_H

#include <iostream>
#include <ros/ros.h>
#include <fstream>
#include <vector>
#include <fstream>
#include <unistd.h>
#include "XmlRpc.h"
#include "unordered_map"

#include "monitoring_core/monitor.h"

const std::string kProcDirectory{"/proc/"};
const std::string kStatusFilename{"/status"};
const std::string kUptimeFilename{"/uptime"};
const std::string kStatFilename{"/stat"};
const std::string kMeminfoFilename{"/meminfo"};


enum CPUStates {
  kUser_ = 0,
  kNice_,
  kSystem_,
  kIdle_,
  kIOwait_,
  kIRQ_,
  kSoftIRQ_,
  kSteal_,
  kGuest_,
  kGuestNice_
};
class Statistics
{
public:

    /* Constructor for the Ros class */ 
    Statistics();

     /* Destructor for the Ros class */
    ~Statistics();

    void updateStatistics();
private:
    ros::NodeHandle nh;

    /*Member Functions*/
    long UpTime();
    long UpTime(std::string pid);
    long ActiveJiffies(std::string pid);
    double computeCPUUsage(std::string pid); 
    double computeMemoryUsage(std::string pid);

    std::string pingPid(std::string nodeName);
    std::string getPid(std::string nodeName);
    std::string ElapsedTime(long elapsedSeconds); 


    void cpuStatistics(std::string &node_name);
    void memoryStatistics(std::string &node_name);
    void timeStatistics(std::string &node_name);
    void nodeStatus(std::string &node_name);
    void getErrorValueFromState(std::string &node_name,std::string &value, double &error_level);


    bool m_setup = false;
    double m_memUsage,m_cpuUsage;
    long m_upTime;

    std::vector<std::string> m_initialNodeList;
    std::vector<std::string> m_nodeListOriginal,m_nodeListCopy;
    std::unordered_map<std::string, std::string> m_nodeLog; 
    char m_nodeState;

    Monitor *m_monitor; ///< An object to monitor class
};


#endif