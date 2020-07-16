#ifndef SYSTEM_STATISTICS_H
#define SYSTEM_STATISTICS_H

#include <iostream>
#include <ros/ros.h>
#include <fstream>
#include <vector>
#include <fstream>
#include <unistd.h>
#include "unordered_map"
#include <mutex>

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
class SystemStatistics
{
public:

    /* Constructor for the Ros class */ 
    SystemStatistics();

     /* Destructor for the Ros class */
    ~SystemStatistics();

    void updateSystemStatistics();
private:
    ros::NodeHandle nh;

    /*Member Functions*/
    void getAverageCPULoad();
    int getNumberOfCores();
    int getCoreTemperature(int core);
    double MemoryUtilization(); 
    double Utilization();
    long ActiveJiffies();
    long IdleJiffies();  
    std::vector<std::string> CpuUtilization();

    float m_averageLoad[3] ={0.0,0.0,0.0};
    //float m_averageLoad_1_min,m_averageLoad_5_min,m_averageLoad_15_min;

    std::mutex m_mutex;


};


#endif