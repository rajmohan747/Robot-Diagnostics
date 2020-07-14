#ifndef SYSTEM_STATISTICS_H
#define SYSTEM_STATISTICS_H

#include <iostream>
#include <ros/ros.h>
#include <fstream>
#include <vector>
#include <fstream>
#include <unistd.h>
#include "unordered_map"


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

    void updateStatistics();
private:
    ros::NodeHandle nh;

    /*Member Functions*/

    double MemoryUtilization(); 
    std::vector<std::string> CpuUtilization();
    long ActiveJiffies();
    long IdleJiffies();  
    double Utilization();
    int getNumberOfCores();
    int getCoreTemperature(int core);


};


#endif