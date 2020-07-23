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
#include "monitoring_core/monitor.h"
#include "utils.h"

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
    double computeMemoryUtilization(); 
    double computeCpuUtilization();
    long ActiveJiffies();
    long IdleJiffies();  
    std::vector<std::string> getCpuData();

    void updateAverageLoadStatus();
    void updateTemperatureStatus(int core ,double temperature);
    void updateCpuStatus();
    void updateMemoryStatus();

    float m_averageLoad[3] ={0.0,0.0,0.0};
    double m_cpuPercentage,m_memoryPercentage;
    double m_cpuThreshold,m_memoryThreshold,m_temperatureThreshold,m_averageLoadThreshold;
    //float m_averageLoad_1_min,m_averageLoad_5_min,m_averageLoad_15_min;
    std::vector<std::string> m_cpuData; 
    std::mutex m_mutex;
    Monitor *m_monitor; ///< An object to monitor class



};


#endif