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
#include <memory>
#include "monitoring_core/monitor.h"
#include "utilities.h"


class SystemStatistics
{
public:

    /* Constructor for the Ros class */ 
    SystemStatistics();

     /* Destructor for the Ros class */
    ~SystemStatistics();

    void computeAndUpdateSystemStatistics();

private:
    ros::NodeHandle nh;

    /*Member Functions*/

    void computeSystemStatistics();
    void publishSystemStatistics();
    void publishAverageLoadStatistics();
    void publishTemperatureStatistics(int core ,double temperature);
    void publishCpuStatistics();
    void publishMemoryStatistics();
    void computeAverageCPULoad();

    int getNumberOfCores();
    int getCoreTemperature(int core);
    
    double computeMemoryUtilization(); 
    double computeCpuUtilization();
    
    long ActiveJiffies();
    long IdleJiffies();  
    
    std::vector<std::string> getCpuData();
   
    /*Member variables*/
    int m_numberOfCores;
    
    double m_coreTemperature[10] ={0.0};
    double m_averageLoad[3] ={0.0,0.0,0.0};
    double m_cpuPercentage,m_memoryPercentage;
    double m_cpuThreshold,m_memoryThreshold,m_temperatureThreshold,m_averageLoadThreshold;
    
    std::vector<std::string> m_cpuData; 
    std::mutex m_mutex;
    std::unordered_map<std::string ,double> systemErrorMap;
    //Monitor *m_monitor; ///< An object to monitor class
    std::shared_ptr<Monitor> m_monitor;

    XmlRpc::XmlRpcValue m_systemErrors;


};


#endif