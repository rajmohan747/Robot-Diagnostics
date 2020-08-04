#ifndef ANALYSER_H
#define ANALYSER_H

#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <unordered_map>
#include <unistd.h>
/*! Criticality enum for defining the criticality of monitoring errors */
enum ErrorStatus
{
    Critical,    /*!< Monitoring error is critical */
    NonCritical, /*!< Monitoring error is non critical */
    Ok,          /*!< No monitoring error */
    Stale        /*!< Default monitoring error type, no analyser data */
};

class Analyser
{
public:

    /* Constructor for the Analyser class */ 
    Analyser(ros::NodeHandle &nh);

     /* Destructor for the Analyser class */
    ~Analyser();

    void getMonitorKey(std::string &key);
    void getMonitorType(const std::string &key,std::string &type);
    void getMonitorAction(std::string &action);

private:




    
    XmlRpc::XmlRpcValue m_monitorParams;
};

#endif /*ANALYSER_H*/