#ifndef ROSBAG_RECORDER_H
#define ROSBAG_RECORDER_H

#include <iostream>
#include <cstdio>
#include <ros/ros.h>


#include <bag_recorder/Rosbag.h>

#include <std_msgs/String.h>
#include <fstream>
#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>

#include <time.h>
#include <stdio.h>
#include <vector>
#include <utilities.h>
class RosBagRecorder
{
public:

    /* Constructor for the RosBagRecorder class */ 
    RosBagRecorder();

     /* Destructor for the RosBagRecorder class */
    ~RosBagRecorder();
    



   
private:

    ros::NodeHandle nh;

    /*Subscribers*/

    /*Publishers*/
    
    ros::Publisher recordPub;
    ros::Publisher stopPub;


    /*Timer*/
    
    ros::Timer recordTimer; 
    ros::Timer clearTimer; 

     /*Member functions*/

    void recordTimerCallback(const ros::TimerEvent &e);
    void clearTimerCallback(const ros::TimerEvent &e);
    void startRecording(); 
    void stopRecording();
    void removeBagFile(std::string bagName);
    void GetBagSize(std::string bagName);
    int GetBagCount(std::string bagFolder,std::vector<std::string> &fileNames);
    int timeFromLastModification(std::string bagName);
    std::string GetTimeStr();
   // std::size_t number_of_files_in_directory(std::filesystem::path path);

    /*Member variables*/
    std::string m_rosBagFolder;
    std::vector <std::string> m_fileNames;

    int m_splitTime;
    bool m_recording = false;

    uint64_t m_lastTime;
};


#endif