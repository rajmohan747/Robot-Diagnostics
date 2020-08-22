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
#include <mutex>
#include <chrono>
#include <ros/package.h>

#define MINUTETOMILLIS 60000 //60000
#define MINUTETOSECONDS 60  //60
#define BYTESTOMB 0.000001
class RosBagRecorder
{
public:
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
    void removeInactiveBags();
    void findActiveBag();
    void GetFolderSize();
    double GetBagSize(std::string bagName);
    void GetBagFiles(std::vector<std::string> &fileNames);
    int timeFromLastModification(std::string bagName);
    std::string GetTimeStr();
   // std::size_t number_of_files_in_directory(std::filesystem::path path);

    /*Member variables*/
    std::string m_rosBagFolder;
    std::string m_currentFileName;
    std::vector <std::string> m_fileNames;
    std::mutex m_mutex;

    

    int m_splitTime;
    int m_oldFileDeletionTime;
    double m_maxBagSize,m_maxFolderSize;
    double m_totalBagsSize;
    bool m_recording = false;

    uint64_t m_lastTime;
};


#endif