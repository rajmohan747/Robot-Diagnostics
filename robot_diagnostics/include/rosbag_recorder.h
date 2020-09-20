#ifndef ROSBAG_RECORDER_H
#define ROSBAG_RECORDER_H

#include <iostream>
#include <cstdio>
#include <ros/ros.h>


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
#include <queue> 
#include <thread>
#include <unistd.h>
#define MINUTETOMILLIS 60000 //60000
#define MINUTETOSECONDS 60  //60
#define BYTESTOMB 0.000001
#define BAGDELETIONSECONDS 30 

struct BagInfo
{
    int seconds;
    std::string bagName;

};

struct CompareError 
{ 
    bool operator()(BagInfo const& p1, BagInfo const& p2) 
    { 
        return p1.seconds < p2.seconds; 
    } 
}; 

class RosBagRecorder
{
public:
    RosBagRecorder();

     /* Destructor for the RosBagRecorder class */
    ~RosBagRecorder();
    
    void initializeRecording();
    void autoBagDeletion();


   
private:

    ros::NodeHandle nh;

    /*Subscribers*/

    /*Publishers*/
    



    /*Timer*/
    


     /*Member functions*/
    

    void removeBagFile(std::string bagName);
    void removeInactiveBags();
    void removeAllExcessFiles();
    void findActiveBag();
    void GetFolderSize();
    void initializeParameters();
    void GetBagFiles(std::vector<std::string> &fileNames);
    int timeFromLastModification(std::string bagName);
    double GetBagSize(std::string bagName);
    
   
   // std::size_t number_of_files_in_directory(std::filesystem::path path);

    /*Member variables*/
    std::string m_rosBagFolder;
    std::string m_currentFileName;
    std::string m_bagDirectory;
    std::vector <std::string> m_fileNames;
    std::vector<std::string> m_topicList;
    std::mutex m_mutex;

    

    int m_splitTime;
    int m_oldFileDeletionTime;
    int m_maxSplit;
    double m_maxBagSize,m_maxFolderSize;
    double m_totalBagsSize;
    bool m_recording = false;
    bool m_findActive = false;

    
    uint64_t m_currentTime,m_previousTime;


    BagInfo m_bagInfo;

    std::priority_queue<BagInfo, std::vector<BagInfo>, CompareError> m_errorQueue;

};


#endif