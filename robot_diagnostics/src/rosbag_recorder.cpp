
#include "rosbag_recorder.h"


RosBagRecorder::RosBagRecorder()
{
    /*Parameters from parameter server*/

    nh.getParam("/rosBagFolder", m_rosBagFolder);
    nh.getParam("/splitTime",m_splitTime);


    /*Subscribers*/
 //   nodeSubscriber.subscribe(nh, "/node_monitoring/monitoring", 1);


    /*Publishers*/
    recordPub      = nh.advertise<bag_recorder::Rosbag>("/record/start",1);
    stopPub        = nh.advertise<std_msgs::String>("/record/stop", 10);



    /*Timer*/

    recordTimer  = nh.createTimer(ros::Duration(1.0), &RosBagRecorder::recordTimerCallback, this);
    clearTimer   = nh.createTimer(ros::Duration(1.0), &RosBagRecorder::clearTimerCallback, this);

    std::string bagName   = "/home/nvidia/catkin_ws/src/robot_diagnostics/src/example.txt";
    std::string bagFolder = "/home/nvidia/catkin_ws/src/robot_diagnostics/src";
    std::string currentFileName = "/home/nvidia/dummy.txt";
    GetBagSize(bagName);
    int folderSize = GetBagCount(m_rosBagFolder,m_fileNames);
    int second = timeFromLastModification(currentFileName);


    m_lastTime  = Utilities::millis<uint64_t>();
    

}  


RosBagRecorder::~RosBagRecorder()
{

}



void RosBagRecorder::recordTimerCallback(const ros::TimerEvent &e)
{
    ROS_INFO("Current file size : %d",m_fileNames.size());
    uint64_t timeoutTime   = Utilities::millis<uint64_t>();
    uint64_t timeoutDelta  = timeoutTime - m_lastTime;
    if(!m_recording)
    {
        startRecording();
        m_lastTime = Utilities::millis<uint64_t>();  
    }

    if(timeoutDelta > m_splitTime)
    {
        stopRecording();
        
    }
    int folderSizes = GetBagCount(m_rosBagFolder,m_fileNames);
    //int folderSize = GetBagCount(m_rosBagFolder,m_fileNames);
    std::cout << "Time : "<< timeoutDelta << std::endl;
}

void RosBagRecorder::clearTimerCallback(const ros::TimerEvent &e)
{
    for(int i=0; i < m_fileNames.size(); i++)
    {
        std::string checkFile = m_rosBagFolder +"/"+ m_fileNames[i];
        //ROS_INFO("Files to be removed : %s  time : %d",checkFile.c_str(),timeFromLastModification(checkFile));
        if(timeFromLastModification(checkFile) > 20)
        {
            ROS_ERROR("File  %s needs to be removed",checkFile.c_str());
            removeBagFile(checkFile);
            m_fileNames.erase(std::remove(m_fileNames.begin(), m_fileNames.end(), m_fileNames[i]), m_fileNames.end());
        }
    }
}

void RosBagRecorder::startRecording()
{
    bag_recorder::Rosbag recordMessage;
    recordMessage.header.stamp = ros::Time::now();
    recordMessage.config    = "standard";
    //recordMessage.bag_name  = "recording";
    recordPub.publish(recordMessage);
    std::string startTime= GetTimeStr();
    ROS_WARN("Record started....  : %s ",startTime.c_str());

    m_recording = true;
}


void RosBagRecorder::stopRecording()
{
    std_msgs::String stopMessage;
    stopMessage.data = "standard";
    stopPub.publish(stopMessage);
    ROS_WARN("Record stopped....");

    m_recording = false;
}


 void RosBagRecorder::GetBagSize(std::string bagName)
{
    std::streampos begin,end;
    std::ifstream myfile (bagName, std::ios::binary);
    begin = myfile.tellg();
    myfile.seekg (0, std::ios::end);
    end = myfile.tellg();
    myfile.close();
    std::cout << "size is: " << (end-begin) << " bytes.\n";
    std::string dir_path;
    
 }

int RosBagRecorder::GetBagCount(std::string bagFolder,std::vector<std::string> &fileNames)
{
    DIR* dirp = opendir(bagFolder.c_str());
    struct dirent * dp;
    int bagCount = 0;
    while ((dp = readdir(dirp)) != NULL) 
    {
        
        std::vector<std::string>::iterator it = std::find(fileNames.begin(), fileNames.end(), dp->d_name);
        if(it == fileNames.end())
        {
            std::string currentFile = dp->d_name;
            if(( currentFile == ".") || (currentFile  == ".."))
            {
                ROS_ERROR("File fail cases,please ignore");
            }
            else
            {
                fileNames.push_back(currentFile);
                bagCount++;
            }
        }
        // v.push_back(dp->d_name);
    }
    std::cout << "Total number of bags present "<< bagCount << std::endl;
    ROS_INFO("File size : %d",fileNames.size());
    closedir(dirp);
    return bagCount;
}


std::string RosBagRecorder::GetTimeStr()
{
    /* Return file name as current time */
    time_t time = std::time(nullptr);
    tm local_time = *std::localtime(&time);

    std::ostringstream os;
    os << std::put_time(&local_time, "_%Y-%m-%d-%H-%M-%S");
    
    return os.str();
}


int RosBagRecorder::timeFromLastModification(std::string bagName)
{
    struct stat t_stat;
    time_t now;
    int seconds;

    stat(bagName.c_str(), &t_stat);
    struct tm * timeinfo = localtime(&t_stat.st_ctime); // or gmtime() depending on what you want
    time(&now);  /* get current time; same as: now = time(NULL)  */

    seconds = difftime(now,mktime(timeinfo));

    //std::cout <<"Time passed : "<< seconds << std::endl;
    return seconds;
}


void RosBagRecorder::removeBagFile(std::string bagName)
{
    /*std::string to char conversion*/
    char filename[(bagName.length() + 1)];
    strcpy(filename,bagName.c_str());
	
    if (remove(filename) != 0)
	{
        ROS_ERROR("File Deletion Failed : %s", bagName.c_str());
    }
	else
	{
        ROS_WARN("Bag  %s deleted successfully ",bagName.c_str());
    }
}

/**
 * @brief Main function
*/
int main(int argc, char** argv)
{
    ros::init(argc, argv, "RosBagRecorder");
    ROS_INFO("Main of RosBagRecorder called");
    /* Create an object to DiagnosticsAggregator */
    RosBagRecorder diagnosticsAggregator;

    ros::NodeHandle n;
    ros::Rate rate(1);
    
    while(ros::ok())
    {
        //diagnosticsAggregator.errorCategorization();
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
