
#include "rosbag_recorder.h"


RosBagRecorder::RosBagRecorder()
{
    /*Parameters from parameter server*/

    
    nh.getParam("/splitTime",m_splitTime);
    nh.getParam("/oldFileDeletionTime",m_oldFileDeletionTime);
    nh.getParam("/maxBagSize",m_maxBagSize);
    nh.getParam("/maxFolderSize", m_maxFolderSize);

    std::string path = ros::package::getPath("robot_diagnostics");
    m_rosBagFolder   = path + "/bags";
    m_lastTime  = Utilities::millis<uint64_t>();



    /*Publishers*/
    recordPub      = nh.advertise<bag_recorder::Rosbag>("/record/start",1);
    stopPub        = nh.advertise<std_msgs::String>("/record/stop", 10);



    /*Timer*/

    recordTimer  = nh.createTimer(ros::Duration(1.0), &RosBagRecorder::recordTimerCallback, this);
    clearTimer   = nh.createTimer(ros::Duration(1.0), &RosBagRecorder::clearTimerCallback, this);

    removeInactiveBags();

}  


RosBagRecorder::~RosBagRecorder()
{

}


/**
* @brief Timer that controls the recording of ros bags
*/
void RosBagRecorder::recordTimerCallback(const ros::TimerEvent &e)
{
    std::unique_lock<std::mutex> recordLock(m_mutex);
    uint64_t timeoutTime   = Utilities::millis<uint64_t>();
    uint64_t timeoutDelta  = timeoutTime - m_lastTime;

    
    bool bagSizeCondition = (GetBagSize(m_currentFileName) > m_maxBagSize);
    bool bagTimeCondition = (timeoutDelta > (m_splitTime*MINUTETOMILLIS));
    findActiveBag();
    if(!m_recording)
    {   
        startRecording();
        m_lastTime    = Utilities::millis<uint64_t>();  
    }

    else if(bagSizeCondition || bagTimeCondition)
    {
        stopRecording(); 
        m_lastTime    = Utilities::millis<uint64_t>();  
    }


    
    GetBagFiles(m_fileNames);
    GetFolderSize();
    std::cout << "Time : "<< timeoutDelta << std::endl;
}

/**
* @brief Timer that controls the clearing of older ros bags
*/
void RosBagRecorder::clearTimerCallback(const ros::TimerEvent &e)
{
    if(m_totalBagsSize > m_maxFolderSize)
    {
        ROS_ERROR("Folder size exceeded limits : %f",m_totalBagsSize);
    }
    for(int i=0; i < m_fileNames.size(); i++)
    {
        std::string checkFile = m_rosBagFolder +"/"+ m_fileNames[i];
        ROS_INFO("Files to be removed : %s  time : %d ",checkFile.c_str(),timeFromLastModification(checkFile));
        /*If older files are found*/
        if(timeFromLastModification(checkFile) > (m_oldFileDeletionTime*MINUTETOSECONDS))
        {
            std::unique_lock<std::mutex> removelock(m_mutex);
            removeBagFile(checkFile);
            m_fileNames.erase(std::remove(m_fileNames.begin(), m_fileNames.end(), m_fileNames[i]), m_fileNames.end());
        }
        /*In case if total bag size greater than max size alloted,clear all*/
        else if(m_totalBagsSize > m_maxFolderSize)
        {
            ROS_ERROR("Clearing entire folder");
            std::unique_lock<std::mutex> removealllock(m_mutex);
            removeBagFile(checkFile);
            m_fileNames.erase(std::remove(m_fileNames.begin(), m_fileNames.end(), m_fileNames[i]), m_fileNames.end());    
        }
    }
}

/**
* @brief Initiates the recording of ROS bags
*/
void RosBagRecorder::startRecording()
{
    bag_recorder::Rosbag recordMessage;
    recordMessage.header.stamp = ros::Time::now();
    recordMessage.config    = "standard";
    recordPub.publish(recordMessage);
    // m_currentFileName   = m_rosBagFolder + "/" +GetTimeStr();
    // m_currentFileName = m_currentFileName + ".bag.active";
    ROS_ERROR("Record started....  : %s ",m_currentFileName.c_str());

    m_recording = true;
}

/**
* @brief Stops the recording of ROS bags
*/
void RosBagRecorder::stopRecording()
{
    std_msgs::String stopMessage;
    stopMessage.data = "standard";
    stopPub.publish(stopMessage);
    ROS_ERROR("Record stopped....");

    m_recording = false;
}

/**
* @brief Computes ros bag size
* @return bag size in MB
*/
double RosBagRecorder::GetBagSize(std::string bagName)
{
    std::streampos begin,end;
    std::ifstream myfile (bagName, std::ios::binary);
    begin = myfile.tellg();
    myfile.seekg (0, std::ios::end);
    end = myfile.tellg();
    myfile.close();
    double sizeMB = (end - begin)*BYTESTOMB;
    ROS_WARN("%s : is with size : %f",bagName.c_str(),sizeMB );
    return sizeMB;    
}


/**
* @brief Gets all the bags  in the given bagFolder
*/
void RosBagRecorder::GetBagFiles(std::vector<std::string> &fileNames)
{
    DIR* dirp = opendir(m_rosBagFolder.c_str());
    struct dirent * dp;
    int bagCount = 0;
    while ((dp = readdir(dirp)) != NULL) 
    {
        bagCount++;
        std::vector<std::string>::iterator it = std::find(fileNames.begin(), fileNames.end(), dp->d_name);
        if(it == fileNames.end())
        {
            std::string currentFile = dp->d_name;
            if(( currentFile == ".") || (currentFile  == ".."))
            {
                continue;
                //ROS_ERROR("File fail cases,please ignore");
            }
            else
            {
                std::string active = "active";
                if (currentFile.find(active) == std::string::npos)
                {
                    fileNames.push_back(currentFile);
                }
        
            }
        }
    }
    std::cout << "Total number of bags present "<< bagCount -2 << std::endl;
   
    closedir(dirp);
    //return (bagCount - 2);
}




void RosBagRecorder::GetFolderSize()
{
    double folderSize =0.0;
    for(int i =0; i< m_fileNames.size();i++)
    {
        std::string checkFile = m_rosBagFolder + "/" +m_fileNames[i];
        folderSize = folderSize + GetBagSize(checkFile);
    }
    m_totalBagsSize = folderSize;
    ROS_INFO("Total bag size : %f",m_totalBagsSize);
}
/**
* @brief Remove the unncessary .active files if any
*/
void RosBagRecorder::removeInactiveBags()
{
    DIR* dirp = opendir(m_rosBagFolder.c_str());
    struct dirent * dp;
    int bagCount = 0;
    while ((dp = readdir(dirp)) != NULL) 
    {
        std::string currentFile = m_rosBagFolder + "/" + dp->d_name;
        ROS_INFO("Existing files : %s",currentFile.c_str());
        std::string activeTag = "active";
        if(currentFile.find(activeTag) != std::string::npos)
        {
            removeBagFile(currentFile);
        }
    }    
}

/**
* @brief Finds the current active file
*/
void RosBagRecorder::findActiveBag()
{
    DIR* dirp = opendir(m_rosBagFolder.c_str());
    struct dirent * dp;
    int bagCount = 0;
    while ((dp = readdir(dirp)) != NULL) 
    {
        std::string currentFile = dp->d_name;
        //ROS_INFO("Existing files : %s",currentFile.c_str());
        std::string activeTag = "active";
        if(currentFile.find(activeTag) != std::string::npos)
        {
            m_currentFileName   = m_rosBagFolder + "/" +currentFile;
        }
    }    
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

/**
* @brief Computes the last modified time of the file
* @return the seconds passed from last modified instance
*/
int RosBagRecorder::timeFromLastModification(std::string bagName)
{
    struct stat t_stat;
    time_t now;
    int seconds;

    stat(bagName.c_str(), &t_stat);
    struct tm * timeinfo = localtime(&t_stat.st_ctime); // or gmtime() depending on what you want
    time(&now);  /* get current time; same as: now = time(NULL)  */

    seconds = difftime(now,mktime(timeinfo));
    return seconds;
}

/**
* @brief Remove the given bag file
*/
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
        ROS_ERROR("Bag  %s deleted successfully ",bagName.c_str());
    }
}

/**
 * @brief Main function
*/
int main(int argc, char** argv)
{
    ros::init(argc, argv, "RosBagRecorder");
    ROS_INFO("Main of RosBagRecorder called");
    
    RosBagRecorder rosBagRecorder;

    ros::NodeHandle n;
    ros::Rate rate(1);
    
    while(ros::ok())
    {
        
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
