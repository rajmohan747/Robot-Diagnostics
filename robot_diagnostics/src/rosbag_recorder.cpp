
#include "rosbag_recorder.h"


RosBagRecorder::RosBagRecorder()
{
    /*Parameters from parameter server*/
    initializeParameters();


    /*Publishers*/
    //recordPub      = nh.advertise<bag_recorder::Rosbag>("/record/start",1);
    //stopPub        = nh.advertise<std_msgs::String>("/record/stop", 10);


    /*Incase previously node not closed properly, ".active" files will be left out
    Needs to clear those as well*/
    removeInactiveBags();


    //initializeRecording();
    // /*Inorder to handle the cases where node gets restarted and the last active bag is
    // still up*/
    //stopRecording();


    
    


    /*Timer*/
    //recordTimer  = nh.createTimer(ros::Duration(1.0), &RosBagRecorder::recordTimerCallback, this);
    //clearTimer   = nh.createTimer(ros::Duration(4.0), &RosBagRecorder::clearTimerCallback, this);


    
}  


RosBagRecorder::~RosBagRecorder()
{

}

/**
* @brief Initialize the parameters
*/
void RosBagRecorder::initializeParameters()
{
    nh.getParam("/bagDirectory",m_bagDirectory);
    nh.getParam("/splitTime",m_splitTime);
    nh.getParam("/oldFileDeletionTime",m_oldFileDeletionTime);
    nh.getParam("/maxBagSize",m_maxBagSize);
    nh.getParam("/maxFolderSize", m_maxFolderSize);
    nh.getParam("maxSplit",m_maxSplit);
    nh.getParam("/bag_topics", m_topicList);
    std::string packagePath = ros::package::getPath("robot_diagnostics");
    m_rosBagFolder          = packagePath + "/bags";
    m_currentTime           = m_previousTime  = Utilities::millis<uint64_t>();
}


/**
* @brief System command to configure the bag recording settings
* This will be doing the bag splitting,controls number of max splits,removal of old bags from the command execution time etc
*/
void RosBagRecorder::initializeRecording()
{
    std::string topicString;
    for(int i=0; i < m_topicList.size(); i++)
    {
        topicString = topicString +" "+ m_topicList[i];
    }

    std::string recordCommand = "rosbag record -o " + m_bagDirectory + " --split --duration " + std::to_string(m_splitTime) + "m --max-splits " + std::to_string(m_maxSplit) + topicString;
    ROS_WARN("The command is : %s",recordCommand.c_str());
    const char *command = recordCommand.c_str(); 
    system(command);
}


/**
* @brief Ensures the folder doesn't exceeds the bag numbers/sizes than predefined values
* This will take care of the unhandled previous bags ,if any
*/

void RosBagRecorder::autoBagDeletion()
{
    while(ros::ok())
    {
        m_currentTime  = Utilities::millis<uint64_t>();
        auto diffTime  = m_currentTime - m_previousTime;
        if(m_currentTime - m_previousTime > (1000*BAGDELETIONSECONDS))
        {
            /*Finds the currently active bag*/
            
            if(m_findActive == false)
            {
                findActiveBag();
            }
            GetBagFiles(m_fileNames);
            GetFolderSize();


            for(int i=0; i < m_fileNames.size(); i++)
            {
                std::string bagAddress = m_rosBagFolder +"/"+ m_fileNames[i];

                bool oldFileDetected  = (timeFromLastModification(bagAddress) > (m_oldFileDeletionTime*MINUTETOSECONDS));
                bool folderSizeExceed = (m_totalBagsSize > m_maxFolderSize); 
                bool splitSizeExceed  = (m_errorQueue.size() > m_maxSplit);
                //ROS_INFO("Files to be removed : %s  time : %d ",bagAddress.c_str(),timeFromLastModification(bagAddress));
                /*If older files are found*/
                if(oldFileDetected)
                {
                    std::unique_lock<std::mutex> oldfilelock(m_mutex);
                    removeBagFile(bagAddress);
                    m_fileNames.erase(std::remove(m_fileNames.begin(), m_fileNames.end(), m_fileNames[i]), m_fileNames.end());
                    ROS_INFO("Old bag detected and removed");
                }
                /*In case if total bag size greater than max size alloted,clear all*/
                else if((folderSizeExceed) || (splitSizeExceed))
                {
                    std::unique_lock<std::mutex> excessfilelock(m_mutex);
                    removeAllExcessFiles();
                    ROS_INFO("Folder size or split size exceed condition");
                    //m_fileNames.erase(std::remove(m_fileNames.begin(), m_fileNames.end(), m_fileNames[i]), m_fileNames.end());    
                }
            }

            /*clearing the priority queue*/

            /*Ideally not reqd ,but to use priority queue clearing of both vector and
            p.queue are reqd,otherwise it might lead to duplication of values in p.queue*/
            m_fileNames.clear();
            m_fileNames.resize(0);
            m_errorQueue = std::priority_queue<BagInfo, std::vector<BagInfo>, CompareError>();



           // ROS_WARN("In test function %d",diffTime);
            m_previousTime = m_currentTime;

        }
        
        /*Adding some sleep so as the while loop not draining out the entire CPU*/
        sleep(10);
    }
    
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
            }
            else
            {
                std::string active = "active";
                if (currentFile.find(active) == std::string::npos)
                {
                    fileNames.push_back(currentFile);
                    
                    /*Data updation for the priority queues*/
                    std::string fileAddress = m_rosBagFolder + "/" + currentFile;
                    m_bagInfo.seconds = timeFromLastModification(fileAddress);
                    m_bagInfo.bagName = currentFile;
                    m_errorQueue.push(m_bagInfo);

                }
        
            }
        }
    }
    //std::cout << "Total number of bags present "<< bagCount -2 << std::endl;
   
    closedir(dirp);
    
}



/**
* @brief Get the entire folder size in MB
*/
void RosBagRecorder::GetFolderSize()
{
    double folderSize =0.0;
    for(int i =0; i< m_fileNames.size();i++)
    {
        std::string bagAddress = m_rosBagFolder + "/" +m_fileNames[i];
        folderSize = folderSize + GetBagSize(bagAddress);
    }
    m_totalBagsSize = folderSize;
    ROS_INFO("Total bag size : %f  MB",m_totalBagsSize);
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


void RosBagRecorder::removeAllExcessFiles()
{

    while (m_errorQueue.size() >= m_maxSplit) 
    { 
        BagInfo p = m_errorQueue.top(); 
        ROS_ERROR("P.queue :size  %d  file:   %s   time: %d",m_errorQueue.size(), (p.bagName).c_str(),p.seconds);
        
        std::string bagAddress = m_rosBagFolder +"/" + p.bagName;
        removeBagFile(bagAddress);
        /*TODO:Remove the unnecessary clears*/
        m_fileNames.erase(std::remove(m_fileNames.begin(), m_fileNames.end(), p.bagName), m_fileNames.end());
        m_errorQueue.pop();

    }
  
}

/**
* @brief Finds the currently active file
*/
void RosBagRecorder::findActiveBag()
{
    DIR* dirp = opendir(m_rosBagFolder.c_str());
    struct dirent * dp;
    int bagCount = 0;
    while ((dp = readdir(dirp)) != NULL) 
    {
        std::string currentFile = dp->d_name;
        std::string activeTag = "active";
        if(currentFile.find(activeTag) != std::string::npos)
        {
            m_currentFileName   = m_rosBagFolder + "/" +currentFile;
            //ROS_WARN("Current file detected as : %s",m_currentFileName.c_str());
            m_findActive        = true;
        }
    }    
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

    ros::Rate rate(1);
    std::thread t1  = std::thread(&RosBagRecorder::initializeRecording,&rosBagRecorder);
    std::thread t2  = std::thread(&RosBagRecorder::autoBagDeletion,&rosBagRecorder);
    //while(ros::ok())
    //{      
        
        t1.join();
        t2.join();
    //    rate.sleep();
    //    ros::spinOnce();
    //}
    return 0;
}
