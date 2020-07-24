#include "node_statistics.h"

    /**
    * @brief  Constructor for the TrajectoryController
    */

NodeStatistics::NodeStatistics(ros::NodeHandle &nh,std::string topicName)
{
    m_nodeName    = topicName;
    
    double timerUpdateFrequency = 1.0;
    nh.getParam("/timerUpdateFrequency", timerUpdateFrequency);
    nh.getParam("/maxPermissibleNodeRestart", m_maxPermissibleNodeRestart);
    nh.getParam("/maxPermissibleCpuUsage", m_maxPermissibleCPUUsage);
    nh.getParam("/maxPermissibleMemoryUsage", m_maxPermissibleMemoryUsage);
    m_ramSize     = getRamSize();

   /*Timer*/
    nodeStatusTimer = nh.createTimer(ros::Duration(1.0 / timerUpdateFrequency), &NodeStatistics::timerCallback, this);
    ROS_WARN("NodeStatistics constructor initialized for node : %s with max restart : %d",m_nodeName.c_str(),m_maxPermissibleNodeRestart);   
    //m_monitor = new Monitor(nh, "Topic Hz Monitor", timerUpdateFrequency);
    //std::shared_ptr<Monitor> m_monitor(new Monitor(nh, "Topic Hz Monitor", timerUpdateFrequency));
    m_monitor  = std::make_shared<Monitor>(nh, "Topic Hz Monitor", timerUpdateFrequency);
}

/**
* @brief  Destructor for the FrequencyStatistics
*/

NodeStatistics::~NodeStatistics()
{

}

void NodeStatistics::timerCallback(const ros::TimerEvent &e)
{

    m_isAvailable = isNodeAvailable(m_nodeName);
    if((m_nodeRestartCount < m_maxPermissibleNodeRestart) && m_isAvailable)
    {
        monitorNodeStatistics();
    }
    else
    {
      ROS_ERROR("Node %s has got restarted %d times",m_nodeName.c_str(),m_nodeRestartCount );
       publishNodeUnavailableInfo();
    }
    
    

}

/**
* @brief  Monitor the different statistics of the given node
*/

void NodeStatistics::monitorNodeStatistics()
{

    std::string currentPid = getPid(m_nodeName);

    if(m_nodeSetup == false)
    {
        m_nodeLog[currentPid] = m_nodeName;
        m_nodeSetup           = true;
    }
    else if (m_nodeLog.find(currentPid) == m_nodeLog.end())
    {
        m_nodeRestartCount++;
        ROS_WARN("Node : %s is killed with last pid %s for %d  th  time",m_nodeName.c_str(),m_lastPid.c_str(),m_nodeRestartCount);
        m_nodeLog.clear();
        m_nodeLog[currentPid] = m_nodeName;
        
        
    }   
    else
    {
        m_lastPid   = currentPid;
        if(m_isAvailable)
        {
            m_memPercentage     = computeNodeMemoryPercentage(currentPid)*100/(m_ramSize/1024); 
            m_cpuPercentage     = computeNodeCPUPercentage(currentPid);
            //ROS_INFO("Node : %s  with  PID  %s mem % : %f  cpu % : %f ",m_nodeName.c_str(),currentPid.c_str(),m_memPercentage,m_cpuPercentage);
            updateNodePingStatus(m_nodeName);
            publishNodeStatistics();
        }
        
    }
}



/**
* @brief Computes the PID for a particular node
* @returns the PID as a string
*/
std::string NodeStatistics::getPid(std::string nodeName)
{

  std::string data;
  int max_buffer = 256;
  char buffer[max_buffer]; 
  std::string str = "rosnode info " + nodeName +" 2>/dev/null | grep Pid| cut -d' ' -f2";

  /*The system command is often run first, before any output commands and the function 
  returns an integer indicating success or failure, but not the output of the string*/

  /*Opens up a read-only stream, runs the command and captures the output,
   stuffs it into the buffer and returns it as a string.*/
//  std::unique_lock<std::mutex> lock (m_mutex);
  FILE *stream = popen(str.c_str(), "r");
  if (stream) 
  {
    while (!feof(stream))
    {
        if (fgets(buffer, max_buffer, stream) != NULL) 
        {
        	data.append(buffer);
        }  
        
    }
    
    pclose(stream);


    /*To avoid the new line character by the end of data*/
    return data.substr(0, data.length() - 1);
   }	
}



/**
* @brief Computes the CPU usage for a particular PID
* @returns the cpu usage in %
* @ref https://stackoverflow.com/questions/16726779/how-do-i-get-the-total-cpu-usage-of-an-application-from-proc-pid-stat/16736599#16736599
*/
double NodeStatistics::computeNodeCPUPercentage(std::string pid)
{
    double active_jiffies = (float)(ActiveJiffies(pid));
    m_upTime       = (float)(UpTime(pid));
    double cpuPercentage = 100*((active_jiffies/sysconf(_SC_CLK_TCK))/m_upTime);
    return cpuPercentage;
}



/**
* @brief Computes the RAM usage for a particular PID
* @returns the RAM usage in Mb
*/
double NodeStatistics::computeNodeMemoryPercentage(std::string pid)
{
  std::string line,key,unit;
  long value;
  std::ifstream inFile(kProcDirectory + "/" + pid + kStatusFilename);
//  std::unique_lock<std::mutex> lock (m_mutex);
  if(inFile.is_open())
  {
  	while(std::getline(inFile,line))
  	{
  		std::replace(line.begin(),line.end(),':',' ');
  		std::istringstream linestream(line);
  		linestream>>key>>value>>unit;
  		if(key == "VmSize")
  		{
  			double convertedValue = value * 0.001;
  			//std::cout << "Ram Usage of "<< pid << " : " << convertedValue << std::endl;
            return convertedValue;
        }
  	}
  }
  return -1.0;
}



/**
* @brief Computes the total time spend since the process started
* @returns the time in seconds
*/

/**
* @brief Computes the total time spend since the process started
* @returns the time in seconds
*/
long NodeStatistics::ActiveJiffies(std::string pid) 
{ 
  long activeJiffies;
  std::string line;
  std::string value;
  std::vector<std::string> dataSet;


  std::ifstream inFile(kProcDirectory + "/" + pid +kStatFilename);
  if (inFile.is_open()) 
  {
    std::getline(inFile, line);/*Can USE while(inFile) instead of with !inFile.eof()*/
    std::istringstream linestream(line);
    while (linestream >> value) 
    {
      dataSet.push_back(value);
    }
  }
  activeJiffies  = std::stol(dataSet[13]) + std::stol(dataSet[14]) + std::stol(dataSet[15]) + stol(dataSet[16]);
  return activeJiffies; 
}



/**
* @brief Computes the up time for a particular PID
* @returns the uptime in seconds
*/
long NodeStatistics::UpTime(std::string pid)
{
  std::string line;
  std::string value;
  long seconds;
  std::vector<std::string> input;
  std::ifstream filestream(kProcDirectory + pid + kStatFilename);
//  std::unique_lock<std::mutex> lock (m_mutex);
  if (filestream.is_open()) 
  {
    std::getline(filestream, line);
    std::istringstream linestream(line);
      while (linestream >> value) 
      {
        input.push_back(value);
      }
  }
  seconds     = std::stol(input[21]);
  m_nodeState = input[2][0];
  //std::cout<< "New update "<< m_nodeState <<std::endl;
  seconds = seconds/sysconf(_SC_CLK_TCK);
  return UpTime() - seconds;

}


/**
* @brief Computes the up time for the system
* @returns the uptime in seconds
*/

long NodeStatistics::UpTime() 
{
  std::string line;
  long upTime, idleTime;
  std::ifstream inFile(kProcDirectory + kUptimeFilename);
//  std::unique_lock<std::mutex> lock (m_mutex);
  if (inFile.is_open()) 
  {
    std::getline(inFile, line);
    std::istringstream linestream(line);
    linestream >> upTime >> idleTime;
    
  }
  return upTime;
}




/**
* @brief Checks whether a node is alive or not by pinging it
*/
void NodeStatistics::updateNodePingStatus(std::string &node_name)
{
  std::string nodeXmlrpcURI = getNodeXmlrpcURI(node_name);

  //std::cout << "retun "<< nodeXmlrpcURI <<std::endl;
  std::string data;
  int max_buffer = 256;
  char buffer[max_buffer]; 
  std::string str = "rosnode ping -a | grep " + nodeXmlrpcURI;
/*Opens up a read-only stream, runs the command and captures the output,
   stuffs it into the buffer and returns it as a string.*/
  std::unique_lock<std::mutex> lock (m_mutex);
  FILE *stream = popen(str.c_str(), "r");
  if (stream) 
  {
    while (!feof(stream))
    {
        if (fgets(buffer, max_buffer, stream) != NULL) 
        {
        	data.append(buffer);
        }  
        
    }
    
    pclose(stream);

    std::istringstream linestream(data);
    std::string l1,l2,l3,l4,l5;

    linestream>>l1>>l2>>l3>>l4>>l5;
   // std::string key = node_name+"/ping_rate";
    float value =std::stof(l5.substr(5,5));
    ROS_WARN("Node %s  ping rate  is %f",m_nodeName.c_str(),value);
   // double error_level = 0.0; 
   // m_monitor->addValue(key, value, "ms", error_level, AggregationStrategies::FIRST);
   }	  
}




/**
* @returns the nodexmlrpcURI corresponding to the node provided
*/
std::string NodeStatistics::getNodeXmlrpcURI(std::string &node_name)
{
  std::string data;
  int max_buffer = 256;
  char buffer[max_buffer]; 
  std::string str = "rosnode list -a | grep " + node_name;
/*Opens up a read-only stream, runs the command and captures the output,
   stuffs it into the buffer and returns it as a string.*/
  std::unique_lock<std::mutex> lock (m_mutex);
  FILE *stream = popen(str.c_str(), "r");
  if (stream) 
  {
    while (!feof(stream))
    {
        if (fgets(buffer, max_buffer, stream) != NULL) 
        {
        	data.append(buffer);
          
        }  
       //std::cout << "data " << data <<std::endl; 
    }
    
    pclose(stream);

    std::istringstream linestream(data);
    std::string l1,l2;
    linestream>>l1>>l2;
    //std::cout << "fun " << l1 << std::endl;
    /*To avoid the new line character by the end of data*/
    return l1;
   }	
}


void NodeStatistics::publishNodeStatistics()
{
  if(m_cpuPercentage > m_maxPermissibleCPUUsage)
  {
    publishNodeCpuUsage(m_nodeName);
  }
  if(m_memPercentage > m_maxPermissibleMemoryUsage)
  {
    publishNodeMemoryUsage(m_nodeName);
  }
}


void NodeStatistics::publishNodeUnavailableInfo()
{
  std::string key = m_nodeName + " is unavailable/restarted beyond limit";
  m_monitor->addValue(key, m_nodeRestartCount, "times", 0.9, AggregationStrategies::FIRST);
}

void NodeStatistics::publishNodeCpuUsage(std::string &node_name)
{
    std::unique_lock<std::mutex> lock (m_mutex);
    std::string key = node_name + "/cpu_usage";
    m_monitor->addValue(key, m_cpuPercentage, "%", 0.3, AggregationStrategies::FIRST);
}


void NodeStatistics::publishNodeMemoryUsage(std::string &node_name)
{
    std::unique_lock<std::mutex> lock (m_mutex);
    std::string key = node_name + "/memory_usage";    
    m_monitor->addValue(key, m_memPercentage, "%", 0.3, AggregationStrategies::FIRST);
}



/**
* @brief Acquires the RAM size of the processor
* @returns the RAM size in Kb
*/
long NodeStatistics::getRamSize() 
{
  std::string line, key;
  long int value, MemTotal;
  std::ifstream inFile(kProcDirectory + kMeminfoFilename);
//  std::unique_lock<std::mutex> lock (m_mutex);
  if (inFile.is_open()) {
    while (std::getline(inFile, line)) 
    {
      std::replace(line.begin(), line.end(), ':', ' ');
      std::istringstream linestream(line);
      linestream >> key >> value;

      if (key == "MemTotal") 
      {
        MemTotal = static_cast<float>(value);
        return MemTotal;
      }
    }
  }
  return -1.0;
}


/**
* @brief checks whehter the given node name is currently registered with ros master
* @returns true if the node is available,else false;
*/
bool NodeStatistics::isNodeAvailable(std::string &node_name)
{
  std::vector<std::string> currentNodeList;
  ros::master::getNodes(currentNodeList);
  for(auto node : currentNodeList)
  {
    if(node == node_name)
    {
      return true;
    }
  }
  return false;
}
