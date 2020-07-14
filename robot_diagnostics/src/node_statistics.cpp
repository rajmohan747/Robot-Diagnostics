#include <ros/console.h> 
#include "node_statistics.h"

/**
* @brief  Constructor for the Statistics
*/
Statistics::Statistics()
{
   ROS_INFO("Statistics constructor called");
   nh.getParam("/nodes", m_initialNodeList);

   /*Gets all the nodes registered in the ROS master and stores it in a vector m_nodeListOriginal*/
   ros::master::getNodes(m_nodeListOriginal);

  bool prm = true;
  m_monitor = new Monitor(nh, "Node Statistic Monitor", true);
   
   //ROS_ERROR("Initial list size : %d",m_initialNodeList.size());
}

/**
* @brief  Destructor for the Statistics
*/

Statistics::~Statistics()
{

}



/**
* @brief Updates the node related statistics
*/
void Statistics::updateStatistics()
{
    m_nodeListCopy.clear();
    m_nodeListCopy.resize(0);
  	m_nodeListCopy = m_nodeListOriginal;
    //ROS_WARN("Node list size original : %d  copy : %d  inital  %d",nodeListOriginal.size(),nodeListCopy.size(),initialNodeList.size());

    for (std::vector<std::string>::iterator it(m_initialNodeList.begin()); it != m_initialNodeList.end(); ++it)
    {
      m_nodeListCopy.erase(std::remove(begin(m_nodeListCopy), end(m_nodeListCopy), *it), end(m_nodeListCopy));
    }

    
    for(std::vector<std::string>::iterator i(m_nodeListCopy.begin());i != m_nodeListCopy.end();++i)
    {
      //std::cout << *i << std::endl;
      std::string currentPid = getPid(*i);
     
      std::cout << *i << " 's PID is : " << currentPid << std::endl;

      //std::cout << *i << " 's PID  "<< currentPid << " uptime is : " << uptime << " : " << "RAM usage in MB : " << memUsage << " CPU Usage: "<< cpuUsage << std::endl;
      //std::string ping       = pingPid(*i);
      if(m_setup == false)
      {
        m_nodeLog[currentPid] = *i;
      }
      else
      {
        if(m_nodeLog.find(currentPid) == m_nodeLog.end())
        {
          ROS_ERROR("Node deleted %s",(*i).c_str());
        }
        else
        {
           m_memUsage     = computeMemoryUsage(currentPid); 
           //m_upTime       = UpTime(currentPid);
           m_cpuUsage     = computeCPUUsage(currentPid);
        }

      }

      cpuStatistics(*i);
      memoryStatistics(*i);
      timeStatistics(*i);
      nodeStatus(*i);
    }
    
    m_setup = true;


}


void Statistics::nodeStatus(std::string &node_name)
{
   std::string key = node_name;
   std::string value ="";
   double error_level = 0.0; 
   getErrorValueFromState(key,value,error_level);
   m_monitor->addValue(key, value, "", error_level, AggregationStrategies::FIRST);

}

void Statistics::getErrorValueFromState(std::string &node_name,std::string &value, double &error_level)
{
    /* Providing the desciption of the state based on state context */
    switch (m_nodeState)
    {
    case 'R':
        value = node_name + " is running";
        error_level = 0.0;
        break;
    case 'S':
        value = node_name + " is sleeping in an interruptible wait";
        error_level = 0.1;
        break;
    case 'D':
        value = node_name + " is waiting in uninterruptible disk sleep";
        error_level = 0.1;
        break;
    case 'T':
        value = node_name + " is stopped (on a signal) or trace stopped";
        error_level = 0.1;
        break;
    case 't':
        value = node_name + " is trace stopped";
        error_level = 0.1;
        break;
    case 'Z':
        value = node_name + " is a zombie";
        error_level = 0.3;
        break;
    case 'W':
        value = node_name + " is paging";
        error_level = 0.1;
        break;
    case 'X':
        value = node_name + " is Dead";
        error_level = 0.3;
        break;
    case 'x':
        value = node_name + " is Dead";
        error_level = 0.3;
        break;
    case 'K':
        value = node_name + " is wakekill";
        error_level = 0.3;
        break;
    case 'P':
        value = node_name + " is parked";
        error_level = 0.1;
        break;
    default:
        value = node_name + " is a zombie";
        error_level = 0.3;
    }
}



void Statistics::cpuStatistics(std::string &node_name)
{
    std::string key = node_name + "/CPU_Usage";
    m_monitor->addValue(key, m_cpuUsage, "%", 0.0, AggregationStrategies::FIRST);

}


void Statistics::memoryStatistics(std::string &node_name)
{
    std::string key = node_name + "/Memory_Usage";    
    m_monitor->addValue(key, m_memUsage, "Mb", 0.0, AggregationStrategies::FIRST);
}

void Statistics::timeStatistics(std::string &node_name)
{
    std::string key = node_name + "/CPU_Time";
    std::string time = "Time from start for "+  node_name + " : ";
    //time = time + ElapsedTime(m_upTime);
    //std::cout << time << std::endl;
    m_monitor->addValue(key, float(m_upTime), "seconds", 0.0, AggregationStrategies::FIRST);
}

/**
* @brief Computes the PID for a particular node
* @returns the PID as a string
*/
std::string Statistics::getPid(std::string nodeName)
{
  // Convert string to const char * as system requires 
  // parameter of type const char * - for system()
  std::string data;
  int max_buffer = 256;
  char buffer[max_buffer]; 
  std::string str = "rosnode info " + nodeName +" 2>/dev/null | grep Pid| cut -d' ' -f2";
  //const char *command = str.c_str(); 
  //system(command); 
  

  /*The system command is often run first, before any output commands and the function 
  returns an integer indicating success or failure, but not the output of the string*/

  /*Opens up a read-only stream, runs the command and captures the output,
   stuffs it into the buffer and returns it as a string.*/
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

    //return data;

    /*To avoid the new line character by the end of data*/
    return data.substr(0, data.length() - 1);
   }	
}


/**
* @brief Computes the up time for the system
* @returns the uptime in seconds
*/
long Statistics::UpTime() 
{
  std::string line;
  long upTime, idleTime;
  std::ifstream inFile(kProcDirectory + kUptimeFilename);
  if (inFile.is_open()) 
  {
    std::getline(inFile, line);
    std::istringstream linestream(line);
    linestream >> upTime >> idleTime;
    
  }
  return upTime;
}

/**
* @brief Computes the up time for a particular PID
* @returns the uptime in seconds
*/
long Statistics::UpTime(std::string pid)
{
  std::string line;
  std::string value;
  long seconds;
  std::vector<std::string> input;
  std::ifstream filestream(kProcDirectory + pid + kStatFilename);
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
* @brief Computes the total time spend since the process started
* @returns the time in seconds
*/
long Statistics::ActiveJiffies(std::string pid) 
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
* @brief Computes the CPU usage for a particular PID
* @returns the cpu usage in %
* @ref https://stackoverflow.com/questions/16726779/how-do-i-get-the-total-cpu-usage-of-an-application-from-proc-pid-stat/16736599#16736599
*/
double Statistics::computeCPUUsage(std::string pid)
{
    double active_jiffies = (float)(ActiveJiffies(pid));
    m_upTime       = (float)(UpTime(pid));
    double cpu_Uti = 100*((active_jiffies/sysconf(_SC_CLK_TCK))/m_upTime);
    return cpu_Uti;
}

/**
* @brief Computes the RAM usage for a particular PID
* @returns the RAM usage in Mb%
*/
double Statistics::computeMemoryUsage(std::string pid)
{
  std::string line,key,unit;
  long value;
  std::ifstream inFile(kProcDirectory + "/" + pid + kStatusFilename);
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






std::string Statistics::ElapsedTime(long elapsedSeconds) 
{
  std::string Time = "";
  int hours, minutes, seconds, time;
  std::string stringHours, stringMinutes, stringSeconds;
  hours = elapsedSeconds / 3600;
  time = elapsedSeconds % 3600;
  minutes = time / 60;
  time = time % 60;
  seconds = time;

  if (hours < 10) 
  {
    stringHours = "0" + std::to_string(hours);
  } 
  else 
  {
    stringHours = std::to_string(hours);
  }

  if (minutes < 10) 
  {
    stringMinutes = "0" + std::to_string(minutes);
  } 
  else 
  {
    stringMinutes = std::to_string(minutes);
  }

  if (seconds < 10) 
  {
    stringSeconds = "0" + std::to_string(seconds);
  } 
  else 
  {
    stringSeconds = std::to_string(seconds);
  }

  Time = stringHours + ":" + stringMinutes + ":" + stringSeconds;
  return Time;
}