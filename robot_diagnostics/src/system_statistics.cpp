#include <ros/console.h> 
#include "system_statistics.h"

/**
* @brief  Constructor for the Statistics
*/
SystemStatistics::SystemStatistics():nh("~")
{
   ROS_INFO("Statistics constructor called");
   nh.getParam("/maxPermissibleCpuUsage", m_cpuThreshold);
   nh.getParam("/maxPermissibleMemoryUsage", m_memoryThreshold);
   nh.getParam("/maxPermissibleTemperature", m_temperatureThreshold);
   nh.getParam("/maxPermissibleAverageCpuUsage", m_averageLoadThreshold);
  
  m_numberOfCores = getNumberOfCores();
  m_monitor = std::make_shared<Monitor>(nh, "System Monitor", true);

}

/**
* @brief  Destructor for the Statistics
*/

SystemStatistics::~SystemStatistics()
{

}



/**
* @brief Computes and updates the system related statistics
*/
void SystemStatistics::computeAndUpdateSystemStatistics()
{
  computeSystemStatistics();
  publishSystemStatistics();
}


/**
* @brief Computes the system related statistics
*/
void SystemStatistics::computeSystemStatistics()
{
  m_cpuPercentage    = computeCpuUtilization();
  m_memoryPercentage = computeMemoryUtilization();
  
  for(int i =0; i<m_numberOfCores;i++)
  {
    m_coreTemperature[i] = getCoreTemperature(i);
  }
  computeAverageCPULoad();
}

/**
* @brief Publishes the system related statistics
*/
void SystemStatistics::publishSystemStatistics()
{
    if(m_cpuPercentage > m_cpuThreshold)
    {
      publishCpuStatistics();
    }

    if(m_memoryPercentage > m_memoryThreshold)
    {
      publishMemoryStatistics();
    }
    if((m_averageLoad[0] > m_averageLoadThreshold) || (m_averageLoad[1] > m_averageLoadThreshold) || (m_averageLoad[2] > m_averageLoadThreshold) )
    {
      publishAverageLoadStatistics();
    }

    for(int i =0; i<m_numberOfCores;i++)
    {
      if(m_coreTemperature[i] > m_temperatureThreshold)
      {
        publishTemperatureStatistics(i,m_coreTemperature[i]);
      }
    }    

}

/**
* @brief Computes the memory utilization of the system in percentage
*/
double SystemStatistics::computeMemoryUtilization() 
{
  std::string line, key;
  long int value, MemTotal, MemFree,Buffers;
  double memoryUtilization = 0.0;
  std::ifstream inFile(Utilities::kProcDirectory + Utilities::kMeminfoFilename);
  if (inFile.is_open()) {
    while (std::getline(inFile, line)) 
    {
      std::replace(line.begin(), line.end(), ':', ' ');
      std::istringstream linestream(line);
      linestream >> key >> value;

      if (key == "MemTotal") 
      {
        MemTotal = static_cast<float>(value);
      }
      if (key == "MemFree")
      {
        MemFree = static_cast<double>(value);

      }
      if (key == "Buffers")
      {
        Buffers = static_cast<double>(value);
        //CPU = (float)(MemTotal - MemFree) / MemTotal;
        memoryUtilization =   100.0*(1.0 -(double(MemFree) / double(MemTotal - Buffers))) ;
        //CPU    = double(5)/double(6);
	//std::cout << "Buffer :  "<< Buffers  << " MemFree  " << MemFree <<"CPU  "<< CPU <<std::endl;
        return memoryUtilization;
      }     

    }
  }
  return -1.0;
}


/**
* @brief Computes the Cpu utilization of the system in percentage
*/
double SystemStatistics::computeCpuUtilization()
{   
  double m_Idle,m_NonIdle,m_PrevTotal,m_Total,m_Totald,m_Idled,m_CpuPercentage,m_PrevIdle,m_PrevNonIdle;
  m_cpuData = getCpuData();
  m_Idle        = (double)IdleJiffies();
  m_NonIdle     = (double)ActiveJiffies();
  m_PrevTotal   = m_PrevIdle + m_PrevNonIdle;
  m_Total       = m_Idle + m_NonIdle;

  m_Totald      =  m_Total - m_PrevTotal;
  m_Idled       = m_Idle - m_PrevIdle;
  m_CpuPercentage = (m_Totald - m_Idled)/m_Totald; 
  
  m_PrevIdle    = m_Idle;
  m_PrevNonIdle = m_NonIdle;
  
  return m_CpuPercentage*100.0;

}



/**
* @brief Reads and returns the CPU data
* @return the CPU data as a vector of string
*/
std::vector<std::string> SystemStatistics::getCpuData() 
{
  std::string line;
  std::string value;
  std::vector<std::string> dataSet;

  std::ifstream inFile(Utilities::kProcDirectory + Utilities::kStatFilename);
  if (inFile.is_open()) 
  {
    /*Accessing single line only and iterating to the  end of that first line with key 'cpu'*/
    std::getline(inFile, line);/*Can USE while(inFile) instead of with !inFile.eof()*/
    std::istringstream linestream(line);
    while (linestream >> value)   
    {
      if(value == "cpu")
        continue;
      else
      {
        dataSet.push_back(value);
      
      }
    }
  }
  return dataSet;
}


/**
* @brief Read the number of active jiffies for the system
* @return the number of active jiffies
*/
long SystemStatistics::ActiveJiffies() 
{ 
  return (std::stol(m_cpuData[Utilities::CPUStates::kUser_])) + (std::stol(m_cpuData[Utilities::CPUStates::kNice_])) + (std::stol(m_cpuData[Utilities::CPUStates::kSystem_])) + (std::stol(m_cpuData[Utilities::CPUStates::kIRQ_])) + (std::stol(m_cpuData[Utilities::CPUStates::kSoftIRQ_])) + (std::stol(m_cpuData[Utilities::CPUStates::kSteal_]));
}

/**
* @brief Read the number of idle jiffies for the system
* @return the number of idle jiffies
*/
long SystemStatistics::IdleJiffies() 
{ 
  return (std::stol(m_cpuData[Utilities::CPUStates::kIdle_]) + (stol(m_cpuData[Utilities::CPUStates::kIOwait_])));
}

/**
* @brief Computes the number of cores of PC
* @returns the number of cores
*/
int SystemStatistics::getNumberOfCores()
{
  std::string data;
  int max_buffer = 256;
  char buffer[max_buffer]; 
  std::string str = "nproc";

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
    return std::stoi(data);
   }
   return -1;	
}


/**
* @brief Computes the core temperature
* @returns the temperature for the corresponding core
*/
int SystemStatistics::getCoreTemperature(int core)
{
  std::string data;
  int max_buffer = 256;
  char buffer[max_buffer]; 
  std::string str = "cat /sys/class/thermal/thermal_zone"+std::to_string(core)+"/temp";

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
          //std::cout << data << std::endl;
        	data.append(buffer);
        }  
        
    }
    
    pclose(stream);
    return std::stoi(data);
    //return data;

    /*To avoid the new line character by the end of data*/
    //return data.substr(0, data.length() - 1);
   }
   return -1;	
}

/**
* @brief Computes the average load of PC in last 1,5,15 min
*/
void SystemStatistics::computeAverageCPULoad()
{
  std::string data;
  int max_buffer = 256;
  char buffer[max_buffer]; 
  std::string str = "uptime";


  /*The system command is often run first, before any output commands and the function 
  returns an integer indicating success or failure, but not the output of the string*/

  /*Opens up a read-only stream, runs the command and captures the output,
   stuffs it into the buffer and returns it as a string.*/

  std::unique_lock<std::mutex> lock(m_mutex);

  FILE *stream = popen(str.c_str(), "r");
  if (stream) 
  {
    while (!feof(stream))
    {
        if (fgets(buffer, max_buffer, stream) != NULL) 
        {
          
        	data.append(buffer);
        } 
        std::string str1,str2;

        std::string mainString = "load average: ";
        /*Finding the position of the mainString in the data string*/
        int stringPosition = data.find(mainString); 
        //std::cout << "Found at "<< stringPosition <<std::endl;
        std::string subString = data.substr(stringPosition);

        std::replace(subString.begin(),subString.end(),':',' ');
        std::replace(subString.begin(),subString.end(),',',' ');
        std::istringstream linestream(subString);
        linestream >> str1 >> str2 >> m_averageLoad[0] >> m_averageLoad[1] >> m_averageLoad[2];
        //std::cout << "Read data  " << m_averageLoad[0] << "  "<<m_averageLoad[1]<< "  "<<m_averageLoad[2] << std::endl;
    }
    
    pclose(stream);

   }
}


/**
* @brief Publishes the memory statistics of the system
*/
void SystemStatistics::publishMemoryStatistics()
{
   std::string key = "/totalMemoryUsage";
   std::string value = std::to_string(m_memoryPercentage);
   double error_level = 0.6; 
   m_monitor->addValue(key, value, "%", error_level, AggregationStrategies::FIRST);
}

/**
* @brief Publishes the CPU statistics of the system
*/
void SystemStatistics::publishCpuStatistics()
{
   std::string key = "/totalCpuUsage";
   std::string value = std::to_string(m_cpuPercentage);
   double error_level = 0.6; 
   m_monitor->addValue(key, value, "%", error_level, AggregationStrategies::FIRST);
}

/**
* @brief Publishes the core temperature statistics of the system
*/
void SystemStatistics::publishTemperatureStatistics(int core ,double temperature)
{
   std::string key = "/systemTemperature";
   std::string value ="Temperature of core-"+ std::to_string(core) + " : "+ std::to_string(temperature);
   double error_level = 0.3; 
   m_monitor->addValue(key, value, "", error_level, AggregationStrategies::FIRST);
}


/**
* @brief Publishes the average load statistics of the system
*/
void SystemStatistics::publishAverageLoadStatistics()
{
   std::string key = "/averageLoad";
   std::string value = "Average load in last 1 min: "+ std::to_string(m_averageLoad[0]) + " last 10 min : "+ std::to_string(m_averageLoad[1]) + " last 15 min : " + std::to_string(m_averageLoad[2]);
   double error_level = 0.3; 
   m_monitor->addValue(key, value, "", error_level, AggregationStrategies::FIRST);
}