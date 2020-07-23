#include <ros/console.h> 
#include "system_statistics.h"

/**
* @brief  Constructor for the Statistics
*/
SystemStatistics::SystemStatistics()
{
   ROS_INFO("Statistics constructor called");
   nh.getParam("/cpu_threshold", m_cpuThreshold);
   nh.getParam("/memory_threshold", m_memoryThreshold);
   nh.getParam("/temperature_threshold", m_temperatureThreshold);
   nh.getParam("/average_cpu_load_threshold", m_averageLoadThreshold);
   /*Gets all the nodes registered in the ROS master and stores it in a vector m_nodeListOriginal*/
   //ros::master::getNodes(m_nodeListOriginal);
  m_monitor = new Monitor(nh, "System Statistic Monitor", true);
   //ROS_ERROR("Initial list size : %d",m_initialNodeList.size());
}

/**
* @brief  Destructor for the Statistics
*/

SystemStatistics::~SystemStatistics()
{

}



/**
* @brief Updates the node related statistics
*/
void SystemStatistics::updateSystemStatistics()
{
    m_cpuPercentage    = computeCpuUtilization();
    m_memoryPercentage = computeMemoryUtilization();
    int numberOfCores = getNumberOfCores();
    double coreTemperature[numberOfCores] ={0.0};
    //std::cout <<"Memory utilization in % " << m_memoryPercentage << std::endl;
    //std::cout << "CPU utilization in % "   << m_cpuPercentage<<std::endl;
    
  //  std::cout << "Number of cores : " <<numberOfCores << std::endl;
    for(int i =0; i<numberOfCores;i++)
    {
      coreTemperature[i] = getCoreTemperature(i);
      if(coreTemperature[i] > m_temperatureThreshold)
      {
        updateTemperatureStatus(i,coreTemperature[i]);
      }
      //std::cout << "Core temperature of "<< i << " th core is  "<< coreTemperature << std::endl;
    }
    getAverageCPULoad();

    if(m_cpuPercentage > m_cpuThreshold)
    {
      updateCpuStatus();
    }

    if(m_memoryPercentage > m_memoryThreshold)
    {
      updateMemoryStatus();
    }
    if((m_averageLoad[0] > m_averageLoadThreshold) || (m_averageLoad[1] > m_averageLoadThreshold) || (m_averageLoad[2] > m_averageLoadThreshold) )
    {
      updateAverageLoadStatus();
    }

}



double SystemStatistics::computeMemoryUtilization() 
{
  std::string line, key;
  long int value, MemTotal, MemFree,Buffers;
  double memoryUtilization = 0.0;
  std::ifstream inFile(kProcDirectory + kMeminfoFilename);
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



double SystemStatistics::computeCpuUtilization()
{   
  m_cpuData = getCpuData();
  double m_Idle,m_NonIdle,m_PrevTotal,m_Total,m_Totald,m_Idled,m_CpuPercentage,m_PrevIdle,m_PrevNonIdle;
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



// TODO: Read and return CPU utilization
std::vector<std::string> SystemStatistics::getCpuData() 
{
  std::string line;
  std::string value;
  std::vector<std::string> dataSet;

  std::ifstream inFile(kProcDirectory + kStatFilename);
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
        //std::cout <<value << std::endl;
      }
    }
  }
  return dataSet;
}



// TODO: Read and return the number of active jiffies for the system
long SystemStatistics::ActiveJiffies() 
{ 
  return (std::stol(m_cpuData[CPUStates::kUser_])) + (std::stol(m_cpuData[CPUStates::kNice_])) + (std::stol(m_cpuData[CPUStates::kSystem_])) + (std::stol(m_cpuData[CPUStates::kIRQ_])) + (std::stol(m_cpuData[CPUStates::kSoftIRQ_])) + (std::stol(m_cpuData[CPUStates::kSteal_]));
}

// TODO: Read and return the number of idle jiffies for the system
long SystemStatistics::IdleJiffies() 
{ 
  return (std::stol(m_cpuData[CPUStates::kIdle_]) + (stol(m_cpuData[CPUStates::kIOwait_])));
}

/**
* @brief Computes the PID for a particular node
* @returns the PID as a string
*/
int SystemStatistics::getNumberOfCores()
{
  // Convert string to const char * as system requires 
  // parameter of type const char * - for system()
  std::string data;
  int max_buffer = 256;
  char buffer[max_buffer]; 
  std::string str = "nproc";
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
* @brief Computes the PID for a particular node
* @returns the PID as a string
*/
int SystemStatistics::getCoreTemperature(int core)
{
  // Convert string to const char * as system requires 
  // parameter of type const char * - for system()
  std::string data;
  int max_buffer = 256;
  char buffer[max_buffer]; 
  std::string str = "cat /sys/class/thermal/thermal_zone"+std::to_string(core)+"/temp";
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


void SystemStatistics::getAverageCPULoad()
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



void SystemStatistics::updateMemoryStatus()
{
   std::string key = "/totalMemoryUsage";
   std::string value = std::to_string(m_memoryPercentage);
   double error_level = 0.6; 
   m_monitor->addValue(key, value, "%", error_level, AggregationStrategies::FIRST);
}

void SystemStatistics::updateCpuStatus()
{
   std::string key = "/totalCpuUsage";
   std::string value = std::to_string(m_cpuPercentage);
   double error_level = 0.6; 
   m_monitor->addValue(key, value, "%", error_level, AggregationStrategies::FIRST);
}

void SystemStatistics::updateTemperatureStatus(int core ,double temperature)
{
   std::string key = "/systemTemperature";
   std::string value ="Temperature of core-"+ std::to_string(core) + " : "+ std::to_string(temperature);
   double error_level = 0.3; 
   m_monitor->addValue(key, value, "", error_level, AggregationStrategies::FIRST);
}

void SystemStatistics::updateAverageLoadStatus()
{
   std::string key = "/averageLoad";
   std::string value = "Average load in last 1 min: "+ std::to_string(m_averageLoad[0]) + " last 10 min : "+ std::to_string(m_averageLoad[1]) + " last 15 min : " + std::to_string(m_averageLoad[2]);
   double error_level = 0.3; 
   m_monitor->addValue(key, value, "", error_level, AggregationStrategies::FIRST);
}