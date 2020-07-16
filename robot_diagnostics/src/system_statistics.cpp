#include <ros/console.h> 
#include "system_statistics.h"

/**
* @brief  Constructor for the Statistics
*/
SystemStatistics::SystemStatistics()
{
   ROS_INFO("Statistics constructor called");
   //nh.getParam("/nodes", m_initialNodeList);

   /*Gets all the nodes registered in the ROS master and stores it in a vector m_nodeListOriginal*/
   //ros::master::getNodes(m_nodeListOriginal);

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
    
   // std::cout <<"Memory utilization in % " << MemoryUtilization() << std::endl;
   // std::cout << "CPU utilization in % "   << Utilization() <<std::endl;
    int numberOfCores = getNumberOfCores();
  //  std::cout << "Number of cores : " <<numberOfCores << std::endl;
    for(int i =0; i<numberOfCores;i++)
    {
      int coreTemperature = getCoreTemperature(i);
     // std::cout << "Core temperature of "<< i << " th core is  "<< coreTemperature << std::endl;
    }
  //  std::cout <<"\n";
    getAverageCPULoad();

}



double SystemStatistics::MemoryUtilization() 
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



// TODO: Read and return CPU utilization
std::vector<std::string> SystemStatistics::CpuUtilization() 
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
  std::vector<std::string> cpuData = CpuUtilization();
  return (std::stol(cpuData[CPUStates::kUser_])) + (std::stol(cpuData[CPUStates::kNice_])) + (std::stol(cpuData[CPUStates::kSystem_])) + (std::stol(cpuData[CPUStates::kIRQ_])) + (std::stol(cpuData[CPUStates::kSoftIRQ_])) + (std::stol(cpuData[CPUStates::kSteal_]));
}

// TODO: Read and return the number of idle jiffies for the system
long SystemStatistics::IdleJiffies() 
{ 
  std::vector<std::string> cpuData = CpuUtilization();
  return (std::stol(cpuData[CPUStates::kIdle_]) + (stol(cpuData[CPUStates::kIOwait_])));
}

double SystemStatistics::Utilization()
{   

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
    // Convert string to const char * as system requires 
  // parameter of type const char * - for system()
  std::string data;
  int max_buffer = 256;
  char buffer[max_buffer]; 
  std::string str = "uptime";
  //const char *command = str.c_str(); 
  //system(command); 
  

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
        std::cout << "Read data  " << m_averageLoad[0] << "  "<<m_averageLoad[1]<< "  "<<m_averageLoad[2] << std::endl;
    }
    
    pclose(stream);

   }
}