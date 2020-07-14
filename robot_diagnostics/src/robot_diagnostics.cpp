#include "ros/ros.h"
#include <vector>
#include <fstream>
const std::string kProcDirectory{"/proc/"};
const std::string kStatusFilename{"/status"};
const std::string kUptimeFilename{"/uptime"};
const std::string kStatFilename{"/stat"};
using namespace std;

// TODO: Read and return the system uptime
long UpTime() 
{
  string line;
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



long UpTime(std::string pid)
{
  string line;
  string value;
  long seconds;
  vector<string> input;
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
  seconds  = std::stol(input[21]);
  seconds = seconds/sysconf(_SC_CLK_TCK);
  return UpTime() - seconds;

}


// TODO: Read andreturn the number of active jiffies for a PID
// REMOVE: [[maybe_unused]] once you define the function
long ActiveJiffies(std::string pid) 
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

double computeCPUUsage(std::string pid)
{
    double active_jiffies = (float)(ActiveJiffies(pid));
    double uptime = (float)(UpTime(pid));
    double cpu_Uti = ((active_jiffies/sysconf(_SC_CLK_TCK))/uptime);
    return cpu_Uti;
}
void computeMemoryUsage(std::string pid)
{
  std::string line,key,unit;
  long value;
  std::ifstream inFile(kProcDirectory + "/" + pid + kStatusFilename);
  if(inFile.is_open())
  {
  	while(getline(inFile,line))
  	{
  		std::replace(line.begin(),line.end(),':',' ');
  		std::istringstream linestream(line);
  		linestream>>key>>value>>unit;
  		if(key == "VmSize")
  		{
  			double convertedValue = value * 0.001;
  			std::cout << "Ram Usage of "<< pid << " : " << convertedValue << std::endl;
  		}
  	}
  }
}


std::string getPid(std::string nodeName)
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

    //std::cout << "Data " << data << "  Modified data : " << data.substr(0, data.length() - 1)  << "datalength " <<data.length()<< std::endl;
    //return data;

    /*To avoid the new line character by the end of data*/
    return data.substr(0, data.length() - 1);
   }	
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{

  ros::init(argc, argv, "diagnostics");


  ros::NodeHandle n;
  std::vector<std::string> initialNodeList;

  n.getParam("/nodes", initialNodeList);
  ROS_ERROR("Initial list size : %d",initialNodeList.size());
  //ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(1);
  std::vector<std::string> nodeListOriginal;
  ros::master::getNodes(nodeListOriginal);
  std::vector < std::string > nodeListCopy;
  //std::cout << ros::this_node::getName() << std:endl;
  

  while (ros::ok())
  {

  	nodeListCopy = nodeListOriginal;
    //std::vector<std::string> node_list_ = nodeList;
    //ROS_WARN("Node list size original : %d  copy : %d  inital  %d",nodeListOriginal.size(),nodeListCopy.size(),initialNodeList.size());

    for (std::vector<std::string>::iterator it(initialNodeList.begin()); it != initialNodeList.end(); ++it)
    {
      nodeListCopy.erase(std::remove(begin(nodeListCopy), end(nodeListCopy), *it), end(nodeListCopy));
    }

    //initialNodeList = nodeListOriginal;
    //ROS_INFO("Node list size original : %d  copy updated: %d",nodeList.size(),nodeListOriginal.size());
    
    for(std::vector<std::string>::iterator i(nodeListCopy.begin());i != nodeListCopy.end();++i)
    {
      //std::cout << *i << std::endl;
      std::string resultPid = getPid(*i);
      std::cout << *i << " 's PID is : " << resultPid << std::endl;
      computeMemoryUsage(resultPid); 
      long uptime = UpTime(resultPid);
      double cpuUsage = computeCPUUsage(resultPid);
      std::cout << *i << " 's PID uptime is : " << uptime << " : " << cpuUsage << std::endl;

    }

    //std::vector < std::string > nodeListOriginal(nodeList);
    loop_rate.sleep();
  }

}
