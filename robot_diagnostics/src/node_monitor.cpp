#include <ros/console.h> 
#include "node_monitor.h"

/**
* @brief  Constructor for the Statistics
*/
NodeMonitor::NodeMonitor():nh("~") 
{
  
   ROS_INFO("Statistics constructor called");
   nh.getParam("/nodes", m_initialNodeList);
   nh.getParam("/nodeFilterType", m_nodeFilterType);

   /*Gets all the nodes registered in the ROS master and stores it in a vector m_nodeListOriginal*/
   ros::master::getNodes(m_nodeListOriginal);
  
   /*Verifying that the mentioned nodes in the yaml file are valid*/
   updateValidNodeList(m_validNodeList);



   applyNodeFilter(m_nodeListFiltered);

   for(int i=0; i< m_nodeListFiltered.size();i++)
   {
       std::cout << "Filtered nodes are " <<m_nodeListFiltered[i] << std::endl;
       std::shared_ptr<NodeStatistics>nodeStatistics(new NodeStatistics(nh,m_nodeListFiltered[i]));
       m_nodeList.push_back(nodeStatistics);
   }




}

/**
* @brief  Destructor for the Statistics
*/

NodeMonitor::~NodeMonitor()
{
    delete m_monitor;
}




void NodeMonitor::updateValidNodeList(std::vector<std::string> &validNodeList)
{
    validNodeList.clear();
    validNodeList.resize(0);
    for(int i=0;i< m_initialNodeList.size();i++)
    {
        if(isValidNode(m_initialNodeList[i]))
        {
            validNodeList.push_back(m_initialNodeList[i]);
        }
    }
}


/**
* @brief checks whehter the given node name is currently registered with ros master
* @returns true if the node is available,else false;
*/
bool NodeMonitor::isValidNode(std::string &node_name)
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


/**
* @brief Applies the node filters on the nodes mentioned 
*/
void NodeMonitor::applyNodeFilter(std::vector<std::string> &nodeListFiltered)
{
   std::vector<std::string> nodeListCopy;
   nodeListCopy.clear();
   nodeListCopy.resize(0);
   nodeListCopy = m_nodeListOriginal;

  if(m_nodeFilterType == NodeFilter::DEFAULT)
  {
    ROS_WARN("Default filter type");
    nodeListCopy = m_nodeListOriginal;
  }
  else if (m_nodeFilterType == NodeFilter::ADD)
  {
    ROS_WARN("ADD filter type");
    nodeListCopy = m_validNodeList;
  }
  else
  {
    ROS_WARN("Remove filter type");
    for (std::vector<std::string>::iterator it(m_validNodeList.begin()); it != m_validNodeList.end(); ++it)
    {
      nodeListCopy.erase(std::remove(begin(nodeListCopy), end(nodeListCopy), *it), end(nodeListCopy));
    }
  }
  nodeListFiltered = nodeListCopy;
}

/**
* @brief Updates the node related statistics
*/
void NodeMonitor::updateNodeStatistics()
{
    // ROS_WARN("Loop called");
    // m_nodeListCopy.clear();
    // m_nodeListCopy.resize(0);
    // //ros::master::getNodes(m_nodeListOriginal);
  	// m_nodeListCopy = m_nodeListOriginal;
    // //ROS_WARN("Node list size original : %d  copy : %d  inital  %d",nodeListOriginal.size(),nodeListCopy.size(),initialNodeList.size());

    // for (std::vector<std::string>::iterator it(m_initialNodeList.begin()); it != m_initialNodeList.end(); ++it)
    // {
    //   m_nodeListCopy.erase(std::remove(begin(m_nodeListCopy), end(m_nodeListCopy), *it), end(m_nodeListCopy));
    // }

    
    // for(std::vector<std::string>::iterator i(m_nodeListCopy.begin());i != m_nodeListCopy.end();++i)
    // {

    //   bool isAvailable = isValidNode(*i);
    //   if(isAvailable)
    //   {
    //     //std::cout << *i << std::endl;
    //     std::string currentPid = getPid(*i);
      
    //     std::cout << *i << " 's PID is : " << currentPid << std::endl;

    //     //std::cout << *i << " 's PID  "<< currentPid << " uptime is : " << uptime << " : " << "RAM usage in MB : " << memUsage << " CPU Usage: "<< cpuUsage << std::endl;
    //     //std::string ping       = pingPid(*i);
    //     if(m_setup == false)
    //     {
    //       m_nodeLog[currentPid] = *i;
    //     }
    //     else
    //     {
    //       if(m_nodeLog.find(currentPid) == m_nodeLog.end())
    //       {
    //         ROS_ERROR("Node deleted %s",(*i).c_str());
    //       }
    //       else
    //       {

    //         m_memPercentage     = computeNodeMemoryPercentage(currentPid)*100/(m_ramSize/1024); 
    //         //m_upTime       = UpTime(currentPid);
    //         m_cpuPercentage     = computeNodeCPUPercentage(currentPid);
    //       }

    //     }
    //     updateCpuStatus(*i);
    //     updateMemoryStatus(*i);
    //     updateTimeStatus(*i);
    //     updateNodeStatus(*i);
    //     updateNodePingStatus(*i);
    //   }
    //   else
    //   {
    //      ROS_ERROR("Node not available is %s",(*i).c_str());
    //   }
    //   // try
    //   // {
        
    //   // }
    //   // catch(const std::exception& e)
    //   // {
    //   //     std::cerr << e.what() << '\n';
    //   // }
    // }
    
    // m_setup = true;


}







// void NodeMonitor::updateCpuStatus(std::string &node_name)
// {
//     std::string key = node_name + "/cpu_usage";
//     m_monitor->addValue(key, m_cpuPercentage, "%", 0.0, AggregationStrategies::FIRST);
// }


// void NodeMonitor::updateMemoryStatus(std::string &node_name)
// {
//     std::string key = node_name + "/memory_usage";    
//     m_monitor->addValue(key, m_memPercentage, "%", 0.0, AggregationStrategies::FIRST);
// }

// /**
// * @brief Computes the total time spend since the node is started
// */
// void NodeMonitor::updateTimeStatus(std::string &node_name)
// {
//     std::string key = node_name + "/cpu_time";
//     std::string time = "Time from start for "+  node_name + " : ";
//     //time = time + ElapsedTime(m_upTime);
//     //std::cout << time << std::endl;
//     m_monitor->addValue(key, float(m_upTime), "seconds", 0.0, AggregationStrategies::FIRST);
// }

// /**
// * @brief Checks the node status
// */
// void NodeMonitor::updateNodeStatus(std::string &node_name)
// {
//    std::string key = node_name;
//    std::string value ="";
//    double error_level = 0.0; 
//    getErrorValueFromState(key,value,error_level);
//    m_monitor->addValue(key, value, "", error_level, AggregationStrategies::FIRST);
// }



// /**
// * @brief Checks whether a node is alive or not by pinging it
// */
// void NodeMonitor::updateNodePingStatus(std::string &node_name)
// {
//   std::string nodeXmlrpcURI = getNodeXmlrpcURI(node_name);
//   std::string data;
//   int max_buffer = 256;
//   char buffer[max_buffer]; 
//   std::string str = "rosnode ping -a | grep " + nodeXmlrpcURI;
// /*Opens up a read-only stream, runs the command and captures the output,
//    stuffs it into the buffer and returns it as a string.*/
//   std::unique_lock<std::mutex> lock (m_mutex);
//   FILE *stream = popen(str.c_str(), "r");
//   if (stream) 
//   {
//     while (!feof(stream))
//     {
//         if (fgets(buffer, max_buffer, stream) != NULL) 
//         {
//         	data.append(buffer);
//         }  
        
//     }
    
//     pclose(stream);

//     //return data;

//     std::istringstream linestream(data);
//     std::string l1,l2,l3,l4,l5;

//     linestream>>l1>>l2>>l3>>l4>>l5;
//     std::string key = node_name+"/ping_rate";
//     float value =std::stof(l5.substr(5,5));
//     double error_level = 0.0; 
//     m_monitor->addValue(key, value, "ms", error_level, AggregationStrategies::FIRST);
//    }	  
// }

// /**
// * @brief Computes status of node whether it's alive,dead,sleeping etc
// */
// void NodeMonitor::getErrorValueFromState(std::string &node_name,std::string &value, double &error_level)
// {
//     /* Providing the desciption of the state based on state context */
//     switch (m_nodeState)
//     {
//     case 'R':
//         value = node_name + " is running";
//         error_level = 0.0;
//         break;
//     case 'S':
//         value = node_name + " is sleeping in an interruptible wait";
//         error_level = 0.1;
//         break;
//     case 'D':
//         value = node_name + " is waiting in uninterruptible disk sleep";
//         error_level = 0.1;
//         break;
//     case 'T':
//         value = node_name + " is stopped (on a signal) or trace stopped";
//         error_level = 0.1;
//         break;
//     case 't':
//         value = node_name + " is trace stopped";
//         error_level = 0.1;
//         break;
//     case 'Z':
//         value = node_name + " is a zombie";
//         error_level = 0.3;
//         break;
//     case 'W':
//         value = node_name + " is paging";
//         error_level = 0.1;
//         break;
//     case 'X':
//         value = node_name + " is Dead";
//         error_level = 0.3;
//         break;
//     case 'x':
//         value = node_name + " is Dead";
//         error_level = 0.3;
//         break;
//     case 'K':
//         value = node_name + " is wakekill";
//         error_level = 0.3;
//         break;
//     case 'P':
//         value = node_name + " is parked";
//         error_level = 0.1;
//         break;
//     default:
//         value = node_name + " is a zombie";
//         error_level = 0.3;
//     }
// }



// /**
// * @returns the nodexmlrpcURI corresponding to the node provided
// */
// std::string NodeMonitor::getNodeXmlrpcURI(std::string &node_name)
// {
//   std::string data;
//   int max_buffer = 256;
//   char buffer[max_buffer]; 
//   std::string str = "rosnode list -a | grep " + node_name;
// /*Opens up a read-only stream, runs the command and captures the output,
//    stuffs it into the buffer and returns it as a string.*/
//   std::unique_lock<std::mutex> lock (m_mutex);
//   FILE *stream = popen(str.c_str(), "r");
//   if (stream) 
//   {
//     while (!feof(stream))
//     {
//         if (fgets(buffer, max_buffer, stream) != NULL) 
//         {
//         	data.append(buffer);
//         }  
        
//     }
    
//     pclose(stream);

//     //return data;

//     std::istringstream linestream(data);
//     std::string l1,l2;
//     linestream>>l1>>l2;
//     //    std::cout << "Returned data  "<< l1 << std::endl;
//     /*To avoid the new line character by the end of data*/
//     return l1;
//    }	
// }









// /**
// * @brief Computes the total time spend since the process started
// * @returns the time in seconds
// */
// long NodeMonitor::ActiveJiffies(std::string pid) 
// { 
//   long activeJiffies;
//   std::string line;
//   std::string value;
//   std::vector<std::string> dataSet;


//   std::ifstream inFile(kProcDirectory + "/" + pid +kStatFilename);
//   if (inFile.is_open()) 
//   {
//     std::getline(inFile, line);/*Can USE while(inFile) instead of with !inFile.eof()*/
//     std::istringstream linestream(line);
//     while (linestream >> value) 
//     {
//       dataSet.push_back(value);
//     }
//   }
//   activeJiffies  = std::stol(dataSet[13]) + std::stol(dataSet[14]) + std::stol(dataSet[15]) + stol(dataSet[16]);
//   return activeJiffies; 
// }

// /**
// * @brief Computes the CPU usage for a particular PID
// * @returns the cpu usage in %
// * @ref https://stackoverflow.com/questions/16726779/how-do-i-get-the-total-cpu-usage-of-an-application-from-proc-pid-stat/16736599#16736599
// */
// double NodeMonitor::computeNodeCPUPercentage(std::string pid)
// {
//     double active_jiffies = (float)(ActiveJiffies(pid));
//     m_upTime       = (float)(UpTime(pid));
//     double cpu_Uti = 100*((active_jiffies/sysconf(_SC_CLK_TCK))/m_upTime);
//     return cpu_Uti;
// }

// /**
// * @brief Computes the RAM usage for a particular PID
// * @returns the RAM usage in Mb
// */
// double NodeMonitor::computeNodeMemoryPercentage(std::string pid)
// {
//   std::string line,key,unit;
//   long value;
//   std::ifstream inFile(kProcDirectory + "/" + pid + kStatusFilename);
// //  std::unique_lock<std::mutex> lock (m_mutex);
//   if(inFile.is_open())
//   {
//   	while(std::getline(inFile,line))
//   	{
//   		std::replace(line.begin(),line.end(),':',' ');
//   		std::istringstream linestream(line);
//   		linestream>>key>>value>>unit;
//   		if(key == "VmSize")
//   		{
//   			double convertedValue = value * 0.001;
//   			//std::cout << "Ram Usage of "<< pid << " : " << convertedValue << std::endl;
//             return convertedValue;
//         }
//   	}
//   }
//   return -1.0;
// }


// /**
// * @brief Computes the up time for the system
// * @returns the uptime in seconds
// */
// long NodeMonitor::UpTime() 
// {
//   std::string line;
//   long upTime, idleTime;
//   std::ifstream inFile(kProcDirectory + kUptimeFilename);
// //  std::unique_lock<std::mutex> lock (m_mutex);
//   if (inFile.is_open()) 
//   {
//     std::getline(inFile, line);
//     std::istringstream linestream(line);
//     linestream >> upTime >> idleTime;
    
//   }
//   return upTime;
// }

// /**
// * @brief Computes the up time for a particular PID
// * @returns the uptime in seconds
// */
// long NodeMonitor::UpTime(std::string pid)
// {
//   std::string line;
//   std::string value;
//   long seconds;
//   std::vector<std::string> input;
//   std::ifstream filestream(kProcDirectory + pid + kStatFilename);
// //  std::unique_lock<std::mutex> lock (m_mutex);
//   if (filestream.is_open()) 
//   {
//     std::getline(filestream, line);
//     std::istringstream linestream(line);
//       while (linestream >> value) 
//       {
//         input.push_back(value);
//       }
//   }
//   seconds     = std::stol(input[21]);
//   m_nodeState = input[2][0];
//   //std::cout<< "New update "<< m_nodeState <<std::endl;
//   seconds = seconds/sysconf(_SC_CLK_TCK);
//   return UpTime() - seconds;

// }


// /**
// * @brief Computes the elapsed time in the format HH:MM:SS
// * @returns the time as  a string
// */
// std::string NodeMonitor::ElapsedTime(long elapsedSeconds) 
// {
//   std::string Time = "";
//   int hours, minutes, seconds, time;
//   std::string stringHours, stringMinutes, stringSeconds;
//   hours = elapsedSeconds / 3600;
//   time = elapsedSeconds % 3600;
//   minutes = time / 60;
//   time = time % 60;
//   seconds = time;

//   if (hours < 10) 
//   {
//     stringHours = "0" + std::to_string(hours);
//   } 
//   else 
//   {
//     stringHours = std::to_string(hours);
//   }

//   if (minutes < 10) 
//   {
//     stringMinutes = "0" + std::to_string(minutes);
//   } 
//   else 
//   {
//     stringMinutes = std::to_string(minutes);
//   }

//   if (seconds < 10) 
//   {
//     stringSeconds = "0" + std::to_string(seconds);
//   } 
//   else 
//   {
//     stringSeconds = std::to_string(seconds);
//   }

//   Time = stringHours + ":" + stringMinutes + ":" + stringSeconds;
//   return Time;
// }


// /**
// * @brief Acquires the RAM size of the processor
// * @returns the RAM size in Kb
// */
// long NodeMonitor::getRamSize() 
// {
//   std::string line, key;
//   long int value, MemTotal;
//   std::ifstream inFile(kProcDirectory + kMeminfoFilename);
// //  std::unique_lock<std::mutex> lock (m_mutex);
//   if (inFile.is_open()) {
//     while (std::getline(inFile, line)) 
//     {
//       std::replace(line.begin(), line.end(), ':', ' ');
//       std::istringstream linestream(line);
//       linestream >> key >> value;

//       if (key == "MemTotal") 
//       {
//         MemTotal = static_cast<float>(value);
//         return MemTotal;
//       }
//     }
//   }
//   return -1.0;
// }





/**
 * @brief Main function
*/
int main(int argc, char** argv)
{
   ros::init(argc, argv, "node_monitoring");
   NodeMonitor topic_monitor;
    //std::unique_ptr<TopicStatistics> statisticsPtr(new TopicStatistics);
    //TopicMonitor topic_monitor(std::move(statisticsPtr));
    ros::Rate rate(20);
    ros::spin();

    // while(ros::ok())
    // {
    //     //topic_monitor.topicMonitoring();
    //     rate.sleep();
    //     ros::spinOnce();
    // }
    return 0;
}