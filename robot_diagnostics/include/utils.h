#ifndef UTILS_H
#define UTILS_H

const std::string kProcDirectory{"/proc/"};
const std::string kStatusFilename{"/status"};
const std::string kUptimeFilename{"/uptime"};
const std::string kStatFilename{"/stat"};
const std::string kMeminfoFilename{"/meminfo"};




enum CPUStates {
  kUser_ = 0,
  kNice_,
  kSystem_,
  kIdle_,
  kIOwait_,
  kIRQ_,
  kSoftIRQ_,
  kSteal_,
  kGuest_,
  kGuestNice_
};


enum NodeFilter
{
    DEFAULT,
    ADD,
    REMOVE
};

#endif
