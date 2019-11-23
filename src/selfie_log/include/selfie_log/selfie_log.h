#ifndef SELFIE_LOG_H
#define SELFIE_LOG_H

#include <ros/ros.h>

#define SELFIE_LOG(log_name, msg) if(log_name) ROS_INFO("%s: %s",#log_name, msg);


#endif // SELFIE_LOG_H
