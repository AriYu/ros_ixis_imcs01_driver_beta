#ifndef SERIAL_H_
#define SERIAL_H_

#include <iostream>
#include <string>
#include <exception>
#include <termios.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <fcntl.h>
#include <system_error>

#include <ros/ros.h>

#include "imcs01_driver/driver/urbtc.h"   /* Linux specific part */
#include "imcs01_driver/driver/urobotc.h" /* OS independent part */

class SerialPort{
public:
  SerialPort(std::string portName)
  {
    fd_ = open(portName.c_str(), O_RDWR);
    if(fd_ < 0)
    {
      ROS_WARN("%s: Open error", portName.c_str());
      exit(-1);
    }
    if (ioctl(fd_, URBTC_CONTINUOUS_READ) < 0){
      ROS_WARN("ioctl: URBTC_CONTINUOUS_READ error");
      exit(1);
    }
    if (ioctl(fd_, URBTC_BUFREAD) < 0){
      ROS_WARN("ioctl: URBTC_CONTINUOUS_READ error");
      exit(1);
    }
  }
  ~SerialPort()
  {
    close(fd_);
  }
public:
  int fd_;
 private:
  
};

#endif
