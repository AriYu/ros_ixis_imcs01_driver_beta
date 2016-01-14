#ifndef IXIS_IMCS01_H_
#define IXIS_IMCS01_H_

#include <ros/ros.h>
#include <string>
#include "serial.hpp"

#define ROBOT_MAX_ENCODER_COUNTS 65535

class iXis_iMCs01{
 public:
  struct uin received_data_;
  std::vector<int> encoder_counts_;
  std::vector<int> last_encoder_counts_;
  std::vector<int> delta_encoder_counts_;
  double delta_encoder_time_;
  double last_encoder_time_;

  iXis_iMCs01()
      : delta_encoder_time_(0), last_encoder_time_(0.0)
  {
    encoder_counts_.resize(3);
    last_encoder_counts_.resize(3);
    delta_encoder_counts_.resize(3);
    for (int i = 0; i < 3; ++i) {
      encoder_counts_[i] = 0;
      last_encoder_counts_[i] = 0;
      delta_encoder_counts_[i] = 0;
    }

  }

  int update(SerialPort *port)
  {
    if(read(port->fd_, &received_data_, sizeof(received_data_)) != sizeof(received_data_)){
      return -1;
    }else{
      parseEncoderCounts();
      return 0;
    }
  }
  int parseEncoderCounts()
  {
    //! 0 is right, 1 is left.
    for (int i = 0; i < encoder_counts_.size(); ++i) {
      encoder_counts_[i] = (int)received_data_.ct[i+1];
      ROS_INFO("ct[%d] : %d", i, encoder_counts_[i]);
    }
    printf("\n");
    delta_encoder_time_ = (double)(received_data_.time) - last_encoder_time_;

    if(delta_encoder_time_ < 0){
      delta_encoder_time_ = 65535 - (last_encoder_time_ - received_data_.time);
    }
    delta_encoder_time_ = delta_encoder_time_ / 1000.0; // [ms] -> [s]
    last_encoder_time_ = (double)(received_data_.time);
    
    for(int i = 0; i < delta_encoder_counts_.size(); ++i){
      if(delta_encoder_counts_[i] == -1 
         || encoder_counts_[i] == last_encoder_counts_[i]){ // First time.

        delta_encoder_counts_[i] = 0;

      }else{
        delta_encoder_counts_[i] = encoder_counts_[i] - last_encoder_counts_[i];

        // checking imcs01 counter overflow.
        if(delta_encoder_counts_[i] > ROBOT_MAX_ENCODER_COUNTS/10){
          delta_encoder_counts_[i] = delta_encoder_counts_[i] - ROBOT_MAX_ENCODER_COUNTS;
        }
        if(delta_encoder_counts_[i] < -ROBOT_MAX_ENCODER_COUNTS/10){
          delta_encoder_counts_[i] = delta_encoder_counts_[i] + ROBOT_MAX_ENCODER_COUNTS;
        }
      }
      last_encoder_counts_[i] = encoder_counts_[i];
    }

    return 0;
  }
};
  

#endif 
