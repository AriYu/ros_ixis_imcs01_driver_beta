#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <serial.hpp>
#include <ixis_imcs01.hpp>
#include <string>

#include <boost/thread.hpp>

class ixis_imcs01_driver
{
 public:
  ixis_imcs01_driver(ros::NodeHandle nh, std::string portName_right, std::string portName_left)
      : nh_(nh), rate_(100), port_right_(portName_right), port_left_(portName_left)
  {
    wheel_state_.name.resize(6);
    wheel_state_.position.resize(6);
    wheel_state_.velocity.resize(6);

    wheel_state_.name[0] = "wheel_front_right";
    wheel_state_.position[0] = 0.0;
    wheel_state_.velocity[0] = 0.0;
    wheel_state_.name[1] = "wheel_front_left";
    wheel_state_.position[1] = 0.0;
    wheel_state_.velocity[1] = 0.0;
    wheel_state_.name[2] = "wheel_middle_right";
    wheel_state_.position[2] = 0.0;
    wheel_state_.velocity[2] = 0.0;
    wheel_state_.name[3] = "wheel_middle_left";
    wheel_state_.position[3] = 0.0;
    wheel_state_.velocity[3] = 0.0;
    wheel_state_.name[4] = "wheel_rear_right";
    wheel_state_.position[4] = 0.0;
    wheel_state_.velocity[4] = 0.0;
    wheel_state_.name[5] = "wheel_rear_left";
    wheel_state_.position[5] = 0.0;
    wheel_state_.velocity[5] = 0.0;

    wheel_velo_pub_ = nh_.advertise<sensor_msgs::JointState>("/wheel_state", 10);
  }

  void calculateWheelSpeed()
  {
    for(int i = 0; i < 3; i++){
      wheel_state_.velocity[2*i] = (imcs01_right_.delta_encoder_counts_[i]*0.000033/imcs01_right_.delta_encoder_time_);
      wheel_state_.velocity[2*i+1] = -(imcs01_left_.delta_encoder_counts_[i]*0.000033/imcs01_left_.delta_encoder_time_);
    }
  }

  void run()
  {
     while(nh_.ok())
	{
      imcs01_left_.update(&port_left_);
      imcs01_right_.update(&port_right_);
      calculateWheelSpeed();
      wheel_state_.header.stamp = ros::Time::now();
      wheel_velo_pub_.publish(wheel_state_);
	  ros::spinOnce();
	  rate_.sleep();
    }
  }
 private:
  SerialPort port_right_;
  SerialPort port_left_;
  iXis_iMCs01 imcs01_right_;
  iXis_iMCs01 imcs01_left_;
  ros::NodeHandle nh_;
  ros::Rate rate_;
  ros::Publisher wheel_velo_pub_;
  sensor_msgs::JointState wheel_state_;
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "ixis_imcs01_driver");
  ros::NodeHandle nh;
  
  ixis_imcs01_driver driver(nh, "/dev/urbtc0", "/dev/urbtc1");
  driver.run();


  return 0;
}
