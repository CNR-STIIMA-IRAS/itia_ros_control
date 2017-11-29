#ifndef __NODELET_HARDWARE_INTERFACE__
#define __NODELET_HARDWARE_INTERFACE__

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
# include <sensor_msgs/JointState.h>
# include <ros/ros.h>
# include <itia_futils/itia_futils.h>

extern ros::Publisher pub;

namespace itia
{
  
namespace control
{

class PosVelEffJointInterface : public hardware_interface::JointCommandInterface 
{
public:
  sensor_msgs::JointStatePtr m_joint_target_msgs;
};


class JointTargetHW : public hardware_interface::RobotHW
{
public:
  JointTargetHW(const ros::Publisher& pub, const std::vector<std::string>& joint_names);

  void read();
  void write();

protected:
  ros::Publisher m_pub;
private:
  hardware_interface::JointStateInterface  m_js_interface;
  itia::control::PosVelEffJointInterface m_pj_interface;
   
  
  unsigned int m_n_dof;
  std::vector<std::string> m_joint_names;
  
};

}
}

#endif
