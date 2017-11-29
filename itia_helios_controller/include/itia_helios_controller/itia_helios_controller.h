#ifndef __itia_helios_controller__
#define __itia_helios_controller__

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
# include <itia_nodelet_hw_interface/nodelet_hw_interface.h>
# include <thread>
# include <mutex>
#include <boost/graph/graph_concepts.hpp>
# include <ros/ros.h>
# include <itia_rutils/itia_rutils.h>
# include <sensor_msgs/JointState.h>
#include <pluginlib/class_list_macros.h>
# include <itia_helios/mpc_planner.h>
namespace itia
{
namespace motion
{
class HeliosController : public controller_interface::Controller<itia::control::PosVelEffJointInterface>
{
public:
  bool init(itia::control::PosVelEffJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
  void update(const ros::Time& time, const ros::Duration& period);
  void starting(const ros::Time& time);
  void stopping(const ros::Time& time);

protected:

  std::vector<hardware_interface::JointHandle> m_joint_handles;
  
  int m_nAx;
  
  std::vector<std::string> m_joint_names;
  itia::control::PosVelEffJointInterface* m_hw;
  itia::rutils::MsgReceiver<sensor_msgs::JointState> m_fb_js_rec;
  ros::Subscriber  m_js_fb_sub;
  ros::NodeHandle m_root_nh;
  ros::NodeHandle m_controller_nh;
  bool m_configured;
  itia::JMotion m_motion;
  
  boost::shared_ptr<itia::helios::MpcPlanner> m_planner;
  
  ~HeliosController();
};


}
}

# endif