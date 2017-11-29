#include <itia_cascade_control/itia_cascade_control_topic_interface.h>

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(itia::control::TopicCascadeControl, controller_interface::ControllerBase);


namespace itia
{
namespace control
{

bool TopicCascadeControl::init ( PosVelEffJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh )
{
  m_hw=hw;
  std::vector<std::string> joint_names=m_hw->getNames();
  m_cascade_controller.reset(new itia::control::CascadeControl(root_nh,controller_nh,joint_names));
  return  m_cascade_controller->isWellInit();
}
void TopicCascadeControl::starting ( const ros::Time& time )
{
  m_cascade_controller->starting(time);
}
void TopicCascadeControl::stopping ( const ros::Time& time )
{
  m_cascade_controller->stopping(time);
  m_hw->m_joint_target_msgs->position = m_cascade_controller->getPosCmd();
  m_hw->m_joint_target_msgs->velocity = m_cascade_controller->getVelCmd();
  m_hw->m_joint_target_msgs->effort   = m_cascade_controller->getEffCmd();
  m_hw->m_joint_target_msgs->header.stamp = ros::Time::now();
}
void TopicCascadeControl::update ( const ros::Time& time, const ros::Duration& period )
{
  m_cascade_controller->update(time,period);
  m_hw->m_joint_target_msgs->position = m_cascade_controller->getPosCmd();
  m_hw->m_joint_target_msgs->velocity = m_cascade_controller->getVelCmd();
  m_hw->m_joint_target_msgs->effort   = m_cascade_controller->getEffCmd();
  m_hw->m_joint_target_msgs->header.stamp = ros::Time::now();
}



}
}
