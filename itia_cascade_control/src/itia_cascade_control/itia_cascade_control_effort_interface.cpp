#include <itia_cascade_control/itia_cascade_control_effort_interface.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(itia::control::EffortCascadeControl, controller_interface::ControllerBase);


namespace itia
{
namespace control
{

bool EffortCascadeControl::init ( hardware_interface::EffortJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh )
{
  m_hw=hw;
  std::vector<std::string> joint_names=m_hw->getNames();
  m_cascade_controller.reset(new itia::control::CascadeControl(root_nh,controller_nh,joint_names));
  if (!m_cascade_controller->isWellInit())
    return false;
  for (int i=0;i<joint_names.size();i++)
    m_jh.push_back(m_hw->getHandle(joint_names.at(i)));
  
  return true;
}
void EffortCascadeControl::starting ( const ros::Time& time )
{
  m_cascade_controller->starting(time);
}
void EffortCascadeControl::stopping ( const ros::Time& time )
{
  m_cascade_controller->stopping(time);
}
void EffortCascadeControl::update ( const ros::Time& time, const ros::Duration& period )
{
  m_cascade_controller->update(time,period);
  
  std::vector<double> effort=m_cascade_controller->getEffCmd();
  for (unsigned int i=0;i<m_jh.size();i++)
    m_jh.at(i).setCommand(effort.at(i));
}

}
}
