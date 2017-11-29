#include <itia_velocity_control/itia_velocity_control_on_velocity_interface.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(itia::control::VelocityInterfaceVelocityControl, controller_interface::ControllerBase);


namespace itia
{
  namespace control
  {
    
    bool VelocityInterfaceVelocityControl::init ( hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh )
    {
      m_hw=hw;
      std::vector<std::string> joint_names=m_hw->getNames();
      m_velocity_controller.reset(new itia::control::VelocityControl(root_nh,controller_nh,joint_names));
      if (!m_velocity_controller->isWellInit())
        return false;
      for (int i=0;i<joint_names.size();i++)
        m_jh.push_back(m_hw->getHandle(joint_names.at(i)));
      
      return true;
    }
    void VelocityInterfaceVelocityControl::starting ( const ros::Time& time )
    {
      m_velocity_controller->starting(time);
    }
    void VelocityInterfaceVelocityControl::stopping ( const ros::Time& time )
    {
      m_velocity_controller->stopping(time);
    }
    void VelocityInterfaceVelocityControl::update ( const ros::Time& time, const ros::Duration& period )
    {
      m_velocity_controller->update(time,period);
      
      std::vector<double> velocity=m_velocity_controller->getVelCmd();
      for (unsigned int i=0;i<m_jh.size();i++)
      {
//         ROS_INFO("vel(%u)=%f",i,velocity.at(i));
//         m_jh.at(i).setCommand(0.0);
        m_jh.at(i).setCommand(velocity.at(i));
        
      }
    }
    
  }
}
