#ifndef __itia_velocity_control_velocity_interface__
#define __itia_velocity_control_velocity_interface__

#include <itia_velocity_control/itia_velocity_control.h>


namespace itia
{
  namespace control
  {
    
    class VelocityInterfaceVelocityControl : public controller_interface::Controller<hardware_interface::VelocityJointInterface>
    {
    public:
      bool init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
      void update(const ros::Time& time, const ros::Duration& period);
      void starting(const ros::Time& time);
      void stopping(const ros::Time& time);
      
    protected:
      hardware_interface::VelocityJointInterface* m_hw;
      boost::shared_ptr<itia::control::VelocityControl> m_velocity_controller;
      std::vector<hardware_interface::JointHandle> m_jh;
    };
    
    
  }
}

# endif