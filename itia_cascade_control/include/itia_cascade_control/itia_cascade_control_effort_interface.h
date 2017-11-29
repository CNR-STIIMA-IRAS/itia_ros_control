#ifndef __itia_cascade_control_effort_interface__
#define __itia_cascade_control_effort_interface__

#include <itia_cascade_control/itia_cascade_control.h>


namespace itia
{
	namespace control
	{
		
		class EffortCascadeControl : public controller_interface::Controller<hardware_interface::EffortJointInterface>
		{
		public:
      bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
			void update(const ros::Time& time, const ros::Duration& period);
			void starting(const ros::Time& time);
			void stopping(const ros::Time& time);
			
		protected:
      hardware_interface::EffortJointInterface* m_hw;
      boost::shared_ptr<itia::control::CascadeControl> m_cascade_controller;
      std::vector<hardware_interface::JointHandle> m_jh;
    };
		
		
	}
}

# endif