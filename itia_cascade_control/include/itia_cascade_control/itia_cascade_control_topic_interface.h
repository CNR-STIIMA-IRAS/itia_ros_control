#ifndef __itia_cascade_control_topic_interface__
#define __itia_cascade_control_topic_interface__

#include <itia_cascade_control/itia_cascade_control.h>

namespace itia
{
	namespace control
	{
		
		class TopicCascadeControl : public  controller_interface::Controller<itia::control::PosVelEffJointInterface>
		{
		public:
			bool init(itia::control::PosVelEffJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
      void update(const ros::Time& time, const ros::Duration& period);
      void starting(const ros::Time& time);
      void stopping(const ros::Time& time);
      
		protected:
			itia::control::PosVelEffJointInterface* m_hw;
      boost::shared_ptr<itia::control::CascadeControl> m_cascade_controller;
      
		};
		
		
	}
}

# endif