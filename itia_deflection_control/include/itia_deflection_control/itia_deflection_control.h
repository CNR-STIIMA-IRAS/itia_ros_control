#ifndef __itia_deflection_control__
#define __itia_deflection_control__

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
# include <itia_nodelet_hw_interface/nodelet_hw_interface.h>
#include <itia_controllers_and_filters/discrete_state_space.h>
# include <thread>
# include <mutex>
#include <boost/graph/graph_concepts.hpp>
# include <ros/ros.h>
# include <itia_rutils/itia_rutils.h>
# include <sensor_msgs/JointState.h>
#include <pluginlib/class_list_macros.h>

namespace itia
{
	namespace control
	{
		
		class DeflectionControl : public controller_interface::Controller<itia::control::PosVelEffJointInterface>
		{
		public:
			bool init(itia::control::PosVelEffJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
			void update(const ros::Time& time, const ros::Duration& period);
			void starting(const ros::Time& time);
			void stopping(const ros::Time& time);
			
		protected:
			ros::CallbackQueue m_queue;
			boost::shared_ptr<ros::AsyncSpinner> m_spinner;
			
			std::vector<hardware_interface::JointHandle> m_joint_handles;
			std::vector<itia::control::DiscreteStateSpacePtr> m_deflection_controllers;
			std::vector<itia::control::DiscreteStateSpacePtr> m_link_pos_filters;
      std::vector<itia::control::DiscreteStateSpacePtr> m_link_vel_filters;
      std::vector<itia::control::DiscreteStateSpacePtr> m_tau_el_filters;
      std::vector<itia::control::DiscreteStateSpacePtr> m_target_tau_el_filters;
      std::vector<itia::control::DiscreteStateSpacePtr> m_target_Dtau_el_filters;
      std::vector<double> m_max_torque;
      
			
			int m_nAx;
			
			std::vector<std::string> m_joint_names;
			std::vector<double> m_elasticity;
      std::vector<double> m_torque_minimum_error;
			bool m_is_in_error;
			
			itia::control::PosVelEffJointInterface* m_hw;
			itia::rutils::MsgReceiver<sensor_msgs::JointState> m_fb_js_rec;
			itia::rutils::MsgReceiver<sensor_msgs::JointState> m_target_js_rec;
			itia::rutils::MsgReceiver<sensor_msgs::JointState> m_ffw_js_rec;
			
			ros::Subscriber  m_js_target_sub;
			ros::Subscriber  m_js_fb_sub;
			ros::Subscriber  m_js_ffw_sub;
			
			ros::NodeHandle m_root_nh;
			ros::NodeHandle m_controller_nh;
			bool m_configured;
			bool m_feedforward_active;
			
			void timerCallback(const ros::TimerEvent& event);
			void stopThreads();
			~DeflectionControl();
		};
		
		
	}
}

# endif