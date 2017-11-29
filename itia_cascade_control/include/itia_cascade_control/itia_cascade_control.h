#ifndef __itia_cascade_control__
#define __itia_cascade_control__

# include <controller_interface/controller.h>
# include <hardware_interface/joint_command_interface.h>
# include <itia_nodelet_hw_interface/nodelet_hw_interface.h>
# include <itia_controllers_and_filters/discrete_state_space.h>
# include <itia_dynamics_core/itia_primitives.h>
# include <thread>
# include <mutex>
# include <boost/graph/graph_concepts.hpp>
# include <ros/ros.h>
# include <itia_rutils/itia_rutils.h>
# include <sensor_msgs/JointState.h>
# include <pluginlib/class_list_macros.h>


namespace itia
{
	namespace control
	{
		
		class CascadeControl
		{
		public:
      CascadeControl(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh, std::vector<std::string> joint_names);
			void update(const ros::Time& time, const ros::Duration& period);
			void starting(const ros::Time& time);
			void stopping(const ros::Time& time);
			
      std::vector<double>& getPosCmd()
      {
        return m_pos_cmd;
      }
      std::vector<double>& getVelCmd()
      {
        return m_vel_cmd;
      }
      std::vector<double>& getEffCmd()
      {
        return m_eff_cmd;
      }
      std::vector<std::string> getJointsName()
      {
        return m_joint_names;
      };
      bool isWellInit(){return m_well_init;};
      
		protected:
			bool m_well_init;
      bool m_use_feedback;
      bool m_use_model_feedforward;
      
      ros::CallbackQueue m_queue;
			boost::shared_ptr<ros::AsyncSpinner> m_spinner;
			
			std::vector<hardware_interface::JointHandle> m_joint_handles;
			std::vector<itia::control::DiscreteStateSpacePtr> m_pos_controllers;
			std::vector<itia::control::DiscreteStateSpacePtr> m_vel_controllers;
      std::vector<itia::control::DiscreteStateSpacePtr> m_vel_filters;
      std::vector<itia::control::DiscreteStateSpacePtr> m_target_vel_filters;
      std::vector<itia::control::DiscreteStateSpacePtr> m_pos_filters;
      std::vector<itia::control::DiscreteStateSpacePtr> m_target_pos_filters;
      std::vector<itia::control::DiscreteStateSpacePtr> m_eff_filters;
      Eigen::MatrixXd a;
      
			double m_pos_deadband;
      int m_nAx;
			std::vector<std::string> m_joint_names;
      std::vector<double> m_static_friction;
      std::vector<double> m_static_friction_threshold;
      std::vector<double> m_static_friction_past_state;
      std::vector<double> m_position_minimum_error;
      
      std::vector<double> m_pos_cmd;
      std::vector<double> m_vel_cmd;
      std::vector<double> m_eff_cmd;
      
			itia::rutils::MsgReceiver<sensor_msgs::JointState> m_fb_js_rec;
			itia::rutils::MsgReceiver<sensor_msgs::JointState> m_target_js_rec;
			itia::rutils::MsgReceiver<sensor_msgs::JointState> m_ffw_js_rec;
      boost::shared_ptr<itia::dynamics::Chain> m_chain;
      
			ros::Subscriber  m_js_target_sub;
			ros::Subscriber  m_js_fb_sub;
			ros::Subscriber  m_js_ffw_sub;
			
			ros::NodeHandle m_root_nh;
			ros::NodeHandle m_controller_nh;
			bool m_configured;
			bool m_feedforward_active;
			bool m_use_target_torque;
      
      
			void timerCallback(const ros::TimerEvent& event);
			void stopThreads();
		};
		
		
	}
}

# endif