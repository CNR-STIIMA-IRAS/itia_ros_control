#ifndef __itia_model_feedforward_controller__
#define __itia_model_feedforward_controller__

# include <controller_interface/controller.h>
# include <hardware_interface/joint_command_interface.h>
# include <itia_nodelet_hw_interface/nodelet_hw_interface.h>
# include <itia_controllers_and_filters/discrete_state_space.h>
# include <itia_rutils/itia_rutils.h>
# include <itia_dynamics_core/itia_primitives.h>
# include <thread>
# include <mutex>
# include <boost/graph/graph_concepts.hpp>
# include <ros/ros.h>
# include <sensor_msgs/JointState.h>
# include <pluginlib/class_list_macros.h>

namespace itia
{
  namespace control
  {
    
    class FeedForwardControl : public controller_interface::Controller<itia::control::PosVelEffJointInterface>
    {
    public:
      bool init(itia::control::PosVelEffJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
      void update(const ros::Time& time, const ros::Duration& period);
      void starting(const ros::Time& time);
      void stopping(const ros::Time& time);
      
    protected:
      ros::CallbackQueue m_queue;
      boost::shared_ptr<ros::AsyncSpinner> m_spinner;
      bool m_is_configured=false;
      std::vector<hardware_interface::JointHandle> m_joint_handles;
      boost::shared_ptr<itia::dynamics::Chain> m_chain;
      Eigen::VectorXd m_q;
      Eigen::VectorXd m_Dq;
      Eigen::VectorXd m_DDq;
      Eigen::VectorXd m_torque;
      double m_transient_time;
      Eigen::VectorXd m_Dq_filt;
      double m_filter;
      
      ros::Time m_start_time;
      int m_nAx;
      std::vector<std::string> m_joint_names;
      
      itia::control::PosVelEffJointInterface* m_hw;
      itia::rutils::MsgReceiver<sensor_msgs::JointState> m_target_js_rec;
      ros::Subscriber  m_js_target_sub;
      
      ros::NodeHandle m_root_nh;
      ros::NodeHandle m_controller_nh;
      
      void timerCallback(const ros::TimerEvent& event);
      void stopThreads();
      ~FeedForwardControl();
    };
    
    
  }
}


#endif