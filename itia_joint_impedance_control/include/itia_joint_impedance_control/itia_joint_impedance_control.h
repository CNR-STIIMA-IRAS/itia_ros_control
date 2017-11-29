#ifndef __itia_joint_impedance_sensor__
#define __itia_joint_impedance_sensor__

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
# include <geometry_msgs/WrenchStamped.h>


# ifdef FOURBYTHREE_MSGS
#  pragma message ( "Using fourbythree messages!" )
#  include <fourbythree_msgs/StiffnessRecommendation.h>
# else
#  pragma message ( "Using standard messages!" )
#  include <std_msgs/Float64MultiArray.h>
# endif


namespace itia
{
  namespace control
  {
    
    class JointImpedanceControl : public controller_interface::Controller<itia::control::PosVelEffJointInterface>
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
      
      Eigen::VectorXd m_target;
      Eigen::VectorXd m_Dtarget;
      
      Eigen::VectorXd m_x;
      Eigen::VectorXd m_Dx;
      Eigen::VectorXd m_DDx;
      Eigen::VectorXd m_Jinv;
      Eigen::VectorXd m_damping;
      Eigen::VectorXd m_k_max;
      Eigen::VectorXd m_k_min;
      Eigen::VectorXd m_torque_deadband;
      
      Eigen::VectorXd m_torque;
      
      double m_recomendation_filter;
      Eigen::VectorXd m_adaptivity;
      Eigen::VectorXd m_adaptivity_filtered;
      
      
      itia::control::PosVelEffJointInterface* m_hw;
      
      itia::rutils::MsgReceiver<sensor_msgs::JointState> m_target_js_rec;
      ros::Subscriber  m_js_target_sub;
      ros::Subscriber  m_adaptivity_sub;
      
      
      itia::rutils::MsgReceiver<sensor_msgs::JointState> m_effort_rec;
      ros::Subscriber  m_effort_sub;
      ros::NodeHandle m_root_nh;
      ros::NodeHandle m_controller_nh;
      
      std::vector< std::string > m_joint_names;
      int m_nAx;
      
      ~JointImpedanceControl();
      
#ifdef FOURBYTHREE_MSGS
      void adaptivityCb(const fourbythree_msgs::StiffnessRecommendationConstPtr msg);
#else
      void adaptivityCb(const std_msgs::Float64MultiArrayConstPtr msg);
#endif
      
    };
    
    
  }
}


#endif
