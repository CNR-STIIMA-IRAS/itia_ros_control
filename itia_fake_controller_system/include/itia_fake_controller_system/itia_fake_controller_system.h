#ifndef __ITIA_FAKE_CONTROLLER_SYSYEM__
#define __ITIA_FAKE_CONTROLLER_SYSYEM__


#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
# include <itia_nodelet_hw_interface/nodelet_hw_interface.h>
# include <thread>
#include <boost/graph/graph_concepts.hpp>
# include <ros/ros.h>
# include <sensor_msgs/JointState.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>

namespace itia{
  
  
  
  class FakeControllerSystem : public controller_interface::Controller<itia::control::PosVelEffJointInterface>
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
    
    itia::control::PosVelEffJointInterface* m_hw;
    
    ros::Subscriber          m_jt_sub;
    
    ros::NodeHandle m_root_nh;
    ros::NodeHandle m_controller_nh;
    

    unsigned int m_nAx;
    double m_filt_coeff;
    std::thread m_main_thread;
    bool m_stop;
    double m_time_constant;
    
    std::vector<double> m_position;
    std::vector<double> m_position_filt;
    std::vector<double> m_velocity;
    std::vector<double> m_effort;
    std::vector<std::string> m_js_name;
    
    
    ~FakeControllerSystem();
    void callbackJointTarget(const sensor_msgs::JointState::ConstPtr& msg);
  };
  
  
  
}

# endif