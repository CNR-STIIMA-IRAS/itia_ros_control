#include <itia_fake_controller_system/itia_fake_controller_system.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(itia::FakeControllerSystem, controller_interface::ControllerBase);


namespace itia{
  

FakeControllerSystem::~FakeControllerSystem()
{
  ROS_INFO("shutting down...");
};


bool FakeControllerSystem::init(itia::control::PosVelEffJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  m_root_nh = root_nh;
  m_controller_nh = controller_nh;
  m_hw = hw;
  m_controller_nh.setCallbackQueue(&m_queue);
  
  std::string setpoint_topic_name;
  

  if (!m_controller_nh.getParam("setpoint_topic_name", setpoint_topic_name))
  {
    ROS_FATAL_STREAM(m_controller_nh.getNamespace()+"/'setpoint_topic_name' does not exist");
    ROS_FATAL("ERROR DURING INITIALIZATION CONTROLLER '%s'", m_controller_nh.getNamespace().c_str());
    return false;
  }
  
  m_jt_sub = m_root_nh.subscribe(setpoint_topic_name,1,&itia::FakeControllerSystem::callbackJointTarget,this);
  
  m_js_name=m_hw->getNames();
  m_nAx = m_js_name.size();
  
  m_position.resize(m_nAx);
  m_position_filt.resize(m_nAx);
  m_velocity.resize(m_nAx);
  m_effort.resize(m_nAx);
  
  std::fill(m_position.begin(),      m_position.end(),      0.0);
  std::fill(m_position_filt.begin(), m_position_filt.end(), 0.0);
  std::fill(m_position.begin(),      m_position.end(),      0.0);
  std::fill(m_effort.begin(),        m_effort.end(),        0.0);
  if (!m_controller_nh.getParam("initial_position", m_position))
  {
    ROS_WARN_STREAM(m_controller_nh.getNamespace()+"/'initial_position' does not exist, set equal to 0");
    std::fill(m_position.begin(),m_position.end(),0.0);
  }
  m_position_filt=m_position;
  
  if (!m_controller_nh.getParam("time_constant", m_time_constant))
  {
    ROS_WARN_STREAM(m_controller_nh.getNamespace()+"/'time_constant' does not exist, set equal to 0.01");
    m_time_constant=0.01;
  }
  
  
  ROS_INFO("Controller '%s%s%s' controls the following joints:",GREEN,m_controller_nh.getNamespace().c_str(),RESET);
  for (unsigned int idx=0;idx<m_nAx;idx++)
    ROS_INFO(" - %s%s%s",GREEN,m_js_name.at(idx).c_str(),RESET);
  
  
  ROS_INFO("Controller '%s%s%s' well initialized",GREEN,m_controller_nh.getNamespace().c_str(),RESET);
  return true;
  
}

void FakeControllerSystem::starting(const ros::Time& time)
{
  ROS_INFO("Controller '%s%s%s' well started",GREEN,m_controller_nh.getNamespace().c_str(),RESET);
}

void FakeControllerSystem::stopping(const ros::Time& time)
{
  ROS_INFO("[ %s%s%s ] Stopping controller", BOLDGREEN, m_controller_nh.getNamespace().c_str(), RESET);
  m_hw->m_joint_target_msgs->header.stamp = ros::Time::now();
}

void FakeControllerSystem::update(const ros::Time& time, const ros::Duration& period)
{
  m_queue.callAvailable();
  m_filt_coeff=std::exp(-period.toSec()/m_time_constant);

  for(int idx=0;idx<m_nAx;idx++)
  {
    m_velocity.at(idx) = (1-m_filt_coeff)*(m_position.at(idx)-m_position_filt.at(idx))/period.toSec();
    m_position_filt.at(idx)=m_filt_coeff*m_position_filt.at(idx)+(1-m_filt_coeff)*m_position.at(idx);
    m_hw->m_joint_target_msgs->position=m_position_filt;
    m_hw->m_joint_target_msgs->velocity=m_velocity;
    m_hw->m_joint_target_msgs->effort=m_effort;
    m_hw->m_joint_target_msgs->name=m_js_name;
  }
  m_hw->m_joint_target_msgs->header.stamp = ros::Time::now();
  
  
}

void FakeControllerSystem::callbackJointTarget(const sensor_msgs::JointState::ConstPtr& msg)
{
  if (msg->position.size()<m_nAx)
  {
    ROS_ERROR("targer dimension are wrong");
    return;
  }
  for(int idx=0;idx<m_nAx;idx++)
  {
    m_position.at(idx)   = msg->position.at(idx);
  }
}
  
}