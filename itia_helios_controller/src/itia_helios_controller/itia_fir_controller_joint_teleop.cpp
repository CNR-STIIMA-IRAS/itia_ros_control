#include <itia_helios_controller/itia_fir_controller_joint_teleop.h>
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(itia::motion::JointTeleopFirController, controller_interface::ControllerBase);

namespace itia
{
namespace motion
{
  
JointTeleopFirController::~JointTeleopFirController()
{

}

bool JointTeleopFirController::init(itia::control::PosVelEffJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  m_root_nh = root_nh;
  m_controller_nh = controller_nh;
  m_hw = hw;
  m_diagnostics_pub = m_root_nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics",1);
  
  
  std::string feedback_topic_name;
  std::string setpoint_topic_name;
  
  if (!m_controller_nh.getParam("feedback_topic_name", feedback_topic_name))
  {
    ROS_FATAL_STREAM(m_controller_nh.getNamespace()+"/'feedback_topic_name' does not exist");
    ROS_FATAL("ERROR DURING STARTING CONTROLLER '%s'", m_controller_nh.getNamespace().c_str());
    return false;
  }
  if (!m_controller_nh.getParam("setpoint_topic_name", setpoint_topic_name))
  {
    ROS_FATAL_STREAM(m_controller_nh.getNamespace()+"/'setpoint_topic_name' does not exist");
    ROS_FATAL("ERROR DURING INITIALIZATION CONTROLLER '%s'", m_controller_nh.getNamespace().c_str());
  }
  
  double timeout;
  if (!m_controller_nh.getParam("message_timeout", timeout))
  {
    ROS_FATAL_STREAM(m_controller_nh.getNamespace()+"/'message_timeout' does not exist, set equal to 0.001 [s]");
    timeout=0.001;
  }
  m_message_timeout=ros::Duration(timeout);
  
  m_js_fb_sub     = m_controller_nh.subscribe<sensor_msgs::JointState>(feedback_topic_name, 1, &itia::rutils::MsgReceiver<sensor_msgs::JointState>::callback, &m_fb_js_rec);
  m_js_target_sub = m_controller_nh.subscribe<sensor_msgs::JointState>(setpoint_topic_name,    1, &itia::rutils::MsgReceiver<sensor_msgs::JointState>::callback, &m_target_js_rec);
  
  m_joint_names = m_hw->getNames();
  m_nAx = m_joint_names.size();
  
  m_controller_nh.setParam("fir_params/dimension",m_nAx);
  
	ROS_INFO("FIR planner '%s%s%s' well initialized, controlled joints:",GREEN,m_controller_nh.getNamespace().c_str(),RESET);
	for (unsigned int idx=0; idx<m_nAx;idx++)
		ROS_INFO("- %s%s%s",GREEN,m_joint_names.at(idx).c_str(),RESET);
  
  urdf::Model model;
  if (!model.initParam("/robot_description"))
  {
    ROS_ERROR("Urdf '/robot_description' does not exist");
    return false;
  }
  
  m_upper_limit.resize(m_nAx);
  m_lower_limit.resize(m_nAx);
  for (int iAx=0;iAx<m_nAx;iAx++)
  {
    if (!model.getJoint(m_joint_names.at(iAx)))
    {
      diagnostic_msgs::DiagnosticArray diag_msg;
      diag_msg.header.stamp=ros::Time::now();
      diag_msg.status.resize(1);
      diag_msg.status.at(0).hardware_id="FourByThreeRobot";
      diag_msg.status.at(0).level=diagnostic_msgs::DiagnosticStatus::ERROR;
      diag_msg.status.at(0).message="Joint name "+m_joint_names.at(iAx)+" is not part of robot_description";
      diag_msg.status.at(0).values.resize(1);
      diag_msg.status.at(0).values.at(0).key="MISSING JOINT";
      diag_msg.status.at(0).values.at(0).value=m_joint_names.at(iAx);
      m_diagnostics_pub.publish(diag_msg);
      return false;
    }
    m_upper_limit (iAx) = model.getJoint(m_joint_names.at(iAx))->limits->upper;
    m_lower_limit (iAx) = model.getJoint(m_joint_names.at(iAx))->limits->lower;
  }
  
  m_override=m_safe_override_1=m_safe_override_2=100;
  
  m_override_topic = m_root_nh.subscribe<std_msgs::Int64>("/speed_ovr",1,&itia::motion::JointTeleopFirController::overrideCallback,this);
  m_safe_override_topic_1 = m_root_nh.subscribe<std_msgs::Int64>("/safe_ovr_1",1,&itia::motion::JointTeleopFirController::safeOverrideCallback_1,this);
  m_safe_override_topic_2 = m_root_nh.subscribe<std_msgs::Int64>("/safe_ovr_2",1,&itia::motion::JointTeleopFirController::safeOverrideCallback_2,this);
  return true;
}

void JointTeleopFirController::starting(const ros::Time& time)
{
  m_configured = false;  
 
  Eigen::VectorXd qini(m_nAx);
  qini.setZero();
  m_motion.resize(3, m_nAx);
  m_motion.setZero();
  
  ROS_INFO("waiting");
  ros::WallTime t0 = ros::WallTime::now();
  while (!m_fb_js_rec.isANewDataAvailable() && (ros::WallTime::now() - t0).toSec() <10 )
    ros::WallDuration(0.001).sleep();
  if (!m_fb_js_rec.isANewDataAvailable())
	{
		ROS_FATAL("FIR planner did not receive message from '%s' topic",m_js_fb_sub.getTopic().c_str());
    return;
	}
  
  if (m_fb_js_rec.getData().position.size() != m_nAx)
  {
    ROS_FATAL("error feedback size is wrong");
    return;
  }
  for (int idx = 0;idx<m_nAx;idx++)
		qini(idx) = m_fb_js_rec.getData().position.at(idx);

  ROS_INFO("configuring planner");
  try 
  {
		m_planner.reset(new itia::helios::FirPlanner(m_controller_nh, "fir_params", qini));
    m_planner->changeVelocityOverride(m_override*m_safe_override_1*m_safe_override_2/10000.0);
    m_configured = true;
    ROS_INFO("started");
  }
  catch (const std::exception& e)
  {
    ROS_FATAL("FAILED DURING HELIOS LOADING: %s", e.what());
    m_configured = false;
  }
  

}

void JointTeleopFirController::stopping(const ros::Time& time)
{
  ROS_INFO("[ %s%s%s ] Stopping controller", BOLDGREEN, m_controller_nh.getNamespace().c_str(), RESET);
  m_configured = false;
}

void JointTeleopFirController::update(const ros::Time& time, const ros::Duration& period)
{
  Eigen::VectorXd vel(m_nAx);
  vel.setZero();
  if ((ros::Time::now()-m_target_js_rec.getData().header.stamp)<m_message_timeout)
  {
    for (int idx = 0;idx<m_nAx;idx++)
      vel(idx) = m_target_js_rec.getData().velocity.at(idx);
    m_motion = m_planner->updateVel(vel);
  } 
  else
  {
    //m_motion = m_planner->softStop();
    m_motion = m_planner->updateVel(vel);
  }
    
  
  for (int iDim = 0;iDim<m_nAx;iDim++)
  {
    m_hw->m_joint_target_msgs->position.at(iDim) = m_motion(0, iDim);
    m_hw->m_joint_target_msgs->velocity.at(iDim) = m_motion(1, iDim);
    m_hw->m_joint_target_msgs->effort.at(iDim)   = m_motion(2, iDim);
  }
  m_hw->m_joint_target_msgs->header.stamp = ros::Time::now();

}

void JointTeleopFirController::overrideCallback(const std_msgs::Int64ConstPtr& msg)
{
  m_override=msg->data;
  if (m_planner)
    m_planner->changeVelocityOverride(m_override*m_safe_override_1*m_safe_override_2/10000.0);
}

void JointTeleopFirController::safeOverrideCallback_1(const std_msgs::Int64ConstPtr& msg)
{
  m_safe_override_1=msg->data;
  if (m_safe_override_1>100)
    m_safe_override_1=100;
  else if (m_safe_override_1<0)
    m_safe_override_1=0;
  
  if (m_planner)
    m_planner->changeVelocityOverride(m_override*m_safe_override_1*m_safe_override_2/10000.0);
}

void JointTeleopFirController::safeOverrideCallback_2(const std_msgs::Int64ConstPtr& msg)
{
  m_safe_override_2=msg->data;
  if (m_safe_override_2>100)
    m_safe_override_2=100;
  else if (m_safe_override_2<0)
    m_safe_override_2=0;
  
  if (m_planner)
    m_planner->changeVelocityOverride(m_override*m_safe_override_1*m_safe_override_2/10000.0);
}



}
}