
 // -------------------------------------------------------------------------------- 
 // Copyright (c) 2017 CNR-ITIA <iras@itia.cnr.it>
 // All rights reserved.
 //
 // Redistribution and use in source and binary forms, with or without
 // modification, are permitted provided that the following conditions are met:
 //
 // 1. Redistributions of source code must retain the above copyright notice,
 // this list of conditions and the following disclaimer.
 // 2. Redistributions in binary form must reproduce the above copyright
 // notice, this list of conditions and the following disclaimer in the
 // documentation and/or other materials provided with the distribution.
 // 3. Neither the name of mosquitto nor the names of its
 // contributors may be used to endorse or promote products derived from
 // this software without specific prior written permission.
 //
 //
 // THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 // AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 // IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 // ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 // LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 // CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 // SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 // INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 // CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 // ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 // POSSIBILITY OF SUCH DAMAGE.
 // -------------------------------------------------------------------------------- 

#include <itia_helios_controller/itia_fir_controller.h>

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(itia::motion::FirController, controller_interface::ControllerBase);

namespace itia
{
namespace motion
{
  
FirController::~FirController()
{

}

bool FirController::init(itia::control::PosVelEffJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  ROS_DEBUG("starting FIR planner '%s%s%s'",GREEN,m_controller_nh.getNamespace().c_str(),RESET);
  m_root_nh = root_nh;
  m_controller_nh = controller_nh;
  m_hw = hw;
  
  m_controller_nh.setCallbackQueue(&m_queue);
  m_diagnostics_pub = m_root_nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics",1);
  
  std::string feedback_topic_name;
  
  if (!m_controller_nh.getParam("feedback_topic_name", feedback_topic_name))
  {
    ROS_FATAL_STREAM(m_controller_nh.getNamespace()+"/'feedback_topic_name' does not exist");
    ROS_FATAL("ERROR DURING STARTING CONTROLLER '%s'", m_controller_nh.getNamespace().c_str());
    return false;
  }
  
  m_js_fb_sub     = m_controller_nh.subscribe<sensor_msgs::JointState>(feedback_topic_name, 1, &itia::rutils::MsgReceiver<sensor_msgs::JointState>::callback, &m_fb_js_rec);
  
  m_joint_names = m_hw->m_joint_target_msgs->name;
  m_nAx = m_joint_names.size();
  
  m_controller_nh.setParam("fir_params/dimension",m_nAx);
  
	ROS_DEBUG("initializing FIR planner '%s%s%s'...\n controlled joints:",GREEN,m_controller_nh.getNamespace().c_str(),RESET);
	for (unsigned int idx=0; idx<m_nAx;idx++)
		ROS_DEBUG("- %s%s%s",GREEN,m_joint_names.at(idx).c_str(),RESET);
  
  
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
      ROS_ERROR_STREAM("Joint name "+m_joint_names.at(iAx)+" is not part of robot_description, where there are the following joints:");
      for( auto j : model.joints_ )
        std::cout << "name: " << j.first << std::endl;
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
  
  m_override_topic = m_root_nh.subscribe<std_msgs::Int64>("/speed_ovr",1,&itia::motion::FirController::overrideCallback,this);
  m_safe_override_topic_1 = m_root_nh.subscribe<std_msgs::Int64>("/safe_ovr_1",1,&itia::motion::FirController::safeOverrideCallback_1,this);
  m_safe_override_topic_2 = m_root_nh.subscribe<std_msgs::Int64>("/safe_ovr_2",1,&itia::motion::FirController::safeOverrideCallback_2,this);
  
  ROS_INFO("FIR planner '%s%s%s' well initialized",GREEN,m_controller_nh.getNamespace().c_str(),RESET);
  return true;
}

void FirController::starting(const ros::Time& time)
{
	ROS_DEBUG("FIR planner '%s%s%s' starting",GREEN,m_controller_nh.getNamespace().c_str(),RESET);
	
  m_configured = false;  
 
  Eigen::VectorXd qini(m_nAx);
  qini.setZero();
  m_motion.resize(3, m_nAx);
  m_motion.setZero();
  
  ROS_DEBUG("waiting");
  m_queue.callAvailable();
  
  ros::Time t0 = ros::Time::now();
  while (!m_fb_js_rec.isANewDataAvailable() && (ros::Time::now() - t0).toSec() <10 )
  {
    m_queue.callAvailable();
    ros::Duration(0.001).sleep();
  }
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

  ROS_DEBUG("configuring planner");
  try 
  {
		m_planner.reset(new itia::helios::FirPlanner(m_controller_nh, "fir_params", qini));
    m_planner->changeVelocityOverride(m_override*m_safe_override_1*m_safe_override_2/10000.0);
    m_planner->setPositionLimit(m_upper_limit,m_lower_limit);
    m_configured = true;
    ROS_DEBUG("started");
  }
  catch (const std::exception& e)
  {
    ROS_FATAL("FAILED DURING HELIOS LOADING: %s", e.what());
    m_configured = false;
  }
  
	ROS_INFO("FIR planner '%s%s%s' well started",GREEN,m_controller_nh.getNamespace().c_str(),RESET);
	
}

void FirController::stopping(const ros::Time& time)
{
  ROS_INFO("[ %s%s%s ] Stopping controller", BOLDGREEN, m_controller_nh.getNamespace().c_str(), RESET);
  m_configured = false;
}

void FirController::update(const ros::Time& time, const ros::Duration& period)
{
  m_queue.callAvailable();
  
  if (m_configured && m_planner)
  {
    m_motion = m_planner->update();
    
    for (int iDim = 0;iDim<m_nAx;iDim++)
    {
      m_hw->m_joint_target_msgs->position.at(iDim) = m_motion(0, iDim);
      m_hw->m_joint_target_msgs->velocity.at(iDim) = m_motion(1, iDim);
      m_hw->m_joint_target_msgs->effort.at(iDim)   = 0.0*m_motion(2, iDim);
    }
    
    m_hw->m_joint_target_msgs->header.stamp = ros::Time::now();
  }
  else
    ROS_FATAL_THROTTLE(1, "[%s%s%s%s] PLANNER NOT CONFIGURED", BOLDRED, m_controller_nh.getNamespace().c_str(), RESET, RED);
  

}

void FirController::overrideCallback(const std_msgs::Int64ConstPtr& msg)
{
  m_override=msg->data;
  if (m_planner)
    m_planner->changeVelocityOverride(m_override*m_safe_override_1*m_safe_override_2/10000.0);
}

void FirController::safeOverrideCallback_1(const std_msgs::Int64ConstPtr& msg)
{
  m_safe_override_1=msg->data;
  if (m_safe_override_1>100)
    m_safe_override_1=100;
  else if (m_safe_override_1<0)
    m_safe_override_1=0;
  
  if (m_planner)
    m_planner->changeVelocityOverride(m_override*m_safe_override_1*m_safe_override_2/10000.0);
}

void FirController::safeOverrideCallback_2(const std_msgs::Int64ConstPtr& msg)
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
