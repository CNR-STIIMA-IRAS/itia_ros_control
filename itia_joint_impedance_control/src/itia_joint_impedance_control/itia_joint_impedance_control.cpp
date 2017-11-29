
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

#include <itia_joint_impedance_control/itia_joint_impedance_control.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(itia::control::JointImpedanceControl, controller_interface::ControllerBase);


namespace itia
{
namespace control
{
    
    
JointImpedanceControl::~JointImpedanceControl()
{
  
}

bool JointImpedanceControl::init(itia::control::PosVelEffJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  m_root_nh = root_nh;
  m_controller_nh = controller_nh;
  m_hw = hw;
  
  m_controller_nh.setCallbackQueue(&m_queue);
  
  try{
    
    std::string setpoint_topic_name;
    if (!m_controller_nh.getParam("setpoint_topic_name", setpoint_topic_name))
    {
      ROS_FATAL_STREAM(m_controller_nh.getNamespace()+"/'setpoint_topic_name' does not exist");
      ROS_FATAL("ERROR DURING INITIALIZATION CONTROLLER '%s'", m_controller_nh.getNamespace().c_str());
      return false;
    }
    m_js_target_sub = m_controller_nh.subscribe<sensor_msgs::JointState>(setpoint_topic_name,    1, &itia::rutils::MsgReceiver<sensor_msgs::JointState>::callback, &m_target_js_rec);
    
    std::string external_torques_topic_name;
    if (!m_controller_nh.getParam("external_torques_topic_name", external_torques_topic_name))
    {
      ROS_FATAL_STREAM(m_controller_nh.getNamespace()+"/'external_torques_topic_name' does not exist");
      ROS_FATAL("ERROR DURING INITIALIZATION CONTROLLER '%s'", m_controller_nh.getNamespace().c_str());
      return false;
    }
    
    std::string adaptivity_topic_name;
    
    
    if (m_controller_nh.getParam("adaptivity_topic_name", adaptivity_topic_name))
    {
//      m_adaptivity_sub=m_root_nh.subscribe<fourbythree_msgs::StiffnessRecommendation>(adaptivity_topic_name,1,&itia::control::JointImpedanceControl::adaptivityCb,this);
      m_adaptivity_sub=m_root_nh.subscribe(adaptivity_topic_name,1,&itia::control::JointImpedanceControl::adaptivityCb,this);
    }
    
    m_effort_sub = m_controller_nh.subscribe<sensor_msgs::JointState>(external_torques_topic_name,    1, &itia::rutils::MsgReceiver<sensor_msgs::JointState>::callback, &m_effort_rec);
      
    m_joint_names=m_hw->getNames();
    m_nAx = m_joint_names.size();
    
    ROS_INFO("Controller '%s%s%s' controls the following joints:",GREEN,m_controller_nh.getNamespace().c_str(),RESET);
    for (unsigned int idx=0;idx<m_nAx;idx++)
      ROS_INFO(" - %s%s%s",GREEN,m_joint_names.at(idx).c_str(),RESET);
    
    
    
    m_target.resize(m_nAx);
    m_Dtarget.resize(m_nAx);
    m_adaptivity.resize(m_nAx);
    m_adaptivity_filtered.resize(m_nAx);
    m_adaptivity.setConstant(1.0);
    m_adaptivity_filtered.setConstant(1.0);
    m_x.resize(m_nAx);
    m_Dx.resize(m_nAx);
    m_DDx.resize(m_nAx);
    
    m_Jinv.resize(m_nAx);
    m_damping.resize(m_nAx);
    m_k_min.resize(m_nAx);
    m_k_max.resize(m_nAx);
    m_torque_deadband.resize(m_nAx);
    m_torque.resize(m_nAx);
    
    m_target.setZero();
    m_Dtarget.setZero();
    m_x.setZero();
    m_Dx.setZero();
    m_DDx.setZero();
    m_torque.setZero();
    
    std::vector<double> inertia, damping, stiffness_max, stiffness_min, torque_deadband;
    if (!m_controller_nh.getParam("inertia", inertia))
    {
      ROS_FATAL_STREAM(m_controller_nh.getNamespace()+"/'inertia' does not exist");
      ROS_FATAL("ERROR DURING INITIALIZATION CONTROLLER '%s'", m_controller_nh.getNamespace().c_str());
      return false;
    }
    if (!m_controller_nh.getParam("damping", damping))
    {
      ROS_FATAL_STREAM(m_controller_nh.getNamespace()+"/'damping' does not exist");
      ROS_FATAL("ERROR DURING INITIALIZATION CONTROLLER '%s'", m_controller_nh.getNamespace().c_str());
      return false;
    }
    if (!m_controller_nh.getParam("stiffness_max", stiffness_max))
    {
      ROS_FATAL_STREAM(m_controller_nh.getNamespace()+"/'stiffness_max' does not exist");
      ROS_FATAL("ERROR DURING INITIALIZATION CONTROLLER '%s'", m_controller_nh.getNamespace().c_str());
      return false;
    }
    if (!m_controller_nh.getParam("stiffness_min", stiffness_min))
    {
      ROS_FATAL_STREAM(m_controller_nh.getNamespace()+"/'stiffness_min' does not exist");
      ROS_FATAL("ERROR DURING INITIALIZATION CONTROLLER '%s'", m_controller_nh.getNamespace().c_str());
      return false;
    }
    if (!m_controller_nh.getParam("torque_deadband", torque_deadband))
    {
      ROS_FATAL_STREAM(m_controller_nh.getNamespace()+"/'torque_deadband' does not exist");
      ROS_FATAL("ERROR DURING INITIALIZATION CONTROLLER '%s'", m_controller_nh.getNamespace().c_str());
      return false;
    }
    
    if (!m_controller_nh.getParam("recommendation_filter", m_recomendation_filter))
    {
      ROS_WARN_STREAM(m_controller_nh.getNamespace()+"/'recommendation_filter' does not exist, set 0.9");
      m_recomendation_filter=0.99;
    }
    if (m_recomendation_filter>1)
    {
      ROS_WARN_STREAM(m_controller_nh.getNamespace()+"/'recommendation_filter' should be less than 1, set 1");
      m_recomendation_filter=1;
    }
     if (m_recomendation_filter<0)
    {
      ROS_WARN_STREAM(m_controller_nh.getNamespace()+"/'recommendation_filter' should be greater than 0, set 0");
      m_recomendation_filter=0;
    }
    
    for (unsigned int iAx=0;iAx<m_nAx;iAx++)
    {
      if (inertia.at(iAx)<=0)
      {
        ROS_INFO("inertia value of Joint %d is not positive, disabling impedance control for this axis",iAx);
        m_Jinv(iAx)=0.0;
      }
      else
        m_Jinv(iAx)=1.0/inertia.at(iAx);
      
      if (damping.at(iAx)<=0)
      {
        ROS_INFO("damping value of Joint %d is not positive, setting equalt to 10/inertia",iAx);
        m_damping(iAx)=10.0*m_Jinv(iAx);
      }
      else
        m_damping(iAx)=damping.at(iAx);
      
      
      if (stiffness_max.at(iAx)<0)
      {
        ROS_INFO("maximum fitness value of Joint %d is negative, setting equal to 0",iAx);
        m_k_max(iAx)=0.0;
      }
      else
        m_k_max(iAx)=stiffness_max.at(iAx);
      
      if (stiffness_min.at(iAx)<0)
      {
        ROS_INFO("minimum fitness value of Joint %d is negative, setting equal to 0",iAx);
        m_k_min(iAx)=0.0;
      }
      else if (stiffness_min.at(iAx)>stiffness_max.at(iAx))
      {
        ROS_INFO("minumum fitness value of Joint %d is greater than the maximum, setting equal to 'stiffness_max'",iAx);
        m_k_min(iAx)=stiffness_max.at(iAx);
      }
      else
        m_k_min(iAx)=stiffness_min.at(iAx);
      
      if (torque_deadband.at(iAx)<=0)
      {
        ROS_INFO("torque_deadband value of Joint %d is not positive, disabling impedance control for this axis",iAx);
        m_torque_deadband(iAx)=0.0;
      }
      else
        m_torque_deadband(iAx)=torque_deadband.at(iAx);
      
    }
    
  }
  catch(const  std::exception& e)
  {
    ROS_FATAL("EXCEPTION: %s", e.what());
    return false;
  }
  ROS_INFO("Controller '%s%s%s' well initialized",GREEN,m_controller_nh.getNamespace().c_str(),RESET);
  
  return true;
  
}

void JointImpedanceControl::starting(const ros::Time& time)
{
  m_is_configured=false;
  ROS_INFO("Controller '%s%s%s' well started",GREEN,m_controller_nh.getNamespace().c_str(),RESET);
}

void JointImpedanceControl::stopping(const ros::Time& time)
{
  ROS_INFO("[ %s%s%s ] Stopping controller", BOLDGREEN, m_controller_nh.getNamespace().c_str(), RESET);
  std::fill(m_hw->m_joint_target_msgs->effort.begin(),m_hw->m_joint_target_msgs->effort.end(),0.0);
  m_hw->m_joint_target_msgs->header.stamp = ros::Time::now();
}

void JointImpedanceControl::update(const ros::Time& time, const ros::Duration& period)
{
  m_queue.callAvailable();
  
  if (m_is_configured)
  {
    
    m_target=Eigen::VectorXd::Map(&(m_target_js_rec.getData().position.at(0)),m_target_js_rec.getData().position.size());
    m_Dtarget=Eigen::VectorXd::Map(&(m_target_js_rec.getData().velocity.at(0)),m_target_js_rec.getData().velocity.size());
    m_torque=Eigen::VectorXd::Map(&(m_effort_rec.getData().effort.at(0)),m_effort_rec.getData().effort.size());
    
    for (unsigned int iAx=0;iAx<m_nAx;iAx++)
    {
      if ( m_torque(iAx) > m_torque_deadband(iAx) )
        m_torque(iAx)-=m_torque_deadband(iAx);
      else if ( m_torque(iAx) < -m_torque_deadband(iAx) )
        m_torque(iAx)+=m_torque_deadband(iAx);
      else
        m_torque(iAx)=0.0;
    }
    
    m_adaptivity_filtered=m_recomendation_filter*m_adaptivity_filtered + (1-m_recomendation_filter)*m_adaptivity;
    
    //Eigen::VectorXd k=m_k_max.cwiseProduct(m_adaptivity_filtered)+m_k_min.cwiseProduct(1-m_adaptivity_filtered.array());
    Eigen::VectorXd k=m_k_min+ (m_k_max-m_k_min).cwiseProduct(m_adaptivity_filtered);
    
    m_DDx = m_Jinv.cwiseProduct( k.cwiseProduct(m_target-m_x) + m_damping.cwiseProduct(m_Dtarget-m_Dx) + m_torque );
    m_x  += m_Dx  * period.toSec() + m_DDx*std::pow(period.toSec(),2.0)*0.5;
    m_Dx += m_DDx * period.toSec();
    for (unsigned int iAx=0;iAx<m_nAx;iAx++)
    {
      m_hw->m_joint_target_msgs->position.at(iAx) = m_x(iAx);
      m_hw->m_joint_target_msgs->velocity.at(iAx) = m_Dx(iAx);
      m_hw->m_joint_target_msgs->effort.at(iAx)=0;
    }
    m_hw->m_joint_target_msgs->header.stamp=ros::Time::now();
    
  }
  else
  {
    m_is_configured=m_target_js_rec.isANewDataAvailable() && m_effort_rec.isANewDataAvailable();
    
    if (m_is_configured)
    {
      ROS_INFO("configured");
      m_x=Eigen::VectorXd::Map(&(m_target_js_rec.getData().position.at(0)),m_target_js_rec.getData().position.size());
      m_Dx=Eigen::VectorXd::Map(&(m_target_js_rec.getData().velocity.at(0)),m_target_js_rec.getData().velocity.size());
      
    }
  }
  
}
    
#ifdef FOURBYTHREE_MSGS
void JointImpedanceControl::adaptivityCb ( const fourbythree_msgs::StiffnessRecommendationConstPtr msg )
{
  if (msg->data.size()==3)
  {
    for (int iAx=0;iAx<3;iAx++)
    {
      if (msg->data.at(iAx)>100)
        m_adaptivity(iAx)=1;
      else if (msg->data.at(iAx)<0)
        m_adaptivity(iAx)=0;
      else
        m_adaptivity(iAx)=msg->data.at(iAx)*0.01;
      }
  }  
}
#else
void JointImpedanceControl::adaptivityCb ( const std_msgs::Float64MultiArrayConstPtr msg )
{
  if (msg->data.size()==3)
  {
    for (int iAx=0;iAx<3;iAx++)
    {
      if (msg->data.at(iAx)>100)
        m_adaptivity(iAx)=1;
      else if (msg->data.at(iAx)<0)
        m_adaptivity(iAx)=0;
      else
        m_adaptivity(iAx)=msg->data.at(iAx)*0.01;
    }
  }  
}
#endif


}
}
