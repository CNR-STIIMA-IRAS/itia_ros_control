
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

#include <itia_cascade_control/itia_cascade_control_topic_interface.h>

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(itia::control::TopicCascadeControl, controller_interface::ControllerBase);


namespace itia
{
namespace control
{

bool TopicCascadeControl::init ( PosVelEffJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh )
{
  m_hw=hw;
  std::vector<std::string> joint_names=m_hw->getNames();
  m_cascade_controller.reset(new itia::control::CascadeControl(root_nh,controller_nh,joint_names));
  return  m_cascade_controller->isWellInit();
}
void TopicCascadeControl::starting ( const ros::Time& time )
{
  m_cascade_controller->starting(time);
}
void TopicCascadeControl::stopping ( const ros::Time& time )
{
  m_cascade_controller->stopping(time);
  m_hw->m_joint_target_msgs->position = m_cascade_controller->getPosCmd();
  m_hw->m_joint_target_msgs->velocity = m_cascade_controller->getVelCmd();
  m_hw->m_joint_target_msgs->effort   = m_cascade_controller->getEffCmd();
  m_hw->m_joint_target_msgs->header.stamp = ros::Time::now();
}
void TopicCascadeControl::update ( const ros::Time& time, const ros::Duration& period )
{
  m_cascade_controller->update(time,period);
  m_hw->m_joint_target_msgs->position = m_cascade_controller->getPosCmd();
  m_hw->m_joint_target_msgs->velocity = m_cascade_controller->getVelCmd();
  m_hw->m_joint_target_msgs->effort   = m_cascade_controller->getEffCmd();
  m_hw->m_joint_target_msgs->header.stamp = ros::Time::now();
}



}
}
