
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

#include <itia_nodelet_hw_interface/nodelet_hw_interface.h>
#include <boost/graph/graph_concepts.hpp>

namespace itia
{
namespace control
{

JointTargetHW::JointTargetHW(const ros::Publisher& pub, const std::vector<std::string>& joint_names)
: 
m_joint_names(joint_names), 
m_pub(pub)
{
  
  m_n_dof = m_joint_names.size();
  
  m_pj_interface.m_joint_target_msgs.reset(new sensor_msgs::JointState());
  m_pj_interface.m_joint_target_msgs->position.resize(m_n_dof);
  m_pj_interface.m_joint_target_msgs->velocity.resize(m_n_dof);
  m_pj_interface.m_joint_target_msgs->effort.resize(m_n_dof);
  m_pj_interface.m_joint_target_msgs->name.resize(m_n_dof);
  m_pj_interface.m_joint_target_msgs->name = m_joint_names;
  m_pj_interface.m_joint_target_msgs->header.stamp=ros::Time(0);
  
  for (auto& elem: m_joint_names) 
  {
    auto i = &elem-&m_joint_names[0];
    
    hardware_interface::JointStateHandle state_handle(elem, 
                                                      &(m_pj_interface.m_joint_target_msgs->position.at(i)), 
                                                      &(m_pj_interface.m_joint_target_msgs->velocity.at(i)), 
                                                      &(m_pj_interface.m_joint_target_msgs->effort.at(i)));
    
    hardware_interface::JointHandle target_handle( state_handle, &(m_pj_interface.m_joint_target_msgs->position.at(i)) );
    m_js_interface.registerHandle( state_handle );
    m_pj_interface.registerHandle( target_handle );
  }

  registerInterface(&m_js_interface);
  registerInterface(&m_pj_interface);
  
}


void JointTargetHW::read()
{

}

void JointTargetHW::write()
{
	if (m_pj_interface.m_joint_target_msgs->header.stamp>ros::Time(0))
  {
		m_pub.publish(m_pj_interface.m_joint_target_msgs);
    
    m_pj_interface.m_joint_target_msgs.reset(new sensor_msgs::JointState());
    m_pj_interface.m_joint_target_msgs->position.resize(m_n_dof);
    m_pj_interface.m_joint_target_msgs->velocity.resize(m_n_dof);
    m_pj_interface.m_joint_target_msgs->effort.resize(m_n_dof);
    m_pj_interface.m_joint_target_msgs->name.resize(m_n_dof);
    m_pj_interface.m_joint_target_msgs->name = m_joint_names;
    m_pj_interface.m_joint_target_msgs->header.stamp=ros::Time(0);
  }
}

}
}