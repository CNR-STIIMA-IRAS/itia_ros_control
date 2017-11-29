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