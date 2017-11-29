
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

#include <itia_velocity_control/itia_velocity_control_on_velocity_interface.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(itia::control::VelocityInterfaceVelocityControl, controller_interface::ControllerBase);


namespace itia
{
  namespace control
  {
    
    bool VelocityInterfaceVelocityControl::init ( hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh )
    {
      m_hw=hw;
      std::vector<std::string> joint_names=m_hw->getNames();
      m_velocity_controller.reset(new itia::control::VelocityControl(root_nh,controller_nh,joint_names));
      if (!m_velocity_controller->isWellInit())
        return false;
      for (int i=0;i<joint_names.size();i++)
        m_jh.push_back(m_hw->getHandle(joint_names.at(i)));
      
      return true;
    }
    void VelocityInterfaceVelocityControl::starting ( const ros::Time& time )
    {
      m_velocity_controller->starting(time);
    }
    void VelocityInterfaceVelocityControl::stopping ( const ros::Time& time )
    {
      m_velocity_controller->stopping(time);
    }
    void VelocityInterfaceVelocityControl::update ( const ros::Time& time, const ros::Duration& period )
    {
      m_velocity_controller->update(time,period);
      
      std::vector<double> velocity=m_velocity_controller->getVelCmd();
      for (unsigned int i=0;i<m_jh.size();i++)
      {
//         ROS_INFO("vel(%u)=%f",i,velocity.at(i));
//         m_jh.at(i).setCommand(0.0);
        m_jh.at(i).setCommand(velocity.at(i));
        
      }
    }
    
  }
}
