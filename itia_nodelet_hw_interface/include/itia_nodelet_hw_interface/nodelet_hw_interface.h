
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

#ifndef __NODELET_HARDWARE_INTERFACE__
#define __NODELET_HARDWARE_INTERFACE__

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
# include <sensor_msgs/JointState.h>
# include <ros/ros.h>
# include <itia_futils/itia_futils.h>

extern ros::Publisher pub;

namespace itia
{
  
namespace control
{

class PosVelEffJointInterface : public hardware_interface::JointCommandInterface 
{
public:
  sensor_msgs::JointStatePtr m_joint_target_msgs;
};


class JointTargetHW : public hardware_interface::RobotHW
{
public:
  JointTargetHW(const ros::Publisher& pub, const std::vector<std::string>& joint_names);

  void read();
  void write();

protected:
  ros::Publisher m_pub;
private:
  hardware_interface::JointStateInterface  m_js_interface;
  itia::control::PosVelEffJointInterface m_pj_interface;
   
  
  unsigned int m_n_dof;
  std::vector<std::string> m_joint_names;
  
};

}
}

#endif
