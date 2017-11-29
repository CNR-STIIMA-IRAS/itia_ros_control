
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