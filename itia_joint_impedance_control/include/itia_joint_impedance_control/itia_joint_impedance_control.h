
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

#ifndef __itia_joint_impedance_sensor__
#define __itia_joint_impedance_sensor__

# include <controller_interface/controller.h>
# include <hardware_interface/joint_command_interface.h>
# include <itia_nodelet_hw_interface/nodelet_hw_interface.h>
# include <itia_controllers_and_filters/discrete_state_space.h>
# include <itia_rutils/itia_rutils.h>
# include <itia_dynamics_core/itia_primitives.h>
# include <thread>
# include <mutex>
# include <boost/graph/graph_concepts.hpp>
# include <ros/ros.h>
# include <sensor_msgs/JointState.h>
# include <pluginlib/class_list_macros.h>
# include <geometry_msgs/WrenchStamped.h>


# ifdef FOURBYTHREE_MSGS
#  pragma message ( "Using fourbythree messages!" )
#  include <fourbythree_msgs/StiffnessRecommendation.h>
# else
#  pragma message ( "Using standard messages!" )
#  include <std_msgs/Float64MultiArray.h>
# endif


namespace itia
{
  namespace control
  {
    
    class JointImpedanceControl : public controller_interface::Controller<itia::control::PosVelEffJointInterface>
    {
    public:
      bool init(itia::control::PosVelEffJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
      void update(const ros::Time& time, const ros::Duration& period);
      void starting(const ros::Time& time);
      void stopping(const ros::Time& time);
      
    protected:
      
      ros::CallbackQueue m_queue;
      boost::shared_ptr<ros::AsyncSpinner> m_spinner;
      bool m_is_configured=false;
      std::vector<hardware_interface::JointHandle> m_joint_handles;
      
      Eigen::VectorXd m_target;
      Eigen::VectorXd m_Dtarget;
      
      Eigen::VectorXd m_x;
      Eigen::VectorXd m_Dx;
      Eigen::VectorXd m_DDx;
      Eigen::VectorXd m_Jinv;
      Eigen::VectorXd m_damping;
      Eigen::VectorXd m_k_max;
      Eigen::VectorXd m_k_min;
      Eigen::VectorXd m_torque_deadband;
      
      Eigen::VectorXd m_torque;
      
      double m_recomendation_filter;
      Eigen::VectorXd m_adaptivity;
      Eigen::VectorXd m_adaptivity_filtered;
      
      
      itia::control::PosVelEffJointInterface* m_hw;
      
      itia::rutils::MsgReceiver<sensor_msgs::JointState> m_target_js_rec;
      ros::Subscriber  m_js_target_sub;
      ros::Subscriber  m_adaptivity_sub;
      
      
      itia::rutils::MsgReceiver<sensor_msgs::JointState> m_effort_rec;
      ros::Subscriber  m_effort_sub;
      ros::NodeHandle m_root_nh;
      ros::NodeHandle m_controller_nh;
      
      std::vector< std::string > m_joint_names;
      int m_nAx;
      
      ~JointImpedanceControl();
      
#ifdef FOURBYTHREE_MSGS
      void adaptivityCb(const fourbythree_msgs::StiffnessRecommendationConstPtr msg);
#else
      void adaptivityCb(const std_msgs::Float64MultiArrayConstPtr msg);
#endif
      
    };
    
    
  }
}


#endif
