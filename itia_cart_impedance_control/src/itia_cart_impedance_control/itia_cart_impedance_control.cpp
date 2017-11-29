
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

#include <itia_cart_impedance_control/itia_cart_impedance_control.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(itia::control::CartImpedanceControl, controller_interface::ControllerBase);


namespace itia
{
  namespace control
  {
    
    
    CartImpedanceControl::~CartImpedanceControl()
    {
      
    }
    
    bool CartImpedanceControl::init(itia::control::PosVelEffJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
    {
      m_root_nh = root_nh;
      m_controller_nh = controller_nh;
      m_hw = hw;
      
      m_controller_nh.setCallbackQueue(&m_queue);
      std::string base_frame;
      std::string tool_frame;
      
      if (!m_controller_nh.getParam("base_frame", base_frame))
      {
        ROS_FATAL_STREAM_THROTTLE(0.1,m_controller_nh.getNamespace()+"/'base_frame' does not exist");
        ROS_FATAL("ERROR DURING INITIALIZATION CONTROLLER '%s'", m_controller_nh.getNamespace().c_str());
        return false;
      }
      if (!m_controller_nh.getParam("tool_frame", tool_frame))
      {
        ROS_FATAL_STREAM_THROTTLE(0.1,m_controller_nh.getNamespace()+"/'tool_frame' does not exist");
        ROS_FATAL("ERROR DURING INITIALIZATION CONTROLLER '%s'", m_controller_nh.getNamespace().c_str());
        return false;
      }
      
      bool base_is_reference=true;
      if (!m_controller_nh.getParam("base_is_reference", base_is_reference))
      {
        ROS_INFO("Using a base reference Cartesian impedance as default");
        base_is_reference=true;
      }
      if (!base_is_reference)
      {
        ROS_INFO("NOT IMPLEMENTED YET");
        base_is_reference=true;
      }
      
      urdf::Model model;
      if (!model.initParam("/robot_description"))
      {
        ROS_ERROR("Urdf robot_description '%s' does not exist",(m_controller_nh.getNamespace()+"/robot_description").c_str());
        return false;
      }
      grav << 0, 0, -9.806;
      m_joint_names=m_hw->getNames();
      m_nAx = m_joint_names.size();
      
      boost::shared_ptr<itia::dynamics::Link> root_link(new itia::dynamics::Link());  
      root_link->fromUrdf(model.root_link_);
      m_chain.reset(new itia::dynamics::Chain(root_link, base_frame,tool_frame, grav));
      m_chain->setInputJointsName(m_joint_names);
      
      try{
        
        std::string setpoint_topic_name;
        if (!m_controller_nh.getParam("setpoint_topic_name", setpoint_topic_name))
        {
          ROS_FATAL_STREAM_THROTTLE(0.1,m_controller_nh.getNamespace()+"/'setpoint_topic_name' does not exist");
          ROS_FATAL("ERROR DURING INITIALIZATION CONTROLLER '%s'", m_controller_nh.getNamespace().c_str());
          return false;
        }
        m_js_target_sub = m_controller_nh.subscribe<sensor_msgs::JointState>(setpoint_topic_name,    1, &itia::rutils::MsgReceiver<sensor_msgs::JointState>::callback, &m_target_js_rec);
        
        std::string external_wrench_topic_name;
        if (!m_controller_nh.getParam("external_wrench_topic_name", external_wrench_topic_name ))
        {
          ROS_FATAL_STREAM_THROTTLE(0.1,m_controller_nh.getNamespace()+"/'external_wrench_topic_name' does not exist");
          ROS_FATAL("ERROR DURING INITIALIZATION CONTROLLER '%s'", m_controller_nh.getNamespace().c_str());
          return false;
        }
        
        std::string adaptivity_topic_name;
        
        
        if (m_controller_nh.getParam("adaptivity_topic_name", adaptivity_topic_name))
        {
          //      m_adaptivity_sub=m_root_nh.subscribe<fourbythree_msgs::StiffnessRecommendation>(adaptivity_topic_name,1,&itia::control::CartImpedanceControl::adaptivityCb,this);
          m_adaptivity_sub=m_root_nh.subscribe(adaptivity_topic_name,1,&itia::control::CartImpedanceControl::adaptivityCb,this);
        }
        
        m_effort_sub = m_controller_nh.subscribe<geometry_msgs::WrenchStamped>( external_wrench_topic_name,    1, &itia::rutils::MsgReceiver<geometry_msgs::WrenchStamped>::callback, &m_effort_rec);
        
         
        ROS_INFO("Controller '%s%s%s' controls the following joints:",GREEN,m_controller_nh.getNamespace().c_str(),RESET);
        for (unsigned int idx=0;idx<m_nAx;idx++)
          ROS_INFO(" - %s%s%s",GREEN,m_joint_names.at(idx).c_str(),RESET);
        
        
        m_DDq_deadband.resize(m_nAx);
        m_target.resize(m_nAx);
        m_Dtarget.resize(m_nAx);
        m_adaptivity.resize(6);
        m_adaptivity_filtered.resize(6);
        m_adaptivity.setConstant(1.0);
        m_adaptivity_filtered.setConstant(1.0);
        m_x.resize(m_nAx);
        m_Dx.resize(m_nAx);
        m_DDx.resize(m_nAx);
        
        m_Jinv.resize(6);
        m_damping.resize(6);
        m_k_min.resize(6);
        m_k_max.resize(6);
        
        m_target.setZero();
        m_Dtarget.setZero();
        m_x.setZero();
        m_Dx.setZero();
        m_DDx.setZero();
        m_wrench.setZero();
        
        std::vector<double> inertia, damping, stiffness_max, stiffness_min, wrench_deadband, DDq_deadband;
        if (!m_controller_nh.getParam("inertia", inertia))
        {
          ROS_FATAL_STREAM_THROTTLE(0.1,m_controller_nh.getNamespace()+"/'inertia' does not exist");
          ROS_FATAL("ERROR DURING INITIALIZATION CONTROLLER '%s'", m_controller_nh.getNamespace().c_str());
          return false;
        }
        if (!m_controller_nh.getParam("damping", damping))
        {
          ROS_FATAL_STREAM_THROTTLE(0.1,m_controller_nh.getNamespace()+"/'damping' does not exist");
          ROS_FATAL("ERROR DURING INITIALIZATION CONTROLLER '%s'", m_controller_nh.getNamespace().c_str());
          return false;
        }
        if (!m_controller_nh.getParam("stiffness_max", stiffness_max))
        {
          ROS_FATAL_STREAM_THROTTLE(0.1,m_controller_nh.getNamespace()+"/'stiffness_max' does not exist");
          ROS_FATAL("ERROR DURING INITIALIZATION CONTROLLER '%s'", m_controller_nh.getNamespace().c_str());
          return false;
        }
        if (!m_controller_nh.getParam("stiffness_min", stiffness_min))
        {
          ROS_FATAL_STREAM_THROTTLE(0.1,m_controller_nh.getNamespace()+"/'stiffness_min' does not exist");
          ROS_FATAL("ERROR DURING INITIALIZATION CONTROLLER '%s'", m_controller_nh.getNamespace().c_str());
          return false;
        }
        if (!m_controller_nh.getParam("wrench_deadband", wrench_deadband))
        {
          ROS_FATAL_STREAM_THROTTLE(0.1,m_controller_nh.getNamespace()+"/'torque_deadband' does not exist");
          ROS_FATAL("ERROR DURING INITIALIZATION CONTROLLER '%s'", m_controller_nh.getNamespace().c_str());
          return false;
        }
        if (!m_controller_nh.getParam("joint_acc_deadband", DDq_deadband))
        {
          ROS_FATAL_STREAM_THROTTLE(0.1,m_controller_nh.getNamespace()+"/'joint_acc_deadband' does not exist");
          ROS_FATAL("ERROR DURING INITIALIZATION CONTROLLER '%s'", m_controller_nh.getNamespace().c_str());
          return false;
        }
        if (DDq_deadband.size()!=m_nAx)
        {
          ROS_FATAL_STREAM_THROTTLE(0.1,m_controller_nh.getNamespace()+"/'joint_acc_deadband' has wrong dimension");
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

          m_DDq_deadband(iAx)=DDq_deadband.at(iAx);
        }
        
        for (unsigned int iAx=0;iAx<6;iAx++)
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
          
          if (wrench_deadband.at(iAx)<=0)
          {
            ROS_INFO("wrench_deadband %d is not positive, disabling impedance control for this axis",iAx);
            m_wrench_deadband(iAx)=0.0;
          }
          else
            m_wrench_deadband(iAx)=wrench_deadband.at(iAx);
          
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
    
    void CartImpedanceControl::starting(const ros::Time& time)
    {
      m_is_configured=false;
      m_is_wrench_received=false;
      ROS_INFO("Controller '%s%s%s' well started",GREEN,m_controller_nh.getNamespace().c_str(),RESET);
    }
    
    void CartImpedanceControl::stopping(const ros::Time& time)
    {
      ROS_INFO("[ %s%s%s ] Stopping controller", BOLDGREEN, m_controller_nh.getNamespace().c_str(), RESET);
      std::fill(m_hw->m_joint_target_msgs->effort.begin(),m_hw->m_joint_target_msgs->effort.end(),0.0);
      m_hw->m_joint_target_msgs->header.stamp = ros::Time::now();
    }
    
    void CartImpedanceControl::update(const ros::Time& time, const ros::Duration& period)
    {
      m_queue.callAvailable();
      
      if (m_is_configured)
      {
        
        m_target=Eigen::VectorXd::Map(&(m_target_js_rec.getData().position.at(0)),m_target_js_rec.getData().position.size());
        m_Dtarget=Eigen::VectorXd::Map(&(m_target_js_rec.getData().velocity.at(0)),m_target_js_rec.getData().velocity.size());
        
        if (m_is_wrench_received)
        {
          m_wrench(0)=m_effort_rec.getData().wrench.force.x;
          m_wrench(1)=m_effort_rec.getData().wrench.force.y;
          m_wrench(2)=m_effort_rec.getData().wrench.force.z;
          m_wrench(3)=m_effort_rec.getData().wrench.torque.x;
          m_wrench(4)=m_effort_rec.getData().wrench.torque.y;
          m_wrench(5)=m_effort_rec.getData().wrench.torque.z;
        }
        else
        {
          m_is_wrench_received= m_effort_rec.isANewDataAvailable();
        }
        
        for (unsigned int iAx=0;iAx<6;iAx++)
        {
          if ( m_wrench(iAx) > m_wrench_deadband(iAx) )
            m_wrench(iAx)-=m_wrench_deadband(iAx);
          else if ( m_wrench(iAx) < -m_wrench_deadband(iAx) )
            m_wrench(iAx)+=m_wrench_deadband(iAx);
          else
            m_wrench(iAx)=0.0;
        }
        m_adaptivity_filtered=m_recomendation_filter*m_adaptivity_filtered + (1-m_recomendation_filter)*m_adaptivity;
        
        //Eigen::VectorXd k=m_k_max.cwiseProduct(m_adaptivity_filtered)+m_k_min.cwiseProduct(1-m_adaptivity_filtered.array());
        Eigen::VectorXd k=m_k_min+ (m_k_max-m_k_min).cwiseProduct(m_adaptivity_filtered);

        Eigen::Affine3d cart_target_pose = m_chain->getTransformation(m_target);
        Eigen::Vector6d cart_target_vel  = m_chain->getJacobian(m_target)*m_Dtarget;
        Eigen::Affine3d cart_pose = m_chain->getTransformation(m_x);
        
        Eigen::Vector6d wrench_in_b=itia::dynamics::spatialRotation(m_wrench,cart_pose.rotation());
        
        Eigen::VectorXd cartesian_error;
        itia::mutils::getFrameDistance(cart_target_pose, cart_pose , &cartesian_error);
        cartesian_error.block(3,0,3,1)=-cartesian_error.block(3,0,3,1);
        
        Eigen::Matrix6Xd J  = m_chain->getJacobian(m_x);
        Eigen::Vector6d cart_vel  = J*m_Dx;
        Eigen::Vector6d cart_DT_nl  = m_chain->getDTwistNonLinearPart(m_x,m_Dx).back(); // DJ*Dq
        
        Eigen::Vector6d cart_DT = m_Jinv.cwiseProduct(k.cwiseProduct(cartesian_error) + m_damping.cwiseProduct(cart_target_vel-cart_vel) + wrench_in_b );
        Eigen::FullPivLU<Eigen::MatrixXd> pinv_J(J);
        pinv_J.setThreshold(1e-2);
        m_DDx=pinv_J.solve(cart_DT-cart_DT_nl);
        if (pinv_J.rank()<6)
        {
          Eigen::MatrixXd null=pinv_J.kernel();
          ROS_WARN_THROTTLE(0.1,"Singolarity point!");
          
          for (int iC=0;iC<null.cols();iC++)
          {
            Eigen::VectorXd null_versor=null.col(iC);
            null_versor.normalize();
            m_DDx=m_DDx-(m_DDx.dot(null_versor))*null_versor;
          }
        }  
        
        
//         double scale= (m_DDx.array()/m_DDq_deadband.array()).abs().maxCoeff();
//         if (scale>1)
//         {
//           m_DDx=m_DDx/scale;
//         }
        
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
        m_is_configured=m_target_js_rec.isANewDataAvailable();
        m_is_wrench_received= m_effort_rec.isANewDataAvailable();
        if (m_is_configured)
        {
          ROS_INFO("configured");
          m_x=Eigen::VectorXd::Map(&(m_target_js_rec.getData().position.at(0)),m_target_js_rec.getData().position.size());
          m_Dx=Eigen::VectorXd::Map(&(m_target_js_rec.getData().velocity.at(0)),m_target_js_rec.getData().velocity.size());
          
        }
      }
      
    }
    
    #ifdef FOURBYTHREE_MSGS
    void CartImpedanceControl::adaptivityCb ( const fourbythree_msgs::StiffnessRecommendationConstPtr msg )
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
    void CartImpedanceControl::adaptivityCb ( const std_msgs::Float64MultiArrayConstPtr msg )
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
