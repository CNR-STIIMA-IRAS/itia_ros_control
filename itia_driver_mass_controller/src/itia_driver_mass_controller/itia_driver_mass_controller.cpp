
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

#include <itia_driver_mass_controller/itia_driver_mass_controller.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(itia::control::DrivenMassController, controller_interface::ControllerBase);


namespace itia
{
  namespace control
  {
    
    
    DrivenMassController::~DrivenMassController()
    {
    }
    
    bool DrivenMassController::init(itia::control::PosVelEffJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
    {
      m_root_nh = root_nh;
      m_controller_nh = controller_nh;
      m_hw = hw;
      m_controller_nh.setCallbackQueue(&m_queue);
      
      std::string feedback_topic_name;
      std::string setpoint_topic_name;
      std::string published_link_topic_name;
      std::string feedforward_topic_name;
      
      
      
      if (!m_controller_nh.getParam("feedback_topic_name", feedback_topic_name))
      {
        ROS_FATAL_STREAM(m_controller_nh.getNamespace()+"/'feedback_topic_name' does not exist");
        ROS_FATAL("ERROR DURING INITIALIZATION CONTROLLER '%s'", m_controller_nh.getNamespace().c_str());
        return false;
      }
      if (!m_controller_nh.getParam("setpoint_topic_name", setpoint_topic_name))
      {
        ROS_FATAL_STREAM(m_controller_nh.getNamespace()+"/'setpoint_topic_name' does not exist");
        ROS_FATAL("ERROR DURING INITIALIZATION CONTROLLER '%s'", m_controller_nh.getNamespace().c_str());
        return false;
      }
      if (!m_controller_nh.getParam("feedforward_topic_name", feedforward_topic_name))
      {
        ROS_FATAL_STREAM(m_controller_nh.getNamespace()+"/'feedforward_topic_name' does not exist");
        ROS_FATAL("ERROR DURING INITIALIZATION CONTROLLER '%s'", m_controller_nh.getNamespace().c_str());
        return false;
      }
      
      
      
      m_js_fb_sub     = m_controller_nh.subscribe<sensor_msgs::JointState>(feedback_topic_name, 1, &itia::rutils::MsgReceiver<sensor_msgs::JointState>::callback, &m_fb_js_rec);
      m_js_target_sub = m_controller_nh.subscribe<sensor_msgs::JointState>(setpoint_topic_name, 1, &itia::rutils::MsgReceiver<sensor_msgs::JointState>::callback, &m_target_js_rec);
      m_js_ffw_sub    = m_controller_nh.subscribe<sensor_msgs::JointState>(feedforward_topic_name, 1, &itia::rutils::MsgReceiver<sensor_msgs::JointState>::callback, &m_ffw_js_rec);
      
      m_joint_names=m_hw->getNames();
      m_nAx = m_joint_names.size();
      
      if (m_controller_nh.getParam("published_link_topic_name", published_link_topic_name))
      {
        ROS_INFO("Controller '%s%s%s' is going to publish the following topic: %s",GREEN,m_controller_nh.getNamespace().c_str(),RESET, published_link_topic_name.c_str());
        m_publishing_link_topic=true;
        m_link_publisher=m_root_nh.advertise<sensor_msgs::JointState>(published_link_topic_name,1);
        m_link_msg.reset(new sensor_msgs::JointState());
        m_link_msg->position.resize(m_nAx);
        m_link_msg->velocity.resize(m_nAx);
        m_link_msg->effort.resize(m_nAx);
      }
      else
        m_publishing_link_topic=false;
      
      ROS_DEBUG("Controller '%s%s%s' controls the following joints:",GREEN,m_controller_nh.getNamespace().c_str(),RESET);
      for (unsigned int idx=0;idx<m_nAx;idx++)
        ROS_DEBUG(" - %s%s%s",GREEN,m_joint_names.at(idx).c_str(),RESET);
      
      m_link_pos_filters.resize(m_nAx);
      m_link_vel_filters.resize(m_nAx);
      m_target_pos_filters.resize(m_nAx);
      m_target_vel_filters.resize(m_nAx);
      m_controllers.resize(m_nAx);
      m_minimum_error.resize(m_nAx);
      m_target_motor_position.resize(m_nAx);
      
      ros::WallTime t0=ros::WallTime::now();
      while (!m_fb_js_rec.isANewDataAvailable())
      {
        m_queue.callAvailable();
        if ( (ros::WallTime::now()-t0).toSec()>20 )
        {
          ROS_FATAL_STREAM(m_controller_nh.getNamespace()+" feedback_topic not received");
          ROS_FATAL("ERROR DURING INITIALIZATION CONTROLLER '%s'", m_controller_nh.getNamespace().c_str());
          return false;
        }
      }
      
      std::vector<double> elastic_joint_positions = m_fb_js_rec.getData().position;
      if ( elastic_joint_positions.size() != 2*m_nAx)
      {
        ROS_FATAL("elastic joint states topic has wrong dimensions (%zu, instead of %d)",elastic_joint_positions.size(), 2*m_nAx);
        ROS_FATAL("ERROR DURING INITIALIZATION CONTROLLER '%s'", m_controller_nh.getNamespace().c_str());
        return false;
      }
      
      for (int i = 0;i<m_nAx;i++)
      {
        
        ROS_DEBUG("[ %s%s%s ] Creating controller objects", BOLDGREEN, m_controller_nh.getNamespace().c_str(), RESET);
        
        Eigen::VectorXd init_cond(1);
        init_cond(0)=0;
        std::string name;
      
        double tmp;
        if (!m_controller_nh.getParam(m_joint_names.at(i)+"/position_minimum_error",tmp))
        {
          ROS_WARN("no torque_minimum_error specified for joint %s, set equal to zero",m_joint_names.at(i).c_str());
          tmp=0;
        }
        m_minimum_error.at(i)=tmp;
        
        name = m_joint_names.at(i)+"/controller";
        m_controllers.at(i)=itia::control::createDiscreteStateSpareFromParam(m_controller_nh, name,init_cond);
        
        name = m_joint_names.at(i)+"/target_filter";
        m_target_pos_filters.at(i)=itia::control::createDiscreteStateSpareFromParam(m_controller_nh, name,init_cond);
        m_target_vel_filters.at(i)=itia::control::createDiscreteStateSpareFromParam(m_controller_nh, name,init_cond);
        
        name = m_joint_names.at(i)+"/link_pos_filter";
        m_link_pos_filters.at(i)=itia::control::createDiscreteStateSpareFromParam(m_controller_nh, name,init_cond);
        
        name = m_joint_names.at(i)+"/link_vel_filter";
        m_link_vel_filters.at(i)=itia::control::createDiscreteStateSpareFromParam(m_controller_nh, name,init_cond);
        
      }
      
      ROS_INFO("Controller '%s%s%s' well initialized",GREEN,m_controller_nh.getNamespace().c_str(),RESET);
      return true;
      
    }
    
    void DrivenMassController::starting(const ros::Time& time)
    {
      ROS_DEBUG("starting Controller '%s%s%s'",GREEN,m_controller_nh.getNamespace().c_str(),RESET);
      m_configured = false;       
      m_feedforward_active=false;
      
      m_queue.callAvailable();
      m_configured = m_configured || (m_target_js_rec.isANewDataAvailable());
      
      std::vector<double> elastic_joint_positions  = m_fb_js_rec.getData().position;
      std::vector<double> elastic_joint_velocities = m_fb_js_rec.getData().velocity;
      
      for (int i = 0;i<m_nAx;i++)
      {
        Eigen::VectorXd init_motor_pos(1);
        init_motor_pos (0) = elastic_joint_positions.at(2*i);
        
        Eigen::VectorXd initial_state;
        
        
        if (m_hw->m_joint_target_msgs->header.stamp>ros::Time(0))
          m_target_motor_position.at(i)=m_hw->m_joint_target_msgs->position.at(i);
        else
          m_target_motor_position.at(i)=elastic_joint_positions.at(2*i);
        
        Eigen::VectorXd init_pos(1);
        init_pos(0)=elastic_joint_positions.at(2*i)+elastic_joint_positions.at(2*i+1);
        initial_state = m_link_pos_filters.at(i)->getInitializationMatrix()*init_pos;
        m_link_pos_filters.at(i)->setState(initial_state);
        
        Eigen::VectorXd init_target_pos(1);
        if (m_configured)
           init_target_pos(0)=m_target_js_rec.getData().position.at(i);
        else
          init_target_pos=init_pos;
        
        initial_state = m_target_pos_filters.at(i)->getInitializationMatrix()*init_target_pos;
        m_target_pos_filters.at(i)->setState(initial_state);
        
        Eigen::VectorXd init_vel(1);
        init_vel(0)=elastic_joint_velocities.at(2*i)+elastic_joint_velocities.at(2*i+1);
        initial_state = m_link_vel_filters.at(i)->getInitializationMatrix()*init_vel;
        m_link_vel_filters.at(i)->setState(initial_state);
        
        Eigen::VectorXd init_target_vel(1);
        if (m_configured)
           init_target_vel(0)=m_target_js_rec.getData().velocity.at(i);
        else
          init_target_vel=init_vel;
        
        initial_state = m_target_vel_filters.at(i)->getInitializationMatrix()*init_target_vel;
        m_target_vel_filters.at(i)->setState(initial_state);
        
        Eigen::VectorXd cmd_vel(1);
        if (m_hw->m_joint_target_msgs->header.stamp>ros::Time(0))
        {
          cmd_vel(0)=m_hw->m_joint_target_msgs->velocity.at(i);
        }
        else
          cmd_vel(0)=elastic_joint_velocities.at(2*i);
        
        Eigen::VectorXd controller_in(2);
        controller_in(0)=init_target_pos(0)-init_pos(0);
        controller_in(1)=0;
        if (std::abs( controller_in(0))<m_minimum_error.at(i))
          controller_in(0)=0;
        
        initial_state = m_controllers.at(i)->getInitializationMatrix()*(cmd_vel-init_target_vel-(m_controllers.at(i)->getDMatrix())*controller_in );
        m_controllers.at(i)->setState(initial_state);

        
      }
      ROS_INFO("Controller '%s%s%s' well started",GREEN,m_controller_nh.getNamespace().c_str(),RESET);
    }
    
    void DrivenMassController::stopping(const ros::Time& time)
    {
      ROS_INFO("[ %s%s%s ] Stopping controller", BOLDGREEN, m_controller_nh.getNamespace().c_str(), RESET);
      m_configured = false;
      m_feedforward_active=false;
      for (int iAx = 0;iAx<m_nAx;iAx++)
      {
//         m_hw->m_joint_target_msgs->position.at(iAx) = m_fb_js_rec.getData().position.at(2*iAx);
//         m_hw->m_joint_target_msgs->velocity.at(iAx) = 0;
      }
      m_hw->m_joint_target_msgs->header.stamp = ros::Time::now();
    }
    
    void DrivenMassController::update(const ros::Time& time, const ros::Duration& period)
    {
      m_queue.callAvailable();
      
      if (m_publishing_link_topic)
      {
        for (int iAx = 0;iAx<m_nAx;iAx++)
        {
          m_link_msg->position.at(iAx) = m_fb_js_rec.getData().position.at(2*iAx) + m_fb_js_rec.getData().position.at(2*iAx+1);
          m_link_msg->velocity.at(iAx) = m_fb_js_rec.getData().velocity.at(2*iAx) + m_fb_js_rec.getData().velocity.at(2*iAx+1);
          m_link_msg->effort.at(iAx)   = m_fb_js_rec.getData().effort.at(2*iAx)   + m_fb_js_rec.getData().effort.at(2*iAx+1);
        }
        m_link_msg->header.stamp=m_fb_js_rec.getData().header.stamp;
        m_link_publisher.publish(m_link_msg);
      }
      
      m_configured = m_configured || (m_target_js_rec.isANewDataAvailable());
      m_feedforward_active=m_feedforward_active || (m_ffw_js_rec.isANewDataAvailable());
      
      
      // CONTROLLER INPUT   DEADZONE-POS ERROR, VEL ERROR
      // CONTROLLER OUTPUT  MOTOR VEL
      Eigen::VectorXd controller_in(2); 
      Eigen::VectorXd controller_out(1);
      
      
      Eigen::VectorXd target_pos_filter_in(1); 
      Eigen::VectorXd target_pos_filter_out(1); 
      
      Eigen::VectorXd target_vel_filter_in(1);
      Eigen::VectorXd target_vel_filter_out(1);
      
      Eigen::VectorXd link_pos_filter_in(1);
      Eigen::VectorXd link_pos_filter_out(1);
      
      Eigen::VectorXd link_vel_filter_in(1);
      Eigen::VectorXd link_vel_filter_out(1);
      
      
      
      for (int iAx = 0;iAx<m_nAx;iAx++)
      {
        
        link_pos_filter_in(0)= m_fb_js_rec.getData().position.at(2*iAx)+m_fb_js_rec.getData().position.at(2*iAx+1);
        link_vel_filter_in(0)= m_fb_js_rec.getData().velocity.at(2*iAx)+m_fb_js_rec.getData().velocity.at(2*iAx+1);
        
        link_pos_filter_out=m_link_pos_filters.at(iAx)->update(link_pos_filter_in);
        link_vel_filter_out=m_link_vel_filters.at(iAx)->update(link_vel_filter_in);

        
        if (m_configured)
        {
          target_pos_filter_in (0)=m_target_js_rec.getData().position.at(iAx);
          target_vel_filter_in (0)=m_target_js_rec.getData().velocity.at(iAx);
        }
        else
        {
          target_pos_filter_in (0)=link_pos_filter_out(0);
          target_vel_filter_in (0)=link_vel_filter_out(0);
        }
        target_pos_filter_out= m_target_pos_filters.at(iAx)->update( target_pos_filter_in );
        target_vel_filter_out= m_target_vel_filters.at(iAx)->update( target_vel_filter_in );
        
        controller_in (0) = target_pos_filter_out(0)-link_pos_filter_out(0);
        controller_in (1) = target_vel_filter_out(0)-link_vel_filter_out(0);
        
        if ( (std::abs( controller_in(0))<m_minimum_error.at(iAx)) && ( std::abs(target_vel_filter_out(0))<1e-2) )
          controller_in(0)=0;
        
        controller_out=m_controllers.at(iAx)->update( controller_in )+target_vel_filter_out;
        
//         if ( (std::abs( controller_in(0))<m_minimum_error.at(iAx)) && ( std::abs(target_vel_filter_out(0))<1e-2) )
//         {
//           controller_out(0)=0;
//         }
//         ROS_INFO("cin(0)=%f,min_err=%f,\t,\ttarget vel=%f",controller_in(0),m_minimum_error.at(iAx),target_vel_filter_out(0));
        //m_target_motor_position.at(iAx) += period.toSec()*controller_out(0));
        double position_adjustment=m_fb_js_rec.getData().position.at(2*iAx)-m_target_motor_position.at(iAx); //with inner loops are saturated
        
        if (position_adjustment>10*m_minimum_error.at(iAx))
          position_adjustment-=10*m_minimum_error.at(iAx);
        else if (position_adjustment<-10*m_minimum_error.at(iAx))
          position_adjustment+=10*m_minimum_error.at(iAx);
        else
          position_adjustment=0.0;
        
        m_target_motor_position.at(iAx) += period.toSec()*(controller_out(0)+2*position_adjustment);
        m_hw->m_joint_target_msgs->position.at(iAx) = m_target_motor_position.at(iAx);
        m_hw->m_joint_target_msgs->velocity.at(iAx) = controller_out(0);
        m_hw->m_joint_target_msgs->effort.at(iAx)   = 0.0; //TODO ADD TORQUE FFW
        

      }
      m_hw->m_joint_target_msgs->header.stamp = ros::Time::now();
      
      
    }
    
    
  }
}
