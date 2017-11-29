#include <itia_velocity_control/itia_velocity_control.h>
#include <pluginlib/class_list_macros.h>


namespace itia
{
  namespace control
  {
    
    
    VelocityControl::VelocityControl(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh, std::vector<std::string> joint_names)
    {
      
      m_root_nh = root_nh;
      m_controller_nh = controller_nh;
      m_well_init=false;
      m_controller_nh.setCallbackQueue(&m_queue);
      
      std::string feedback_topic_name;
      std::string setpoint_topic_name;
      std::string feedforward_topic_name;
      
      if (!m_controller_nh.getParam("feedback_topic_name", feedback_topic_name))
      {
        ROS_FATAL_STREAM(m_controller_nh.getNamespace()+"/'feedback_topic_name' does not exist");
        ROS_FATAL("ERROR DURING INITIALIZATION CONTROLLER '%s'", m_controller_nh.getNamespace().c_str());
        return;
      }
      if (!m_controller_nh.getParam("setpoint_topic_name", setpoint_topic_name))
      {
        ROS_FATAL_STREAM(m_controller_nh.getNamespace()+"/'setpoint_topic_name' does not exist");
        ROS_FATAL("ERROR DURING INITIALIZATION CONTROLLER '%s'", m_controller_nh.getNamespace().c_str());
        return;
      }
      if (!m_controller_nh.getParam("feedforward_topic_name", feedforward_topic_name))
      {
        ROS_FATAL_STREAM(m_controller_nh.getNamespace()+"/'feedforward_topic_name' does not exist");
        ROS_FATAL("ERROR DURING INITIALIZATION CONTROLLER '%s'", m_controller_nh.getNamespace().c_str());
        return;
      }
      
      m_js_fb_sub     = m_controller_nh.subscribe<sensor_msgs::JointState>(feedback_topic_name,    1, &itia::rutils::MsgReceiver<sensor_msgs::JointState>::callback, &m_fb_js_rec);
      m_js_target_sub = m_controller_nh.subscribe<sensor_msgs::JointState>(setpoint_topic_name,    1, &itia::rutils::MsgReceiver<sensor_msgs::JointState>::callback, &m_target_js_rec);
      m_js_ffw_sub    = m_controller_nh.subscribe<sensor_msgs::JointState>(feedforward_topic_name, 1, &itia::rutils::MsgReceiver<sensor_msgs::JointState>::callback, &m_ffw_js_rec);
      
      m_joint_names=joint_names;
      
      
      
      m_nAx = m_joint_names.size();
      ROS_INFO("Controller '%s%s%s' controls the following joints:",GREEN,m_controller_nh.getNamespace().c_str(),RESET);
      for (unsigned int idx=0;idx<m_nAx;idx++)
        ROS_INFO(" - %s%s%s",GREEN,m_joint_names.at(idx).c_str(),RESET);
      
      m_pos_controllers.resize(m_nAx);
      
      m_pos_filters.resize(m_nAx);
      m_target_pos_filters.resize(m_nAx);
      
      m_pos_cmd.resize(m_nAx);
      m_vel_cmd.resize(m_nAx);
      m_eff_cmd.resize(m_nAx);
      
      m_position_minimum_error.resize(m_nAx);
      
      for (int i = 0;i<m_nAx;i++)
      {
        Eigen::VectorXd init_cond(1);
        init_cond(0)=0;
        std::string name = m_joint_names.at(i)+"/position";
        m_pos_controllers.at(i)=itia::control::createDiscreteStateSpareFromParam(m_controller_nh, name,init_cond);
        
        name = m_joint_names.at(i)+"/pos_filter";
        m_pos_filters.at(i)=itia::control::createDiscreteStateSpareFromParam(m_controller_nh, name,init_cond);
        
        name = m_joint_names.at(i)+"/target_pos_filter";
        m_target_pos_filters.at(i)=itia::control::createDiscreteStateSpareFromParam(m_controller_nh, name,init_cond);
        
        double tmp;
        name = m_joint_names.at(i)+"/position_minimum_error";
        if (!m_controller_nh.getParam(name,tmp))
        {
          ROS_WARN("no position_minimum_error specified for joint %s, set equal to zero",m_joint_names.at(i).c_str());
          tmp=0;
        }
        m_position_minimum_error.at(i)=tmp;
      }
      
      
      
      ROS_INFO("Controller '%s%s%s' well initialized",GREEN,m_controller_nh.getNamespace().c_str(),RESET);
      
      m_well_init=true;
      return;
      
    }
    
    void VelocityControl::starting(const ros::Time& time)
    {
      m_configured = false;       
      m_feedforward_active=false;
      m_queue.callAvailable();
      
      ros::Time t0=ros::Time::now();
      ROS_INFO("waiting for '%s%s%s'",GREEN,m_js_fb_sub.getTopic().c_str(),RESET);
      while (!m_fb_js_rec.isANewDataAvailable())
      {
        m_queue.callAvailable();
        if ( (ros::Time::now()-t0).toSec()>20 )
        {
          ROS_FATAL_STREAM(m_controller_nh.getNamespace()+" feedback_topic not received");
          ROS_FATAL("ERROR DURING INITIALIZATION CONTROLLER '%s'", m_controller_nh.getNamespace().c_str());
          return;
        }
        ros::Duration(0.001).sleep();
      }
      ROS_INFO("TOPIC RECEIVED in %f seconds",(ros::Time::now()-t0).toSec());
      
      
      sensor_msgs::JointState fb_msg=m_fb_js_rec.getData();
      if (!itia::rutils::permutationName(m_joint_names,fb_msg.name,fb_msg.position))
      {
        ROS_ERROR("Error in feedback message");
        std::fill(m_vel_cmd.begin(),m_vel_cmd.end(),0.0);
        return;
      }
      
      if ( fb_msg.position.size() != m_nAx)
      {
        ROS_FATAL("joint states topic has wrong dimensions (%zu, instead of %d)",fb_msg.position.size(), m_nAx);
        ROS_FATAL("ERROR DURING STARTING CONTROLLER '%s'", m_controller_nh.getNamespace().c_str());
        return;
      }
      
      for (int i = 0;i<m_nAx;i++)
      {
        ros::Time t1=ros::Time::now();
        ROS_INFO("[ %s%s%s ] Creating controller objects", BOLDGREEN, m_controller_nh.getNamespace().c_str(), RESET);
        
        Eigen::VectorXd initial_state;
        
        Eigen::VectorXd init_vel(1);
        init_vel (0)=fb_msg.velocity.at(i);
        
        initial_state = m_pos_controllers.at(i)->getInitializationMatrix()*init_vel;
        m_pos_controllers.at(i)->setState(initial_state);
        
        Eigen::VectorXd init_pos(1);
        init_pos (0)=fb_msg.position.at(i);
        initial_state = m_pos_filters.at(i)->getInitializationMatrix()*init_pos;
        m_pos_filters.at(i)->setState(initial_state);
        initial_state = m_target_pos_filters.at(i)->getInitializationMatrix()*init_pos;
        m_target_pos_filters.at(i)->setState(initial_state);
        
        
        ROS_INFO("----- in %f seconds",(ros::Time::now()-t1).toSec());
        
      }
      
      ROS_INFO("Controller '%s%s%s' started in %f seconds",GREEN,m_controller_nh.getNamespace().c_str(),RESET,(ros::Time::now()-t0).toSec());
      
    }
    
    void VelocityControl::stopping(const ros::Time& time)
    {
      ROS_INFO("[ %s%s%s ] Stopping controller", BOLDGREEN, m_controller_nh.getNamespace().c_str(), RESET);
      m_configured = false;
      m_feedforward_active=false;
      
      sensor_msgs::JointState fb_msg=m_fb_js_rec.getData();
      if (!itia::rutils::permutationName(m_joint_names,fb_msg.name,fb_msg.position))
      {
        ROS_ERROR("Error in feedback message");
        std::fill(m_vel_cmd.begin(),m_vel_cmd.end(),0.0);
        return;
      }
      
      m_pos_cmd=fb_msg.position;
      std::fill(m_vel_cmd.begin(),m_vel_cmd.end(),0.0);
    }
    
    void VelocityControl::update(const ros::Time& time, const ros::Duration& period)
    {
      m_queue.callAvailable();
      m_configured = m_configured || (m_target_js_rec.isANewDataAvailable());
      m_feedforward_active=m_feedforward_active || (m_ffw_js_rec.isANewDataAvailable());
      m_configured &= (m_target_js_rec.getData().position.size()==m_nAx);
      
      Eigen::VectorXd pos_filter_input(1);
      Eigen::VectorXd target_pos_filter_input(1);
      
      Eigen::VectorXd pos_filter_output(1);
      Eigen::VectorXd target_pos_filter_output(1);
      
      Eigen::VectorXd pos_controller_input(1);
      Eigen::VectorXd pos_controller_output(1);
      
      Eigen::VectorXd q(m_nAx);
      Eigen::VectorXd Dq(m_nAx);
      
      sensor_msgs::JointState fb_msg=m_fb_js_rec.getData();
      if (!itia::rutils::permutationName(m_joint_names,fb_msg.name,fb_msg.position))
      {
        ROS_ERROR("Error in feedback message");
        std::fill(m_vel_cmd.begin(),m_vel_cmd.end(),0.0);
        return;
      }
      sensor_msgs::JointState target_msg=m_target_js_rec.getData();
      if (!itia::rutils::permutationName(m_joint_names,target_msg.name,target_msg.position,target_msg.velocity))
      {
        ROS_ERROR("Error in feedback message");
        std::fill(m_vel_cmd.begin(),m_vel_cmd.end(),0.0);
        return;
      }
      sensor_msgs::JointState ffw_msg=m_ffw_js_rec.getData();
      if (!itia::rutils::permutationName(m_joint_names,ffw_msg.name,ffw_msg.position,ffw_msg.velocity))
      {
        ROS_ERROR("Error in feedback message");
        std::fill(m_vel_cmd.begin(),m_vel_cmd.end(),0.0);
        return;
      }
      
      try
      {
        for (int iAx = 0;iAx<m_nAx;iAx++)
        {
          pos_filter_input(0)=fb_msg.position.at(iAx);
          pos_filter_output = m_pos_filters.at(iAx)->update(pos_filter_input); 
          
          if (m_configured)
            target_pos_filter_input(0) = target_msg.position.at(iAx);
          else
            target_pos_filter_input(0) = m_target_pos_filters.at(iAx)->getOutput()(0);
          
          if (m_feedforward_active)
            target_pos_filter_input(0) += ffw_msg.position.at(iAx);
          target_pos_filter_output = m_target_pos_filters.at(iAx)->update(target_pos_filter_input); 
          
          pos_controller_input(0) = target_pos_filter_output(0)-pos_filter_output(0); //position error
          if (std::abs(pos_controller_input(0)) < m_position_minimum_error.at(iAx))
            pos_controller_input(0)=0.0;
          
          pos_controller_output    = m_pos_controllers.at(iAx)->update( pos_controller_input );
          if (m_configured)
            pos_controller_output(0) += target_msg.velocity.at(iAx);
          if (m_feedforward_active)
            pos_controller_output(0) += ffw_msg.velocity.at(iAx);
          
          m_pos_cmd.at(iAx) = target_pos_filter_output(0);
          m_vel_cmd.at(iAx) = pos_controller_output(0);
          m_eff_cmd.at(iAx) = 0.0;
        }
        
      }
      catch (...)
      {
        ROS_WARN("something wrong: Controller '%s%s%s'",BOLDRED,m_controller_nh.getNamespace().c_str(),RESET);
      }
      
    }
    
    
    
    
  }
}
