#include <itia_cascade_control/itia_cascade_control.h>
#include <pluginlib/class_list_macros.h>


namespace itia
{
namespace control
{
		
		
  CascadeControl::CascadeControl(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh, std::vector<std::string> joint_names)
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
  
  
  m_use_target_torque=false;
  if (!m_controller_nh.getParam("use_target_torque", m_use_target_torque))
  {
    ROS_WARN_STREAM(m_controller_nh.getNamespace()+"/use_target_torque does not exist, set FALSE");
  }
  
  
  m_nAx = m_joint_names.size();
  ROS_INFO("Controller '%s%s%s' controls the following joints:",GREEN,m_controller_nh.getNamespace().c_str(),RESET);
  for (unsigned int idx=0;idx<m_nAx;idx++)
    ROS_INFO(" - %s%s%s",GREEN,m_joint_names.at(idx).c_str(),RESET);
  
  m_pos_controllers.resize(m_nAx);
  m_vel_controllers.resize(m_nAx);
  
  m_vel_filters.resize(m_nAx);
  m_target_vel_filters.resize(m_nAx);
  m_pos_filters.resize(m_nAx);
  m_target_pos_filters.resize(m_nAx);
  m_eff_filters.resize(m_nAx);
  
  m_pos_cmd.resize(m_nAx);
  m_vel_cmd.resize(m_nAx);
  m_eff_cmd.resize(m_nAx);
  m_static_friction.resize(m_nAx);
  m_static_friction_past_state.resize(m_nAx);
  m_static_friction_threshold.resize(m_nAx);
  m_position_minimum_error.resize(m_nAx);
  
  for (int i = 0;i<m_nAx;i++)
  {
    Eigen::VectorXd init_cond(1);
    init_cond(0)=0;
    std::string name = m_joint_names.at(i)+"/position";
    m_pos_controllers.at(i)=itia::control::createDiscreteStateSpareFromParam(m_controller_nh, name,init_cond);
    
    name = m_joint_names.at(i)+"/vel_filter";
    m_vel_filters.at(i)=itia::control::createDiscreteStateSpareFromParam(m_controller_nh, name,init_cond);
   
    name = m_joint_names.at(i)+"/target_vel_filter";
    m_target_vel_filters.at(i)=itia::control::createDiscreteStateSpareFromParam(m_controller_nh, name,init_cond);
    
    name = m_joint_names.at(i)+"/pos_filter";
    m_pos_filters.at(i)=itia::control::createDiscreteStateSpareFromParam(m_controller_nh, name,init_cond);
    
    name = m_joint_names.at(i)+"/target_pos_filter";
    m_target_pos_filters.at(i)=itia::control::createDiscreteStateSpareFromParam(m_controller_nh, name,init_cond);
    
    name = m_joint_names.at(i)+"/velocity";
    m_vel_controllers.at(i)=itia::control::createDiscreteStateSpareFromParam(m_controller_nh, name,init_cond);
    
    name = m_joint_names.at(i)+"/effort_filter";
    m_eff_filters.at(i)=itia::control::createDiscreteStateSpareFromParam(m_controller_nh, name,init_cond);
    
    double tmp;
    name = m_joint_names.at(i)+"/static_friction";
    if (!m_controller_nh.getParam(name,tmp))
    {
      ROS_WARN("no static friction specified for joint %s, set equal to zero",m_joint_names.at(i).c_str());
      tmp=0;
    }
    m_static_friction.at(i)=tmp;
    name = m_joint_names.at(i)+"/static_friction_vel_threshold";
    if (!m_controller_nh.getParam(name,tmp))
    {
      tmp=0;
    }
    m_static_friction_threshold.at(i)=tmp;
    m_static_friction_past_state.at(i)=1;
    name = m_joint_names.at(i)+"/position_minimum_error";
    if (!m_controller_nh.getParam(name,tmp))
    {
      ROS_WARN("no position_minimum_error specified for joint %s, set equal to zero",m_joint_names.at(i).c_str());
      tmp=0;
    }
    m_position_minimum_error.at(i)=tmp;
  }
  
  if (!m_controller_nh.getParam("use_model_feedforward", m_use_model_feedforward))
  {
    ROS_WARN_STREAM(m_controller_nh.getNamespace()+"/'use_model_feedforward' does not exist, set equal to false");
    m_use_model_feedforward=false;
  }
  
  std::string base_frame;
  std::string tool_frame;
  if (m_use_model_feedforward)
  {
    if (!m_controller_nh.getParam("base_frame", base_frame))
    {
      ROS_FATAL_STREAM(m_controller_nh.getNamespace()+"/'base_frame' does not exist, disable model feedforward");
      m_use_model_feedforward=false;;
    }
    else if (!m_controller_nh.getParam("tool_frame", tool_frame))
    {
      ROS_FATAL_STREAM(m_controller_nh.getNamespace()+"/'tool_frame' does not exist, disable model feedforward");
      m_use_model_feedforward=false;;
    }
    else if (!m_controller_nh.hasParam("robot_description"))
    {
      ROS_FATAL_STREAM(m_controller_nh.getNamespace()+"/'robot_description' does not exist, disable model feedforward");
      m_use_model_feedforward=false;;
    }
    else if (!m_controller_nh.getParam("use_feedback", m_use_feedback))
    {
      ROS_WARN_STREAM(m_controller_nh.getNamespace()+"/'use_feedback' does not exist, set equal to false");
      m_use_feedback=false;
    }
  }
  if (m_use_model_feedforward)
  {
    urdf::Model model;
    model.initParam(m_controller_nh.getNamespace()+"/robot_description");
    Eigen::Vector3d grav;
    grav << 0, 0, -9.806;
    
    boost::shared_ptr<itia::dynamics::Link> root_link(new itia::dynamics::Link());  
    root_link->fromUrdf(model.root_link_);
    m_chain.reset(new itia::dynamics::Chain(root_link, base_frame,tool_frame, grav));
    m_chain->setInputJointsName(m_joint_names);
  }
  
  ROS_INFO("Controller '%s%s%s' well initialized",GREEN,m_controller_nh.getNamespace().c_str(),RESET);

  m_well_init=true;
  return;
  
}

void CascadeControl::starting(const ros::Time& time)
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

  std::vector<double> fb_joint_positions = m_fb_js_rec.getData().position;
  if ( fb_joint_positions.size() != m_nAx)
  {
    ROS_FATAL("joint states topic has wrong dimensions (%zu, instead of %d)",fb_joint_positions.size(), m_nAx);
    ROS_FATAL("ERROR DURING STARTING CONTROLLER '%s'", m_controller_nh.getNamespace().c_str());
    return;
  }
  
  for (int i = 0;i<m_nAx;i++)
  {
    ros::Time t1=ros::Time::now();
    ROS_INFO("[ %s%s%s ] Creating controller objects", BOLDGREEN, m_controller_nh.getNamespace().c_str(), RESET);
    
    Eigen::VectorXd initial_state;
    
    Eigen::VectorXd init_vel(1);
    init_vel (0)=m_fb_js_rec.getData().velocity.at(i);
    
    initial_state = m_pos_controllers.at(i)->getInitializationMatrix()*init_vel;
    m_pos_controllers.at(i)->setState(initial_state);
    initial_state = m_vel_filters.at(i)->getInitializationMatrix()*init_vel;
    m_vel_filters.at(i)->setState(initial_state);
    initial_state = m_target_vel_filters.at(i)->getInitializationMatrix()*init_vel;
    m_target_vel_filters.at(i)->setState(initial_state);
  
    Eigen::VectorXd init_pos(1);
    init_pos (0)=m_fb_js_rec.getData().position.at(i);
    initial_state = m_pos_filters.at(i)->getInitializationMatrix()*init_pos;
    m_pos_filters.at(i)->setState(initial_state);
    initial_state = m_target_pos_filters.at(i)->getInitializationMatrix()*init_pos;
    m_target_pos_filters.at(i)->setState(initial_state);
  
    
    Eigen::VectorXd init_eff(1);
    init_eff(0)=m_fb_js_rec.getData().effort.at(i);
    initial_state = m_vel_controllers.at(i)->getInitializationMatrix()*init_eff;
    m_vel_controllers.at(i)->setState(initial_state);
    initial_state = m_eff_filters.at(i)->getInitializationMatrix()*init_eff;
    m_eff_filters.at(i)->setState(initial_state);
  
    ROS_INFO("----- in %f seconds",(ros::Time::now()-t1).toSec());
    
  }
  
  ROS_INFO("Controller '%s%s%s' started in %f seconds",GREEN,m_controller_nh.getNamespace().c_str(),RESET,(ros::Time::now()-t0).toSec());

}

void CascadeControl::stopping(const ros::Time& time)
{
  ROS_INFO("[ %s%s%s ] Stopping controller", BOLDGREEN, m_controller_nh.getNamespace().c_str(), RESET);
  m_configured = false;
  m_feedforward_active=false;
  m_pos_cmd=m_fb_js_rec.getData().position;
  std::fill(m_vel_cmd.begin(),m_vel_cmd.end(),0.0);
//   std::fill(m_eff_cmd.begin(),m_eff_cmd.end(),0.0);
}

void CascadeControl::update(const ros::Time& time, const ros::Duration& period)
{
  m_queue.callAvailable();
  m_configured = m_configured || (m_target_js_rec.isANewDataAvailable());
  m_feedforward_active=m_feedforward_active || (m_ffw_js_rec.isANewDataAvailable());
  m_configured &= (m_target_js_rec.getData().position.size()==m_nAx);
  
  Eigen::VectorXd pos_filter_input(1);
  Eigen::VectorXd target_pos_filter_input(1);
  Eigen::VectorXd vel_filter_input(1);
  Eigen::VectorXd eff_filter_input(1);
  
  Eigen::VectorXd pos_filter_output(1);
  Eigen::VectorXd target_pos_filter_output(1);
  Eigen::VectorXd vel_filter_output(1);
  Eigen::VectorXd target_vel_filter_output(1);
  Eigen::VectorXd eff_filter_output(1);
  
  Eigen::VectorXd pos_controller_input(1);
  Eigen::VectorXd vel_controller_input(3);
  Eigen::VectorXd pos_controller_output(1);
  Eigen::VectorXd vel_controller_output(1);
  
  Eigen::VectorXd q(m_nAx);
  Eigen::VectorXd Dq(m_nAx);
  Eigen::VectorXd DDq(m_nAx);
  Eigen::VectorXd tau(m_nAx);
  DDq.setZero();

  try
  {
    for (int iAx = 0;iAx<m_nAx;iAx++)
    {
      pos_filter_input(0)=m_fb_js_rec.getData().position.at(iAx);
      eff_filter_input(0)=m_fb_js_rec.getData().effort.at(iAx);
      vel_filter_input(0)=m_fb_js_rec.getData().velocity.at(iAx);
      pos_filter_output = m_pos_filters.at(iAx)->update(pos_filter_input); 
      vel_filter_output = m_vel_filters.at(iAx)->update(vel_filter_input); 
      eff_filter_output = m_eff_filters.at(iAx)->update(eff_filter_input); 
      
      if (m_configured)
        target_pos_filter_input(0) = m_target_js_rec.getData().position.at(iAx);
      else
        target_pos_filter_input(0) = m_target_pos_filters.at(iAx)->getOutput()(0);
      
      if (m_feedforward_active)
        target_pos_filter_input(0) += m_ffw_js_rec.getData().position.at(iAx);
      target_pos_filter_output = m_target_pos_filters.at(iAx)->update(target_pos_filter_input); 
      
      pos_controller_input(0) = target_pos_filter_output(0)-pos_filter_output(0); //position error
      
      
      pos_controller_output    = m_pos_controllers.at(iAx)->update( pos_controller_input );
      if (m_configured)
        pos_controller_output(0) += m_target_js_rec.getData().velocity.at(iAx);
      if (m_feedforward_active)
        pos_controller_output(0) += m_ffw_js_rec.getData().velocity.at(iAx);
        
      target_vel_filter_output = m_target_vel_filters.at(iAx)->update(pos_controller_output);
      
      vel_controller_input(0) = target_vel_filter_output(0)-vel_filter_output(0); //velocity error
      vel_controller_input(1) = vel_controller_input(0);
      
      if (std::abs(pos_controller_input(0))<m_position_minimum_error.at(iAx))
        if (m_configured)
          if (std::abs(m_target_js_rec.getData().velocity.at(iAx))<0.01)
            vel_controller_input(1)=0;
        else
          vel_controller_input(1)=0;
      
      
      vel_controller_input(2) = eff_filter_output(0) -m_vel_controllers.at(iAx)->getOutput()(0); // difference between real effort and last compute effort
      
      vel_controller_output = m_vel_controllers.at(iAx)->update(vel_controller_input);
        
      
      double friction_comp=0;
      if (vel_filter_output(0)>m_static_friction_threshold.at(iAx))
        friction_comp=m_static_friction.at(iAx);
      else if (vel_filter_output(0)<-m_static_friction_threshold.at(iAx))
        friction_comp=-m_static_friction.at(iAx);
      else 
        friction_comp=m_static_friction.at(iAx)*vel_filter_output(0)/m_static_friction_threshold.at(iAx);
      
      
      if (m_use_model_feedforward)
      {
        if (m_use_feedback)
        {
          q(iAx)=pos_filter_output(0);
          Dq(iAx)=vel_filter_output(0);
        }
        else
        {
          q(iAx)=target_pos_filter_output(0);
          Dq(iAx)=target_vel_filter_output(0);
        }
      }
      
      m_pos_cmd.at(iAx) = target_pos_filter_output(0);
      m_vel_cmd.at(iAx) = target_vel_filter_output(0);
      m_eff_cmd.at(iAx) = vel_controller_output(0)+friction_comp;
      if (m_feedforward_active)
        m_eff_cmd.at(iAx) += m_ffw_js_rec.getData().effort.at(iAx);
      if (m_use_target_torque && m_configured)
        m_eff_cmd.at(iAx) += m_target_js_rec.getData().effort.at(iAx);
    }
    if (m_use_model_feedforward)
    {
      tau = m_chain->getJointTorque(q, Dq, DDq);
      for (int iAx = 0;iAx<m_nAx;iAx++)
        m_eff_cmd.at(iAx) += tau(iAx);
    }
  }
  catch (...)
  {
    ROS_WARN("something wrong: Controller '%s%s%s'",BOLDRED,m_controller_nh.getNamespace().c_str(),RESET);
  }
  
}



		
}
}
