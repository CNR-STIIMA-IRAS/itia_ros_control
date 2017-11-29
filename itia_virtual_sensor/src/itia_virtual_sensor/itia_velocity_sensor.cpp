
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

#include <itia_virtual_sensor/itia_velocity_sensor.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(itia::control::VelocitySensor, controller_interface::ControllerBase);


namespace itia
{
namespace control
{
    
    
VelocitySensor::~VelocitySensor()
{
  
}

bool VelocitySensor::init(itia::control::PosVelEffJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  m_root_nh = root_nh;
  m_controller_nh = controller_nh;
  m_hw = hw;
  
  m_change_tool = m_controller_nh.advertiseService("tool_configured",&itia::control::VelocitySensor::configureTool,this);
  
  m_controller_nh.setCallbackQueue(&m_queue);
  m_filter=0.999;
  
  m_Tft.setIdentity();
  m_Tub.setIdentity();
  
  try{
    
    std::string setpoint_topic_name;
    if (!m_controller_nh.getParam("setpoint_topic_name", setpoint_topic_name))
    {
      ROS_FATAL_STREAM(m_controller_nh.getNamespace()+"/'setpoint_topic_name' does not exist");
      ROS_FATAL("ERROR DURING INITIALIZATION CONTROLLER '%s'", m_controller_nh.getNamespace().c_str());
      return false;
    }
    m_js_target_sub = m_controller_nh.subscribe<sensor_msgs::JointState>(setpoint_topic_name,    1, &itia::rutils::MsgReceiver<sensor_msgs::JointState>::callback, &m_target_js_rec);
    
    std::string base_frame;
    std::string tool_frame;
    
    if (!m_controller_nh.getParam("base_frame", base_frame))
    {
      ROS_FATAL_STREAM(m_controller_nh.getNamespace()+"/'base_frame' does not exist");
      ROS_FATAL("ERROR DURING INITIALIZATION CONTROLLER '%s'", m_controller_nh.getNamespace().c_str());
      return false;
    }
    if (!m_controller_nh.getParam("tool_frame", tool_frame))
    {
      ROS_FATAL_STREAM(m_controller_nh.getNamespace()+"/'tool_frame' does not exist");
      ROS_FATAL("ERROR DURING INITIALIZATION CONTROLLER '%s'", m_controller_nh.getNamespace().c_str());
      return false;
    }
    if (!m_controller_nh.getParam("base_is_reference", m_base_is_reference))
    {
      ROS_WARN_STREAM(m_controller_nh.getNamespace()+"/'base_is_reference' does not exist, set true");
      m_base_is_reference=true;
    }
    
    m_joint_names=m_hw->getNames();
    m_nAx = m_joint_names.size();
    
    ROS_DEBUG("Controller '%s%s%s' controls the following joints:",GREEN,m_controller_nh.getNamespace().c_str(),RESET);
    for (unsigned int idx=0;idx<m_nAx;idx++)
      ROS_DEBUG(" - %s%s%s",GREEN,m_joint_names.at(idx).c_str(),RESET);
    
    
    
    std::map<std::string,double> frame_data;
    if (!tool_frame.compare("ACTIVE_TOOL"))
    {
      tool_frame="flange";
      if (m_root_nh.getParam("/env_param/TOOL",frame_data ))
      {
        try 
        {
          Eigen::Vector3d orig;
          orig << frame_data.at("x"), frame_data.at("y"), frame_data.at("z");
          m_Tft.translation()=orig;
          
          Eigen::AngleAxisd rot;
          rot = Eigen::AngleAxisd(frame_data.at("c"),   Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(frame_data.at("b"), Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(frame_data.at("a"),  Eigen::Vector3d::UnitX()); 
          m_Tft.linear()=rot.toRotationMatrix();
        }
        catch (std::out_of_range& e)
        {
          ROS_ERROR("A field is not present, set TOOL = FLANGE. error message: %s",e.what());
          m_Tft.setIdentity();
        }
      }
    }
      
    
    if (!base_frame.compare("ACTIVE_FRAME"))
    {
      base_frame="base";
      if (m_root_nh.getParam("/env_param/TOOL",frame_data ))
      {
        try 
        {
          Eigen::Vector3d orig;
          orig << frame_data.at("x"), frame_data.at("y"), frame_data.at("z");
          m_Tub.translation()=orig;
          
          Eigen::AngleAxisd rot;
          rot = Eigen::AngleAxisd(frame_data.at("c"),   Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(frame_data.at("b"), Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(frame_data.at("a"),  Eigen::Vector3d::UnitX()); 
          m_Tub.linear()=rot.toRotationMatrix();
        }
        catch (std::out_of_range& e)
        {
          ROS_ERROR("A field is not present, set FRAME = BASE. error message: %s",e.what());
          m_Tub.setIdentity();
        }
      }
    }
    
    
    std::string twist_topic_name;
    if (!m_controller_nh.getParam("twist_topic_name", twist_topic_name))
    {
      ROS_FATAL_STREAM(m_controller_nh.getNamespace()+"/'twist_topic_name' does not exist");
      ROS_FATAL("ERROR DURING INITIALIZATION CONTROLLER '%s'", m_controller_nh.getNamespace().c_str());
      return false;
    }
    m_twist_pub=m_root_nh.advertise<geometry_msgs::TwistStamped>(twist_topic_name,1);
    
    
    urdf::Model model;
    if (!model.initParam("/robot_description"))
    {
      ROS_ERROR("Urdf robot_description '%s' does not exist",(m_controller_nh.getNamespace()+"/robot_description").c_str());
      return false;
    }
    
    boost::shared_ptr<itia::dynamics::Link> root_link(new itia::dynamics::Link());  
    root_link->fromUrdf(model.root_link_);
    m_chain.reset(new itia::dynamics::Chain(root_link, base_frame,tool_frame, grav));
    m_chain->setInputJointsName(m_joint_names);
    
    
    m_velocity_filter_coeff = 0.9;
    if (!m_controller_nh.getParam("velocity_filtering_coeff", m_velocity_filter_coeff))
    {
      ROS_WARN_STREAM(m_controller_nh.getNamespace()+"/'velocity_filtering_coeff' does not exist, set 0.9");
      m_velocity_filter_coeff=0.9;
    }
    if (m_velocity_filter_coeff>1)
    {
      ROS_WARN_STREAM(m_controller_nh.getNamespace()+"/'velocity_filtering_coeff' should be less than 1, set 1");
      m_velocity_filter_coeff=1;
    }
    if (m_velocity_filter_coeff<0)
    {
      ROS_WARN_STREAM(m_controller_nh.getNamespace()+"/'velocity_filtering_coeff' should be greater than 1, set 0 (NO TORQUE ESTIMATION)");
      m_velocity_filter_coeff=0;
    }
    
  
    m_q.resize(m_nAx);
    m_Dq.resize(m_nAx);
    m_Dq_filt.resize(m_nAx);
  
    m_q.setZero();
    m_Dq.setZero();
    m_Dq_filt.setZero();
    
    

    
  }
  catch(const  std::exception& e)
  {
    ROS_FATAL("EXCEPTION: %s", e.what());
    return false;
  }
  ROS_INFO("Controller '%s%s%s' well initialized",GREEN,m_controller_nh.getNamespace().c_str(),RESET);
  
  return true;
  
}

void VelocitySensor::starting(const ros::Time& time)
{
  m_is_configured=false;
  ROS_INFO("Controller '%s%s%s' well started",GREEN,m_controller_nh.getNamespace().c_str(),RESET);
  m_start_time=ros::Time::now();
}

void VelocitySensor::stopping(const ros::Time& time)
{
  ROS_INFO("[ %s%s%s ] Stopping controller", BOLDGREEN, m_controller_nh.getNamespace().c_str(), RESET);
  std::fill(m_hw->m_joint_target_msgs->effort.begin(),m_hw->m_joint_target_msgs->effort.end(),0.0);
  m_hw->m_joint_target_msgs->header.stamp = ros::Time::now();
}

void VelocitySensor::update(const ros::Time& time, const ros::Duration& period)
{
  m_queue.callAvailable();
  double t=(ros::Time::now()-m_start_time).toSec();

  if (m_is_configured)
  {
    
    m_q=Eigen::VectorXd::Map(&(m_target_js_rec.getData().position.at(0)),m_target_js_rec.getData().position.size());
    m_Dq=Eigen::VectorXd::Map(&(m_target_js_rec.getData().velocity.at(0)),m_target_js_rec.getData().velocity.size());
    m_Dq_filt=m_filter*m_Dq_filt+(1-m_filter)*m_Dq;
    
    
    Eigen::Affine3d Tbf=m_chain->getTransformation(m_q);
    Eigen::Affine3d Tbt=Tbf*m_Tft;
    
    Eigen::Vector6d twist_flange_wrt_base_in_base;
    twist_flange_wrt_base_in_base=m_chain->getTwist(m_q,m_Dq_filt).back();
    Eigen::Vector6d twist_tool_wrt_base_in_base;
    twist_tool_wrt_base_in_base=itia::dynamics::spatialTranslation(twist_flange_wrt_base_in_base, (Eigen::Affine3d)Tbf.linear()* m_Tft.translation() );
    
    m_twist.reset(new geometry_msgs::TwistStamped);
    if (m_base_is_reference)
    {
      Eigen::Vector6d twist_tool_wrt_base_in_user;
      twist_tool_wrt_base_in_user=itia::dynamics::spatialRotation(twist_tool_wrt_base_in_base,m_Tub.linear());
      m_twist->twist.linear.x=twist_tool_wrt_base_in_user(0);
      m_twist->twist.linear.y=twist_tool_wrt_base_in_user(1);
      m_twist->twist.linear.z=twist_tool_wrt_base_in_user(2);
      
      m_twist->twist.angular.x=twist_tool_wrt_base_in_user(3);
      m_twist->twist.angular.y=twist_tool_wrt_base_in_user(4);
      m_twist->twist.angular.z=twist_tool_wrt_base_in_user(5);
      m_twist->header.frame_id=m_tool_frame+" velocity w.r.t base in user frame";
    }
    else
    {
      Eigen::Vector6d twist_tool_wrt_base_in_tool;
      twist_tool_wrt_base_in_tool=itia::dynamics::spatialRotation(twist_tool_wrt_base_in_base,Tbt.linear().transpose());
      m_twist->twist.linear.x=twist_tool_wrt_base_in_tool(0);
      m_twist->twist.linear.y=twist_tool_wrt_base_in_tool(1);
      m_twist->twist.linear.z=twist_tool_wrt_base_in_tool(2);
      
      m_twist->twist.angular.x=twist_tool_wrt_base_in_tool(3);
      m_twist->twist.angular.y=twist_tool_wrt_base_in_tool(4);
      m_twist->twist.angular.z=twist_tool_wrt_base_in_tool(5);
      m_twist->header.frame_id=m_tool_frame+" velocity w.r.t base in tool frame";
    } 
    m_twist->header.stamp=m_hw->m_joint_target_msgs->header.stamp=m_target_js_rec.getData().header.stamp;
    m_twist_pub.publish(m_twist);
  }
  else
  {
    m_is_configured=m_target_js_rec.isANewDataAvailable();
    if (m_is_configured)
    {
      ROS_DEBUG("configured");
      m_Dq=Eigen::VectorXd::Map(&(m_target_js_rec.getData().velocity.at(0)),m_target_js_rec.getData().velocity.size());
      m_Dq_filt=m_Dq;
    }
  }
  
}


bool VelocitySensor::configureTool ( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res )
{
  std::string base_frame;
  std::string tool_frame;
 
  
  if (!m_controller_nh.getParam("base_frame", base_frame))
  {
    ROS_FATAL_STREAM(m_controller_nh.getNamespace()+"/'base_frame' does not exist");
    ROS_FATAL("ERROR DURING INITIALIZATION CONTROLLER '%s'", m_controller_nh.getNamespace().c_str());
    return false;
  }
  if (!m_controller_nh.getParam("tool_frame", tool_frame))
  {
    ROS_FATAL_STREAM(m_controller_nh.getNamespace()+"/'tool_frame' does not exist");
    ROS_FATAL("ERROR DURING INITIALIZATION CONTROLLER '%s'", m_controller_nh.getNamespace().c_str());
    return false;
  }
  if (!m_controller_nh.getParam("base_is_reference", m_base_is_reference))
  {
    ROS_WARN_STREAM(m_controller_nh.getNamespace()+"/'base_is_reference' does not exist, set true");
    m_base_is_reference=true;
  }
  
  std::map<std::string,double> frame_data;
  if (!tool_frame.compare("ACTIVE_TOOL"))
  {
    tool_frame="flage";
    if (m_root_nh.getParam("/env_param/TOOL",frame_data ))
    {
      try 
      {
        Eigen::Vector3d orig;
        orig << frame_data.at("x"), frame_data.at("y"), frame_data.at("z");
        m_Tft.translation()=orig;
        
        Eigen::AngleAxisd rot;
        rot = Eigen::AngleAxisd(frame_data.at("c"),   Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(frame_data.at("b"), Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(frame_data.at("c"),  Eigen::Vector3d::UnitX()); 
        m_Tft.linear()=rot.toRotationMatrix();
      }
      catch (std::out_of_range& e)
      {
        ROS_ERROR("A field is not present, set TOOL = FLANGE. error message: %s",e.what());
        m_Tft.setIdentity();
      }
    }
  }
  
  if (!base_frame.compare("ACTIVE_FRAME"))
  {
    base_frame="base";
    if (m_root_nh.getParam("/env_param/TOOL",frame_data ))
    {
      try 
      {
        Eigen::Vector3d orig;
        orig << frame_data.at("x"), frame_data.at("y"), frame_data.at("z");
        m_Tub.translation()=orig;
        
        Eigen::AngleAxisd rot;
        rot = Eigen::AngleAxisd(frame_data.at("c"),   Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(frame_data.at("b"), Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(frame_data.at("a"),  Eigen::Vector3d::UnitX()); 
        m_Tub.linear()=rot.toRotationMatrix();
      }
      catch (std::out_of_range& e)
      {
        ROS_ERROR("A field is not present, set FRAME = BASE. error message: %s",e.what());
        m_Tub.setIdentity();
      }
    }
  }
  return true;
}

  
}
}
