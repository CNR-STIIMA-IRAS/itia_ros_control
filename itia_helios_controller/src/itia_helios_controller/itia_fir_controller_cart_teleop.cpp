
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

#include <itia_helios_controller/itia_fir_controller_cart_teleop.h>
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(itia::motion::CartTeleopFirController, controller_interface::ControllerBase);

namespace itia
{
namespace motion
{
  
CartTeleopFirController::~CartTeleopFirController()
{

}

bool CartTeleopFirController::init(itia::control::PosVelEffJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  m_root_nh = root_nh;
  m_controller_nh = controller_nh;
  m_hw = hw;
  m_scale_speed=0.250;
  
  m_change_tool = m_controller_nh.advertiseService("tool_configured",&itia::motion::CartTeleopFirController::configureTool,this);
  m_diagnostics_pub = m_root_nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics",1);
  
  std::string feedback_topic_name;
  std::string setpoint_topic_name;
  
  if (!m_controller_nh.getParam("feedback_topic_name", feedback_topic_name))
  {
    ROS_FATAL_STREAM(m_controller_nh.getNamespace()+"/'feedback_topic_name' does not exist");
    ROS_FATAL("ERROR DURING STARTING CONTROLLER '%s'", m_controller_nh.getNamespace().c_str());
    return false;
  }
  if (!m_controller_nh.getParam("setpoint_topic_name", setpoint_topic_name))
  {
    ROS_FATAL_STREAM(m_controller_nh.getNamespace()+"/'setpoint_topic_name' does not exist");
    ROS_FATAL("ERROR DURING INITIALIZATION CONTROLLER '%s'", m_controller_nh.getNamespace().c_str());
  }
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
  
  double timeout;
  if (!m_controller_nh.getParam("message_timeout", timeout))
  {
    ROS_FATAL_STREAM(m_controller_nh.getNamespace()+"/'message_timeout' does not exist, set equal to 0.001 [s]");
    timeout=0.001;
  }
  m_message_timeout=ros::Duration(timeout);
  
  
  
  m_js_fb_sub     = m_controller_nh.subscribe<sensor_msgs::JointState>(feedback_topic_name, 1, &itia::rutils::MsgReceiver<sensor_msgs::JointState>::callback, &m_fb_js_rec);
  m_target_sub = m_controller_nh.subscribe<geometry_msgs::TwistStamped>(setpoint_topic_name,    1, &itia::rutils::MsgReceiver<geometry_msgs::TwistStamped>::callback, &m_target_rec);
  
  m_joint_names = m_hw->getNames();
  m_nAx = m_joint_names.size();
  
  m_controller_nh.setParam("fir_params/dimension",m_nAx);
  
	ROS_INFO("FIR planner '%s%s%s' well initialized, controlled joints:",GREEN,m_controller_nh.getNamespace().c_str(),RESET);
	for (unsigned int idx=0; idx<m_nAx;idx++)
		ROS_INFO("- %s%s%s",GREEN,m_joint_names.at(idx).c_str(),RESET);
  
  m_Tft.setIdentity();
  m_Tub.setIdentity();
  
  std::map<std::string,double> frame_data;
  ROS_INFO("TOOL = '%s'",tool_frame.c_str());
  if (!tool_frame.compare("ACTIVE_TOOL"))
  {
    ROS_INFO("QUI");
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
    if (m_root_nh.getParam("/env_param/FRAME",frame_data ))
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
        m_Tub=m_Tub.inverse();
      }
      catch (std::out_of_range& e)
      {
        ROS_ERROR("A field is not present, set FRAME = BASE. error message: %s",e.what());
        m_Tub.setIdentity();
      }
    }
  }
  
  urdf::Model model;
  if (!model.initParam("/robot_description"))
  {
    ROS_ERROR("robot_description parameter not found!");
    return false;
  }
//   model.getJoint()->limits->lower;
  
  Eigen::Vector3d grav;
  grav << 0, 0, -9.806;
  
  boost::shared_ptr<itia::dynamics::Link> root_link(new itia::dynamics::Link());  
  root_link->fromUrdf(model.root_link_);
  m_chain.reset(new itia::dynamics::Chain(root_link, base_frame,tool_frame, grav));
  m_chain->setInputJointsName(m_joint_names);
  

  m_upper_limit.resize(m_nAx);
  m_lower_limit.resize(m_nAx);
  for (int iAx=0;iAx<m_nAx;iAx++)
  {
    if (!model.getJoint(m_joint_names.at(iAx)))
    {
      diagnostic_msgs::DiagnosticArray diag_msg;
      diag_msg.header.stamp=ros::Time::now();
      diag_msg.status.resize(1);
      diag_msg.status.at(0).hardware_id="FourByThreeRobot";
      diag_msg.status.at(0).level=diagnostic_msgs::DiagnosticStatus::ERROR;
      diag_msg.status.at(0).message="Joint name "+m_joint_names.at(iAx)+" is not part of robot_description";
      diag_msg.status.at(0).values.resize(1);
      diag_msg.status.at(0).values.at(0).key="MISSING JOINT";
      diag_msg.status.at(0).values.at(0).value=m_joint_names.at(iAx);
      m_diagnostics_pub.publish(diag_msg);
      return false;
    }
    m_upper_limit (iAx) = model.getJoint(m_joint_names.at(iAx))->limits->upper;
    m_lower_limit (iAx) = model.getJoint(m_joint_names.at(iAx))->limits->lower;
  }
  
  
  m_override=m_safe_override_1=m_safe_override_2=100;
  
  m_override_topic = m_root_nh.subscribe<std_msgs::Int64>("/speed_ovr",1,&itia::motion::CartTeleopFirController::overrideCallback,this);
  m_safe_override_topic_1 = m_root_nh.subscribe<std_msgs::Int64>("/safe_ovr_1",1,&itia::motion::CartTeleopFirController::safeOverrideCallback_1,this);
  m_safe_override_topic_2 = m_root_nh.subscribe<std_msgs::Int64>("/safe_ovr_2",1,&itia::motion::CartTeleopFirController::safeOverrideCallback_2,this);
  return true;
}

void CartTeleopFirController::starting(const ros::Time& time)
{
  m_configured = false;  
 
  Eigen::VectorXd qini(m_nAx);
  qini.setZero();
  m_motion.resize(3, m_nAx);
  m_motion.setZero();
  
  ROS_INFO("waiting");
  ros::WallTime t0 = ros::WallTime::now();
  while (!m_fb_js_rec.isANewDataAvailable() && (ros::WallTime::now() - t0).toSec() <10 )
    ros::WallDuration(0.001).sleep();
  if (!m_fb_js_rec.isANewDataAvailable())
	{
		ROS_FATAL("FIR planner did not receive message from '%s' topic",m_js_fb_sub.getTopic().c_str());
    return;
	}
  
  if (m_fb_js_rec.getData().position.size() != m_nAx)
  {
    ROS_FATAL("error feedback size is wrong");
    return;
  }
  for (int idx = 0;idx<m_nAx;idx++)
		qini(idx) = m_fb_js_rec.getData().position.at(idx);

  ROS_INFO("configuring planner");
  try 
  {
		m_planner.reset(new itia::helios::FirPlanner(m_controller_nh, "fir_params", qini));
    m_planner->changeVelocityOverride(m_override*m_safe_override_1*m_safe_override_2/10000.0);
    m_configured = true;
    ROS_INFO("started");
  }
  catch (const std::exception& e)
  {
    ROS_FATAL("FAILED DURING HELIOS LOADING: %s", e.what());
    m_configured = false;
  }
  

}

void CartTeleopFirController::stopping(const ros::Time& time)
{
  ROS_INFO("[ %s%s%s ] Stopping controller", BOLDGREEN, m_controller_nh.getNamespace().c_str(), RESET);
  m_configured = false;
}

void CartTeleopFirController::update(const ros::Time& time, const ros::Duration& period)
{
  Eigen::Matrix<double,6,1> twist;
  twist.setZero();
  bool no_value=true;
  
  if ((ros::Time::now()-m_target_rec.getData().header.stamp)<m_message_timeout)
  {
    no_value=true;
    // twist is in base frame
    twist(0,0)=m_target_rec.getData().twist.linear.x*m_scale_speed;
    twist(1,0)=m_target_rec.getData().twist.linear.y*m_scale_speed;
    twist(2,0)=m_target_rec.getData().twist.linear.z*m_scale_speed;
    twist(3,0)=m_target_rec.getData().twist.angular.x*m_scale_speed;
    twist(4,0)=m_target_rec.getData().twist.angular.y*m_scale_speed;
    twist(5,0)=m_target_rec.getData().twist.angular.z*m_scale_speed;
    
    if (!m_target_rec.getData().header.frame_id.compare("TOOL"))
    {
      Eigen::Affine3d Tbt=m_chain->getTransformation(m_motion.row(0).transpose())*m_Tft;
      twist=itia::dynamics::spatialRotation(twist,Tbt.rotation());
    }
    if (!m_target_rec.getData().header.frame_id.compare("FRAME"))
      twist=itia::dynamics::spatialRotation(twist,m_Tub.rotation().inverse());
    
    
    Eigen::VectorXd vel(m_nAx);
    Eigen::Matrix6Xd J;
    
    J=m_chain->getJacobian(m_motion.row(0).transpose());
    
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
    if (svd.singularValues()(svd.cols()-1)==0)
      ROS_WARN_THROTTLE(1,"SINGULARITY POINT");
    else if (svd.singularValues()(0)/svd.singularValues()(svd.cols()-1) > 1e2)
      ROS_WARN_THROTTLE(1,"SINGULARITY POINT");
    
    vel = svd.solve(twist);
    
    m_motion = m_planner->updateVel(vel);
  }
  else
  {
    Eigen::VectorXd vel(m_nAx);
    vel.setZero();
    m_motion = m_planner->updateVel(vel);
    //m_motion = m_planner->softStop();
  }
    

  for (int iDim = 0;iDim<m_nAx;iDim++)
  {
    m_hw->m_joint_target_msgs->position.at(iDim) = m_motion(0, iDim);
    m_hw->m_joint_target_msgs->velocity.at(iDim) = m_motion(1, iDim);
    m_hw->m_joint_target_msgs->effort.at(iDim)   = m_motion(2, iDim);
  }
  m_hw->m_joint_target_msgs->header.stamp = ros::Time::now();

}

void CartTeleopFirController::overrideCallback(const std_msgs::Int64ConstPtr& msg)
{
  m_override=msg->data;
  if (m_planner)
    m_planner->changeVelocityOverride(m_override*m_safe_override_1*m_safe_override_2/10000.0);
}

void CartTeleopFirController::safeOverrideCallback_1(const std_msgs::Int64ConstPtr& msg)
{
  m_safe_override_1=msg->data;
  if (m_safe_override_1>100)
    m_safe_override_1=100;
  else if (m_safe_override_1<0)
    m_safe_override_1=0;
  
  if (m_planner)
    m_planner->changeVelocityOverride(m_override*m_safe_override_1*m_safe_override_2/10000.0);
}

void CartTeleopFirController::safeOverrideCallback_2(const std_msgs::Int64ConstPtr& msg)
{
  m_safe_override_2=msg->data;
  if (m_safe_override_2>100)
    m_safe_override_2=100;
  else if (m_safe_override_2<0)
    m_safe_override_2=0;
  
  if (m_planner)
    m_planner->changeVelocityOverride(m_override*m_safe_override_1*m_safe_override_2/10000.0);
}


bool CartTeleopFirController::configureTool ( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res )
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
    if (m_root_nh.getParam("/env_param/FRAME",frame_data ))
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
        m_Tub=m_Tub.inverse();
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
