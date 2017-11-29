#include <itia_virtual_sensor/itia_virtual_sensor.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(itia::control::VirtualSensor, controller_interface::ControllerBase);


namespace itia
{
namespace control
{
    
    
VirtualSensor::~VirtualSensor()
{
  
}

bool VirtualSensor::init(itia::control::PosVelEffJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  m_root_nh = root_nh;
  m_controller_nh = controller_nh;
  m_hw = hw;
  
  m_change_tool = m_root_nh.advertiseService("tool_configured",&itia::control::VirtualSensor::configureTool,this);
  m_resetting_force = m_controller_nh.advertiseService("virtual_sensor_zeroing",&itia::control::VirtualSensor::resetOffset,this);
  
  m_controller_nh.setCallbackQueue(&m_queue);
  m_filter=0.999;
  
  m_center_of_mass.resize(3);
  m_center_of_mass.setZero();
  m_mass=0;
  
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
    
    std::string external_torques_topic_name;
    if (!m_controller_nh.getParam("external_torques_topic_name", external_torques_topic_name))
    {
      ROS_FATAL_STREAM(m_controller_nh.getNamespace()+"/'external_torques_topic_name' does not exist");
      ROS_FATAL("ERROR DURING INITIALIZATION CONTROLLER '%s'", m_controller_nh.getNamespace().c_str());
      return false;
    }
    
    m_extra_torque_pub=m_root_nh.advertise<sensor_msgs::JointState>(external_torques_topic_name,1);
    
    std::string external_wrench_topic_name;
    if (!m_controller_nh.getParam("external_wrench_topic_name", external_wrench_topic_name))
    {
      ROS_FATAL_STREAM(m_controller_nh.getNamespace()+"/'external_wrench_topic_name' does not exist");
      ROS_FATAL("ERROR DURING INITIALIZATION CONTROLLER '%s'", m_controller_nh.getNamespace().c_str());
      return false;
    }
    m_extra_wrench_pub=m_root_nh.advertise<geometry_msgs::WrenchStamped>(external_wrench_topic_name,1);
    
    m_joint_names=m_hw->getNames();
    m_nAx = m_joint_names.size();
    
    ROS_DEBUG("Controller '%s%s%s' controls the following joints:",GREEN,m_controller_nh.getNamespace().c_str(),RESET);
    for (unsigned int idx=0;idx<m_nAx;idx++)
      ROS_DEBUG(" - %s%s%s",GREEN,m_joint_names.at(idx).c_str(),RESET);
    
    
    m_extra_wrench.reset(new geometry_msgs::WrenchStamped);
    m_extra_torque.reset(new sensor_msgs::JointState);
    m_extra_torque->position.resize(m_nAx);
    m_extra_torque->velocity.resize(m_nAx);
    m_extra_torque->effort.resize(m_nAx);
    m_extra_torque->name=m_joint_names;
    std::fill(m_extra_torque->position.begin(),m_extra_torque->position.end(),0.0);
    std::fill(m_extra_torque->velocity.begin(),m_extra_torque->velocity.end(),0.0);
    std::fill(m_extra_torque->effort.begin(),m_extra_torque->effort.end(),0.0);
    
    m_tool_frame=tool_frame;
    m_Tft.setIdentity();
    m_Tub.setIdentity();

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
    
    
    
    
    urdf::Model model;
    if (!model.initParam(m_controller_nh.getNamespace()+"/robot_description"))
    {
      ROS_ERROR("Urdf robot_description '%s' does not exist",(m_controller_nh.getNamespace()+"/robot_description").c_str());
      return false;
    }
    
    grav << 0, 0, -9.806;
    
    boost::shared_ptr<itia::dynamics::Link> root_link(new itia::dynamics::Link());  
    root_link->fromUrdf(model.root_link_);
    m_chain.reset(new itia::dynamics::Chain(root_link, base_frame,tool_frame, grav));
    m_chain->setInputJointsName(m_joint_names);
    
    
    
    m_forgetting_factor=0.95;
    m_force_filter_coeff  = 0.9;
    m_torque_filter_coeff = 0.9;
    if (!m_controller_nh.getParam("forgetting_factor", m_forgetting_factor))
    {
      ROS_WARN_STREAM(m_controller_nh.getNamespace()+"/'forgetting_factor' does not exist, set 0.95");
      m_forgetting_factor=0.95;
    }
    if (!m_controller_nh.getParam("force_filtering_coeff", m_force_filter_coeff))
    {
      ROS_WARN_STREAM(m_controller_nh.getNamespace()+"/'force_filtering_coeff' does not exist, set 0.9");
      m_force_filter_coeff=0.9;
    }
    if (m_force_filter_coeff>1)
    {
      ROS_WARN_STREAM(m_controller_nh.getNamespace()+"/'force_filtering_coeff' should be less than 1, set 1");
      m_force_filter_coeff=1;
    }
    if (m_force_filter_coeff<0)
    {
      ROS_WARN_STREAM(m_controller_nh.getNamespace()+"/'force_filtering_coeff' should be greater than 1, set 0 (NO FORCE ESTIMATION)");
      m_force_filter_coeff=0;
    }
    
    if (!m_controller_nh.getParam("torque_filtering_coeff", m_torque_filter_coeff))
    {
      ROS_WARN_STREAM(m_controller_nh.getNamespace()+"/'torque_filtering_coeff' does not exist, set 0.9");
      m_torque_filter_coeff=0.9;
    }
    if (m_torque_filter_coeff>1)
    {
      ROS_WARN_STREAM(m_controller_nh.getNamespace()+"/'torque_filtering_coeff' should be less than 1, set 1");
      m_torque_filter_coeff=1;
    }
    if (m_torque_filter_coeff<0)
    {
      ROS_WARN_STREAM(m_controller_nh.getNamespace()+"/'torque_filtering_coeff' should be greater than 1, set 0 (NO TORQUE ESTIMATION)");
      m_torque_filter_coeff=0;
    }
    
    if (!m_controller_nh.getParam("acc_filtering_coeff", m_filter))
    {
      ROS_WARN_STREAM(m_controller_nh.getNamespace()+"/'acc_filtering_coeff' does not exist, set 0.9");
      m_filter=0.9;
    }
    if (m_filter>1)
    {
      ROS_WARN_STREAM(m_controller_nh.getNamespace()+"/'acc_filtering_coeff' should be less than 1, set 1");
      m_filter=1;
    }
    if (m_filter<0)
    {
      ROS_WARN_STREAM(m_controller_nh.getNamespace()+"/'acc_filtering_coeff' should be greater than 1, set 0 (NO TORQUE ESTIMATION)");
      m_filter=0;
    }
    
  

    
    
    m_q.resize(m_nAx);
    m_Dq.resize(m_nAx);
    m_DDq.resize(m_nAx);
    m_Dq_filt.resize(m_nAx);
    m_torque.resize(m_nAx);
    m_torque_model.resize(m_nAx);
    m_torque_offset.resize(m_nAx);
    m_external_torque_filt.resize(m_nAx);
    m_external_wrench_filt.resize(6);
    m_external_wrench_vect_tool_in_base.resize(6);
    
    m_q.setZero();
    m_Dq.setZero();
    m_DDq.setZero();
    m_Dq_filt.setZero();
    m_torque.setZero();
    m_torque_model.setZero();
    m_external_wrench_vect_tool_in_base.setZero();
    m_external_wrench_filt.setZero();
    m_external_torque_filt.setZero();
    m_torque_offset.setZero();
    m_lambda=1e-1;
    
    
    for (unsigned int idx = 0;idx <m_joint_names.size();idx++)
    {
      std::string component_type;
      if (m_root_nh.getParam( "/"+model.getName()+"/"+m_joint_names.at(idx)+"/spring/type", component_type))
      {
        if (!component_type.compare("Ideal"))
        {
          ROS_DEBUG("JOINT '%s' has a spring component", m_joint_names.at(idx).c_str());
          m_components.push_back(itia::dynamics::ComponentPtr(new itia::dynamics::IdealSpring(m_joint_names.at(idx), model.getName()) ));
        }
      }
      
      if (m_root_nh.getParam( "/"+model.getName()+"/"+m_joint_names.at(idx)+"/friction/type", component_type))
      {
        
        if (!component_type.compare("Polynomial1"))
        {
          ROS_DEBUG("JOINT '%s' has a Polynomial1 component", m_joint_names.at(idx).c_str());
          m_components.push_back( itia::dynamics::ComponentPtr(new itia::dynamics::FirstOrderPolynomialFriction( m_joint_names.at(idx), model.getName() ) ));
        } 
        else if (!component_type.compare("Polynomial2"))
        {
          ROS_DEBUG("JOINT '%s' has a Polynomial2 component", m_joint_names.at(idx).c_str());
          m_components.push_back(itia::dynamics::ComponentPtr(new itia::dynamics::SecondOrderPolynomialFriction(m_joint_names.at(idx), model.getName()) ));
        } 
      }      
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

void VirtualSensor::starting(const ros::Time& time)
{
  m_is_configured=false;
  ROS_INFO("Controller '%s%s%s' well started",GREEN,m_controller_nh.getNamespace().c_str(),RESET);
  m_start_time=ros::Time::now();
}

void VirtualSensor::stopping(const ros::Time& time)
{
  ROS_INFO("[ %s%s%s ] Stopping controller", BOLDGREEN, m_controller_nh.getNamespace().c_str(), RESET);
  std::fill(m_hw->m_joint_target_msgs->effort.begin(),m_hw->m_joint_target_msgs->effort.end(),0.0);
  m_hw->m_joint_target_msgs->header.stamp = ros::Time::now();
}

void VirtualSensor::update(const ros::Time& time, const ros::Duration& period)
{
  m_queue.callAvailable();
  double t=(ros::Time::now()-m_start_time).toSec();

  if (m_is_configured)
  {
    
    m_q=Eigen::VectorXd::Map(&(m_target_js_rec.getData().position.at(0)),m_target_js_rec.getData().position.size());
    m_Dq=Eigen::VectorXd::Map(&(m_target_js_rec.getData().velocity.at(0)),m_target_js_rec.getData().velocity.size());
    m_torque=Eigen::VectorXd::Map(&(m_target_js_rec.getData().effort.at(0)),m_target_js_rec.getData().effort.size());
    m_Dq_filt=m_filter*m_Dq_filt+(1-m_filter)*m_Dq;
    m_DDq=(1-m_filter)/period.toSec()*(m_Dq-m_Dq_filt);
    
    m_torque_model = m_chain->getJointTorque(m_q, m_Dq, m_DDq)+m_torque_offset;
    for (unsigned int iC=0;iC<m_components.size();iC++)
      m_torque_model += m_components.at(iC)->getTorque(m_q, m_Dq, m_DDq);
    
    Eigen::MatrixXd J = m_chain->getJacobian(m_q);
    Eigen::MatrixXd W = m_lambda * Eigen::MatrixXd::Identity(m_nAx,m_nAx);
    
    m_external_torque_filt=m_torque_filter_coeff*m_external_torque_filt+(1-m_torque_filter_coeff)*(m_torque_model-m_torque);
    
    
    m_extra_torque.reset(new sensor_msgs::JointState);
    m_extra_torque->position.resize(m_nAx);
    m_extra_torque->velocity.resize(m_nAx);
    m_extra_torque->effort.resize(m_nAx);
    m_extra_torque->name=m_joint_names;
    std::fill(m_extra_torque->position.begin(),m_extra_torque->position.end(),0.0);
    std::fill(m_extra_torque->velocity.begin(),m_extra_torque->velocity.end(),0.0);
    
    for (unsigned int iAx=0;iAx<m_nAx;iAx++)
    {
      m_hw->m_joint_target_msgs->effort.at(iAx)=m_torque_model(iAx);
      m_extra_torque->effort.at(iAx)= m_external_torque_filt(iAx); // POSITIVE IF A POSITIVE EXTERNAL TORQUE IS APPLIED
    }
    
    // Dq'*tau = T'*W
    // T       = J*Dq
    // Dq'*tau = Dq'*J'*W
    // tau     = J'*W
    // J*tau     = J*J'*W
    // W= (J*J')*J*tau + N*u -> N null space, u arbitrary
    // N*u = (l*Wlast-W) -> u=N\(l*Wlast-W) -> l forgetting factor
    // W= (J*J')*J*tau + N*(N\(l*Wlast-W))
    
//      m_external_wrench_vect_tool_in_base = (J*J.transpose()+W).inverse()*J*(m_torque-m_torque_model);
    
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(m_chain->getJacobian(m_q).transpose(), Eigen::ComputeThinU | Eigen::ComputeThinV);
    int rank = 0;
    while ( std::abs(svd.singularValues()(rank))>(1e-2*std::abs(svd.singularValues()(0))) )
    {
      rank++;
      if (rank == (svd.matrixV().rows()))
        break;
    }
    
    if ((svd.matrixV().cols()-rank)>0)
    {
      Eigen::MatrixXd null_space;
      Eigen::VectorXd external_wrench_vect = -svd.solve(m_torque-m_torque_model);
      null_space = svd.matrixV().block(0, rank, svd.matrixV().rows(),  svd.matrixV().cols()-rank);
      Eigen::JacobiSVD<Eigen::MatrixXd> svd_null(null_space, Eigen::ComputeThinU | Eigen::ComputeThinV);
      m_external_wrench_vect_tool_in_base=external_wrench_vect+null_space*svd_null.solve(m_forgetting_factor*m_external_wrench_vect_tool_in_base-external_wrench_vect );
    }
    else
      m_external_wrench_vect_tool_in_base = -svd.solve(m_torque-m_torque_model);
    
    
    Eigen::Affine3d Tbf=m_chain->getTransformation(m_q);
    Eigen::Affine3d Tbt=Tbf*m_Tft;
    
    m_external_wrench_vect_tool_in_base.block(0,0,3,1)-=  m_mass*grav;
    m_external_wrench_vect_tool_in_base.block(3,0,3,1)-=  ((Eigen::Vector3d)(Tbf.linear()*m_center_of_mass)).cross(m_mass*grav);
    
    
    Eigen::Vector6d external_wrench;
    if (m_base_is_reference)

      external_wrench = itia::dynamics::spatialRotation(m_external_wrench_vect_tool_in_base,m_Tub.linear()); 
    else
    {
      external_wrench = itia::dynamics::spatialRotation(m_external_wrench_vect_tool_in_base,Tbf.linear().inverse()); 
      external_wrench = itia::dynamics::spatialDualTranformation(external_wrench,m_Tft.inverse()); 
    } 
    
    m_external_wrench_filt=m_force_filter_coeff*m_external_wrench_filt+(1-m_force_filter_coeff)*external_wrench;
    
    
    
    
    
    m_extra_wrench.reset(new geometry_msgs::WrenchStamped);
    m_extra_wrench->wrench.force.x=m_external_wrench_filt(0);
    m_extra_wrench->wrench.force.y=m_external_wrench_filt(1);
    m_extra_wrench->wrench.force.z=m_external_wrench_filt(2);
    
    m_extra_wrench->wrench.torque.x=m_external_wrench_filt(3);
    m_extra_wrench->wrench.torque.y=m_external_wrench_filt(4);
    m_extra_wrench->wrench.torque.z=m_external_wrench_filt(5);
    m_extra_wrench->header.frame_id=m_tool_frame;
    
    m_hw->m_joint_target_msgs->header.stamp = ros::Time::now();
    m_extra_torque->header.stamp=m_hw->m_joint_target_msgs->header.stamp;
    m_extra_wrench->header.stamp=m_hw->m_joint_target_msgs->header.stamp;
    
    m_extra_wrench_pub.publish(m_extra_wrench);
    m_extra_torque_pub.publish(m_extra_torque);
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

bool VirtualSensor::resetOffset( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res )
{
  m_torque_offset-=m_external_torque_filt;
  return true;
}

bool VirtualSensor::configureTool ( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res )
{
  std::string base_frame;
  std::string tool_frame;
  
  if (m_root_nh.getParam("/env_param/TOOLDATA/mass",m_mass))
  {
    m_mass=0;
    ROS_WARN_STREAM("/env_param/TOOLDATA/mass not set, assume 0");
  }
  
  double tmp;
  if (m_root_nh.getParam("/env_param/TOOLDATA/x",tmp))
  {
    tmp=0;
    ROS_WARN_STREAM("/env_param/TOOLDATA/x not set, assume 0");
  }
  m_center_of_mass(0)=tmp;
  
  if (m_root_nh.getParam("/env_param/TOOLDATA/y",tmp))
  {
    tmp=0;
    ROS_WARN_STREAM("/env_param/TOOLDATA/y not set, assume 0");
  }
  m_center_of_mass(1)=tmp;
  
  if (m_root_nh.getParam("/env_param/TOOLDATA/z",tmp))
  {
    tmp=0;
    ROS_WARN_STREAM("/env_param/TOOLDATA/z not set, assume 0");
  }
  m_center_of_mass(2)=tmp;
  
  
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
