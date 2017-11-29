#include <itia_model_feedforward/itia_model_feedforward.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(itia::control::FeedForwardControl, controller_interface::ControllerBase);


namespace itia
{
  namespace control
  {
    
    
    FeedForwardControl::~FeedForwardControl()
    {
      
    }
    
    bool FeedForwardControl::init(itia::control::PosVelEffJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
    {
      m_root_nh = root_nh;
      m_controller_nh = controller_nh;
      m_hw = hw;
      
      m_controller_nh.setCallbackQueue(&m_queue);
      
      try{
        
        std::string setpoint_topic_name;
        if (!m_controller_nh.getParam("setpoint_topic_name", setpoint_topic_name))
        {
          ROS_FATAL_STREAM(m_controller_nh.getNamespace()+"/'setpoint_topic_name' does not exist");
          ROS_FATAL("ERROR DURING INITIALIZATION CONTROLLER '%s'", m_controller_nh.getNamespace().c_str());
          return false;
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
        if (!m_controller_nh.getParam("transient_time", m_transient_time))
        {
          ROS_FATAL_STREAM(m_controller_nh.getNamespace()+"/'transient_time' does not exist");
          ROS_FATAL("ERROR DURING INITIALIZATION CONTROLLER '%s'", m_controller_nh.getNamespace().c_str());
          return false;
        }
        
        if (!m_controller_nh.getParam("acc_filtering_coeff", m_filter))
        {
          ROS_WARN_STREAM(m_controller_nh.getNamespace()+"/'acc_filtering_coeff' does not exist, set 1.0 (NO ACCELERATION TERM)");
          m_filter=1.0;
        }
        if (m_filter>1)
        {
          ROS_WARN_STREAM(m_controller_nh.getNamespace()+"/'acc_filtering_coeff' should be less than 1, set 1 (NO ACCELERATION TERM)");
          m_filter=1;
        }
        if (m_filter<0)
        {
          ROS_WARN_STREAM(m_controller_nh.getNamespace()+"/'acc_filtering_coeff' should be greater than 1, set 0");
          m_filter=0;
        }
        
        m_js_target_sub = m_controller_nh.subscribe<sensor_msgs::JointState>(setpoint_topic_name,    1, &itia::rutils::MsgReceiver<sensor_msgs::JointState>::callback, &m_target_js_rec);
        
        m_joint_names=m_hw->getNames();
        m_nAx = m_joint_names.size();
        
        ROS_INFO("Controller '%s%s%s' controls the following joints:",GREEN,m_controller_nh.getNamespace().c_str(),RESET);
        for (unsigned int idx=0;idx<m_nAx;idx++)
          ROS_INFO(" - %s%s%s",GREEN,m_joint_names.at(idx).c_str(),RESET);
        
        
        
        urdf::Model model;
        if (!model.initParam(m_controller_nh.getNamespace()+"/robot_description"))
        {
          ROS_ERROR("Urdf robot_description '%s' does not exist",(m_controller_nh.getNamespace()+"/robot_description").c_str());
          return false;
        }
        Eigen::Vector3d grav;
        grav << 0, 0, -9.806;
        
        boost::shared_ptr<itia::dynamics::Link> root_link(new itia::dynamics::Link());  
        root_link->fromUrdf(model.root_link_);
        m_chain.reset(new itia::dynamics::Chain(root_link, base_frame,tool_frame, grav));
        m_chain->setInputJointsName(m_joint_names);
       
        
        m_q.resize(m_nAx);
        m_Dq.resize(m_nAx);
        m_Dq_filt.resize(m_nAx);
        m_DDq.resize(m_nAx);
        m_torque.resize(m_nAx);
        m_q.setZero();
        m_Dq.setZero();
        m_DDq.setZero();
        m_torque.setZero();
      }
      catch(const  std::exception& e)
      {
        ROS_FATAL("EXCEPTION: %s", e.what());
        return false;
      }
      ROS_INFO("Controller '%s%s%s' well initialized",GREEN,m_controller_nh.getNamespace().c_str(),RESET);
      
      return true;
      
    }
    
    void FeedForwardControl::starting(const ros::Time& time)
    {
      m_is_configured=false;
      ROS_INFO("Controller '%s%s%s' well started",GREEN,m_controller_nh.getNamespace().c_str(),RESET);
      m_start_time=ros::Time::now();
    }
    
    void FeedForwardControl::stopping(const ros::Time& time)
    {
      ROS_INFO("[ %s%s%s ] Stopping controller", BOLDGREEN, m_controller_nh.getNamespace().c_str(), RESET);
      std::fill(m_hw->m_joint_target_msgs->effort.begin(),m_hw->m_joint_target_msgs->effort.end(),0.0);
      m_hw->m_joint_target_msgs->header.stamp = ros::Time::now();
    }
    
    void FeedForwardControl::update(const ros::Time& time, const ros::Duration& period)
    {
      m_queue.callAvailable();
      double t=(ros::Time::now()-m_start_time).toSec();
      double scale=t/m_transient_time;
      
      if ( t>m_transient_time )
        scale=1.0;
      
      if (m_is_configured)
      {
        
        for (int iAx=0;iAx<m_nAx;iAx++)
        {
          m_q(iAx)   = m_target_js_rec.getData().position.at(iAx);
          m_Dq(iAx)  = m_target_js_rec.getData().velocity.at(iAx);
          m_DDq(iAx) = 0.0;//m_target_js_rec.getData().effort.at(iAx);
        }
        
        m_Dq_filt=m_filter*m_Dq_filt+(1-m_filter)*m_Dq;
        m_DDq=(1-m_filter)/period.toSec()*(m_Dq-m_Dq_filt);
        
        m_torque = m_chain->getJointTorque(m_q, m_Dq, m_DDq);
  
        m_hw->m_joint_target_msgs->position = m_target_js_rec.getData().position;
        m_hw->m_joint_target_msgs->velocity = m_target_js_rec.getData().velocity;
        
        for (unsigned int iAx=0;iAx<m_nAx;iAx++)
          m_hw->m_joint_target_msgs->effort.at(iAx)=m_torque(iAx)*scale;
        
        m_hw->m_joint_target_msgs->header.stamp = ros::Time::now();
      }
      else
      {
        m_is_configured=m_target_js_rec.isANewDataAvailable();
        if (m_is_configured)
          ROS_INFO("configured");
      }
      
    }
    
    
  }
}
