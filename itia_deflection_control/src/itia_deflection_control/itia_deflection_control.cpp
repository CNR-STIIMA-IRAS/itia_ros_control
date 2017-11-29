#include <itia_deflection_control/itia_deflection_control.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(itia::control::DeflectionControl, controller_interface::ControllerBase);


namespace itia
{
	namespace control
	{
		
		
		DeflectionControl::~DeflectionControl()
		{
			
		}
		
		bool DeflectionControl::init(itia::control::PosVelEffJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
		{
			m_root_nh = root_nh;
			m_controller_nh = controller_nh;
			m_hw = hw;
			
			m_controller_nh.setCallbackQueue(&m_queue);
			
			std::string feedback_topic_name;
			std::string setpoint_topic_name;
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
			m_js_ffw_sub = m_controller_nh.subscribe<sensor_msgs::JointState>(feedforward_topic_name, 1, &itia::rutils::MsgReceiver<sensor_msgs::JointState>::callback, &m_ffw_js_rec);
			
      
      m_joint_names=m_hw->getNames();
      m_nAx = m_joint_names.size();
			
      ROS_INFO("Controller '%s%s%s' controls the following joints:",GREEN,m_controller_nh.getNamespace().c_str(),RESET);
			for (unsigned int idx=0;idx<m_nAx;idx++)
				ROS_INFO(" - %s%s%s",GREEN,m_joint_names.at(idx).c_str(),RESET);
			
			m_deflection_controllers.resize(m_nAx);
      m_link_pos_filters.resize(m_nAx);
      m_link_vel_filters.resize(m_nAx);
      m_tau_el_filters.resize(m_nAx);
      m_target_tau_el_filters.resize(m_nAx);
      m_target_Dtau_el_filters.resize(m_nAx);
      m_torque_minimum_error.resize(m_nAx);
      
			m_elasticity.resize(m_nAx);
			m_max_torque.resize(m_nAx);
      
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
				double k;
				if (!m_controller_nh.getParam(m_joint_names.at(i)+"/elasticity",k))
				{
					ROS_ERROR("cannot find '%s' parameter!",(m_controller_nh.getNamespace()+"/"+m_joint_names.at(i)+"/elasticity").c_str());
					return false;
				}
				m_elasticity.at(i)=k;
				
				double security_max_elastic_torque;
				if (!m_controller_nh.getParam(m_joint_names.at(i)+"/security_max_elastic_torque",security_max_elastic_torque))
				{
					ROS_ERROR("cannot find '%s' parameter!",(m_controller_nh.getNamespace()+"/"+m_joint_names.at(i)+"/security_max_elastic_torque").c_str());
					return false;
				}
				m_max_torque.at(i)=security_max_elastic_torque;
				
        double torque_minimum_error;
        if (!m_controller_nh.getParam(m_joint_names.at(i)+"/torque_minimum_error",torque_minimum_error))
        {
          ROS_WARN("no torque_minimum_error specified for joint %s, set equal to zero",m_joint_names.at(i).c_str());
          torque_minimum_error=0;
        }
        m_torque_minimum_error.at(i)=torque_minimum_error;
        
        ROS_INFO("[ %s%s%s ] Creating controller objects", BOLDGREEN, m_controller_nh.getNamespace().c_str(), RESET);
        
        Eigen::VectorXd init_cond(1);
        init_cond(0)=0;
        std::string name;
        
        name = m_joint_names.at(i)+"/deflection";
        m_deflection_controllers.at(i)=itia::control::createDiscreteStateSpareFromParam(m_controller_nh, name,init_cond);
				
        name = m_joint_names.at(i)+"/link_pos_filter";
        m_link_pos_filters.at(i)=itia::control::createDiscreteStateSpareFromParam(m_controller_nh, name,init_cond);
        
        name = m_joint_names.at(i)+"/link_vel_filter";
        m_link_vel_filters.at(i)=itia::control::createDiscreteStateSpareFromParam(m_controller_nh, name,init_cond);
        
        name = m_joint_names.at(i)+"/tau_elastic_filter";
        m_tau_el_filters.at(i)=itia::control::createDiscreteStateSpareFromParam(m_controller_nh, name,init_cond);
        
        name = m_joint_names.at(i)+"/target_tau_elastic_filter";
        m_target_tau_el_filters.at(i)=itia::control::createDiscreteStateSpareFromParam(m_controller_nh, name,init_cond);
        
        name = m_joint_names.at(i)+"/target_Dtau_elastic_filter";
        m_target_Dtau_el_filters.at(i)=itia::control::createDiscreteStateSpareFromParam(m_controller_nh, name,init_cond);
        
			}
			
			ROS_INFO("Controller '%s%s%s' well initialized",GREEN,m_controller_nh.getNamespace().c_str(),RESET);
			return true;
			
		}
		
		void DeflectionControl::starting(const ros::Time& time)
		{
			m_configured = false;  			
			m_feedforward_active=false;
			m_is_in_error=false;
			
			m_queue.callAvailable();
			std::vector<double> elastic_joint_positions  = m_fb_js_rec.getData().position;
      std::vector<double> elastic_joint_velocities = m_fb_js_rec.getData().velocity;
      for (int i = 0;i<m_nAx;i++)
			{
				Eigen::VectorXd init_torque(1);
				init_torque(0)=-elastic_joint_positions.at(2*i+1)*m_elasticity.at(i);
				
        Eigen::VectorXd initial_state = m_tau_el_filters.at(i)->getInitializationMatrix()*init_torque;
        m_tau_el_filters.at(i)->setState(initial_state);
        
        initial_state = m_target_tau_el_filters.at(i)->getInitializationMatrix()*init_torque;
        m_target_tau_el_filters.at(i)->setState(initial_state);
                
        initial_state = m_target_Dtau_el_filters.at(i)->getInitializationMatrix()*init_torque;
        m_target_Dtau_el_filters.at(i)->setState(initial_state);
        
        initial_state = m_deflection_controllers.at(i)->getInitializationMatrix()*init_torque;
        m_deflection_controllers.at(i)->setState(initial_state);
        
        
        Eigen::VectorXd init_pos(1);
        init_pos(0)=elastic_joint_positions.at(2*i)+elastic_joint_positions.at(2*i+1);
        initial_state = m_link_pos_filters.at(i)->getInitializationMatrix()*init_pos;
        m_link_pos_filters.at(i)->setState(initial_state);
        
        Eigen::VectorXd init_vel(1);
        init_vel(0)=elastic_joint_velocities.at(2*i)+elastic_joint_velocities.at(2*i+1);
        initial_state = m_link_pos_filters.at(i)->getInitializationMatrix()*init_vel;
        m_link_vel_filters.at(i)->setState(initial_state);
        
        
			}
		}
		
		void DeflectionControl::stopping(const ros::Time& time)
		{
			ROS_INFO("[ %s%s%s ] Stopping controller", BOLDGREEN, m_controller_nh.getNamespace().c_str(), RESET);
			m_configured = false;
			m_feedforward_active=false;
			for (int iAx = 0;iAx<m_nAx;iAx++)
			{
				m_hw->m_joint_target_msgs->position.at(iAx) = m_fb_js_rec.getData().position.at(2*iAx);
				m_hw->m_joint_target_msgs->velocity.at(iAx) = 0;
// 				m_hw->m_joint_target_msgs->effort.at(iAx)   = 0;
			}
			m_hw->m_joint_target_msgs->header.stamp = ros::Time::now();
		}
		
		void DeflectionControl::update(const ros::Time& time, const ros::Duration& period)
		{
			m_queue.callAvailable();
			
			m_configured = m_configured || (m_target_js_rec.isANewDataAvailable());
			m_feedforward_active=m_feedforward_active || (m_ffw_js_rec.isANewDataAvailable());
			
			Eigen::VectorXd defl_controller_in(2);
      Eigen::VectorXd defl_controller_out(1);
      
			Eigen::VectorXd link_pos_filter_in(1);
			Eigen::VectorXd link_pos_filter_out(1);
			
			Eigen::VectorXd link_vel_filter_in(1);
      Eigen::VectorXd link_vel_filter_out(1);
      
      Eigen::VectorXd tau_el_filter_in(1);
      Eigen::VectorXd tau_el_filter_out(1);
      
      Eigen::VectorXd target_tau_el_filter_in(1);
      Eigen::VectorXd target_tau_el_filter_out(1);
      
      Eigen::VectorXd target_Dtau_el_filter_in(1);
      Eigen::VectorXd target_Dtau_el_filter_out(1);
      
      
			for (int iAx = 0;iAx<m_nAx;iAx++)
			{
				
				if (m_configured)
          target_tau_el_filter_in(0)=m_target_js_rec.getData().effort.at(iAx);
        else
          target_tau_el_filter_in(0)=m_target_tau_el_filters.at(iAx)->getOutput()(0);
        
        tau_el_filter_in(0)=-m_fb_js_rec.getData().position.at(2*iAx+1)*m_elasticity.at(iAx);
//         link_pos_filter_in(0)=m_fb_js_rec.getData().position.at(2*iAx)+m_fb_js_rec.getData().position.at(2*iAx+1);
//         link_vel_filter_in(0)=m_fb_js_rec.getData().velocity.at(2*iAx)+m_fb_js_rec.getData().velocity.at(2*iAx+1);
        link_pos_filter_in(0)= m_target_js_rec.getData().position.at(iAx);
        link_vel_filter_in(0)= m_target_js_rec.getData().velocity.at(iAx);
        
        target_tau_el_filter_out=m_target_tau_el_filters.at(iAx)->update(target_tau_el_filter_in);
        tau_el_filter_out=m_tau_el_filters.at(iAx)->update(tau_el_filter_in);
        link_pos_filter_out=m_link_pos_filters.at(iAx)->update(link_pos_filter_in);
        link_vel_filter_out=m_link_vel_filters.at(iAx)->update(link_vel_filter_in);
        
        defl_controller_in(0) = target_tau_el_filter_out(0)-tau_el_filter_out(0);
        defl_controller_in(1) = defl_controller_in(0);
        if (std::abs(defl_controller_in(1))<m_torque_minimum_error.at(iAx))
          defl_controller_in(1)=0;
        
        defl_controller_out=m_deflection_controllers.at(iAx)->update(defl_controller_in)+target_tau_el_filter_out;
        
        target_Dtau_el_filter_in=target_tau_el_filter_in;
        target_Dtau_el_filter_out= m_target_Dtau_el_filters.at(iAx)->update(target_Dtau_el_filter_in);
        
        m_hw->m_joint_target_msgs->position.at(iAx) = link_pos_filter_out(0) + target_tau_el_filter_out(0)/m_elasticity.at(iAx);
        m_hw->m_joint_target_msgs->velocity.at(iAx) = link_vel_filter_out(0) + (target_Dtau_el_filter_out(0)+defl_controller_out(0))/m_elasticity.at(iAx);
        m_hw->m_joint_target_msgs->effort.at(iAx)   = defl_controller_out(0);
        
// 				if (m_feedforward_active)
// 				{
// 					m_hw->m_joint_target_msgs->position.at(iAx) += m_ffw_js_rec.getData().position.at(iAx);
// 					m_hw->m_joint_target_msgs->velocity.at(iAx) += m_ffw_js_rec.getData().velocity.at(iAx);
// 					m_hw->m_joint_target_msgs->effort.at(iAx)   += m_ffw_js_rec.getData().effort.at(iAx);
// 				}
			}
			if (m_is_in_error)
			{
				ROS_FATAL_THROTTLE(4,"TOO MUCH DEFLECTION, RESTART CONTROLLER");
				std::fill(m_hw->m_joint_target_msgs->velocity.begin(),m_hw->m_joint_target_msgs->velocity.end(),0.0);
				std::fill(m_hw->m_joint_target_msgs->effort.begin(),m_hw->m_joint_target_msgs->effort.end(),0.0);
				m_hw->m_joint_target_msgs->header.stamp = ros::Time(0);
			}
			else
				m_hw->m_joint_target_msgs->header.stamp = ros::Time::now();
			
			
		}
		
		
	}
}
