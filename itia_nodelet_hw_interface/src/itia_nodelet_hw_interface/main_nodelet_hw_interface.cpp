#include <itia_nodelet_hw_interface/main_nodelet_hw_interface.h>
#include <pluginlib/class_list_macros.h>
# include <diagnostic_msgs/DiagnosticArray.h>

PLUGINLIB_EXPORT_CLASS(itia::control::JointStatesNodeletHwInterface, nodelet::Nodelet) 


namespace itia
{
namespace control
{
void JointStatesNodeletHwInterface::onInit()
{
  ROS_INFO("[JointStatesNodeletHwInterface %s]", getPrivateNodeHandle().getNamespace().c_str());
  m_stop = false;
  m_main_thread = std::thread(&itia::control::JointStatesNodeletHwInterface::main_thread, this);
  
};

JointStatesNodeletHwInterface::~JointStatesNodeletHwInterface()
{
  ROS_DEBUG("[JointStatesNodeletHwInterface: %s] shutting down...", getPrivateNodeHandle().getNamespace().c_str());
  m_stop = true;
  if (m_main_thread.joinable())
    m_main_thread.join();
  ROS_INFO("[JointStatesNodeletHwInterface: %s] shutted down...", getPrivateNodeHandle().getNamespace().c_str());
  m_stop = false;
};

void JointStatesNodeletHwInterface::main_thread()
{
  ros::NodeHandle nh = getPrivateNodeHandle();
  ros::Publisher  m_diagnostics_pub = nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics",1);
  double sampling_period=0.01;
  if (!nh.getParam("nodelet_hardware_interface/sampling_period", sampling_period))
  {
    ROS_WARN_STREAM(nh.getNamespace()+"/'nodelet_hardware_interface/sampling_period' does not exist, set equal to 0.001");
    sampling_period = 1.0e-3;
  }
  
  std::vector<std::string> joint_names;
  if (!nh.getParam("/controlled_joint_names", joint_names))
  {
    ROS_FATAL_STREAM(nh.getNamespace()+"/'nodelet_hardware_interface/joint_names' does not exist");
    ROS_FATAL("ERROR DURING STARTING HARDWARE INTERFACE '%s'", nh.getNamespace().c_str());
    return;
  }
  
  std::string topic_name;
  if (!nh.getParam("nodelet_hardware_interface/topic_name", topic_name))
  {
    ROS_WARN_STREAM(nh.getNamespace()+"/'nodelet_hardware_interface/topic_name' does not exist, set equal to '/joint_target");
    topic_name = "/joint_target";
  }
  
  
  ros::Duration period(sampling_period);
  ros::WallRate loop_rate(1.0/sampling_period);
  
  
  
  ROS_DEBUG("%sSTARTED HARDWARE INTERFACE '%s'%s", BOLDGREEN, nh.getNamespace().c_str(), RESET);
  ROS_DEBUG("Target Topic: '%s%s%s'", GREEN, topic_name.c_str(), RESET);
  ROS_DEBUG("Frequency: '%s%f%s'", GREEN, 1.0/sampling_period, RESET);
  ROS_DEBUG("Controller joints:");
  for (auto& elem: joint_names) 
    ROS_DEBUG("- '%s%s%s'", GREEN, elem.c_str(), RESET);
  
  ros::Publisher pub =nh.advertise<sensor_msgs::JointState>(topic_name, 1);
  itia::control::JointTargetHW hw(pub, joint_names);
  controller_manager::ControllerManager cm(&hw, nh);
  
  long unsigned int n_cycles=0;
  long unsigned int n_delay=0;
  while (ros::ok() && !m_stop)
  {
     hw.read();
     cm.update(ros::Time::now(), period);
     hw.write();
     if (!loop_rate.sleep())
       n_delay++;
     n_cycles++;
     
     if (n_cycles>500)
     {
       double ratio=(double)n_delay/(double)n_cycles;
       if (ratio>0.01)
       {
         ROS_FATAL("[JointStatesNodeletHwInterface %s] desired rate is unmet (%f%%), it is possible to have undesired behaviour, please stop the robot",getPrivateNodeHandle().getNamespace().c_str(),ratio*100);
         diagnostic_msgs::DiagnosticArray diag_msg;
        
        diag_msg.header.stamp=ros::Time::now();
        diag_msg.status.resize(1);
        diag_msg.status.at(0).name=nh.getNamespace();
        diag_msg.status.at(0).hardware_id="FourByThreeRobot";
        diag_msg.status.at(0).level=diagnostic_msgs::DiagnosticStatus::ERROR;
        diag_msg.status.at(0).message="desired rate is unmet, it is possible to have undesired behaviour, please stop the robot";
        diag_msg.status.at(0).values.resize(2);
        
        diag_msg.status.at(0).values.at(0).key="period's violation rate";
        diag_msg.status.at(0).values.at(0).value=std::to_string(ratio*100);

        diag_msg.status.at(0).values.at(1).key="desired rate";
        diag_msg.status.at(0).values.at(1).value=std::to_string(1.0/sampling_period);
        
        m_diagnostics_pub.publish(diag_msg);
       }
     }
     else
       n_delay=0;
     
  }
  return;  

}


}
}