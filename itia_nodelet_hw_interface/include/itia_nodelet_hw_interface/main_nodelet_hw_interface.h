#ifndef __MAIN_NODELET_HARDWARE_INTERFACE__
#define __MAIN_NODELET_HARDWARE_INTERFACE__

# include <controller_manager/controller_manager.h>
# include <itia_nodelet_hw_interface/nodelet_hw_interface.h>
# include <nodelet/nodelet.h>
# include <thread>

namespace itia
{
namespace control
{
  


class JointStatesNodeletHwInterface : public nodelet::Nodelet
{
public:
  virtual void onInit();

protected:
  std::thread m_main_thread;
  bool m_stop;


  void main_thread();
  ~JointStatesNodeletHwInterface();
};



}
}
# endif