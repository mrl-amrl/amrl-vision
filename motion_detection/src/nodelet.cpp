#include <motion-detection/motion_detection.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace amrl_motion_detection
{

class motion_detection : public nodelet::Nodelet
{
private:
  MotionDetection *impl;

public:
  motion_detection() : impl(0) {}
  virtual ~motion_detection() { delete impl; }

private:
  void onInit()
  {
    impl = new MotionDetection(&getPrivateNodeHandle());
  }
};

} // namespace amrl_motion_detection

PLUGINLIB_EXPORT_CLASS(amrl_motion_detection::motion_detection, nodelet::Nodelet)