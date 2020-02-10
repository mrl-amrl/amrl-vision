#include <hazmat_detection/hazmat_detection.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace amrl_hazmat_detection
{

class hazmat_detection : public nodelet::Nodelet
{
private:
  HazmatDetection *impl;

public:
  hazmat_detection() : impl(0) {}
  virtual ~hazmat_detection() { delete impl; }

private:
  void onInit()
  {
    impl = new HazmatDetection(getNodeHandle(), getPrivateNodeHandle());
  }
};

} // namespace amrl_hazmat_detection

PLUGINLIB_EXPORT_CLASS(amrl_hazmat_detection::hazmat_detection, nodelet::Nodelet)