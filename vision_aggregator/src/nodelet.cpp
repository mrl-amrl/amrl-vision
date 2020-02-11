#include <vision_aggregator/vision_aggregator.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace amrl_vision_aggregator
{

class vision_aggregator_nodelet : public nodelet::Nodelet
{
private:
  vision_aggregator::VisionAggregator *impl;

public:
  vision_aggregator_nodelet() : impl(0) {}
  virtual ~vision_aggregator_nodelet() { delete impl; }

private:
  void onInit()
  {
    impl = new vision_aggregator::VisionAggregator(getNodeHandle(), getPrivateNodeHandle());
  }
};

} // namespace amrl_vision_aggregator

PLUGINLIB_EXPORT_CLASS(amrl_vision_aggregator::vision_aggregator_nodelet, nodelet::Nodelet)