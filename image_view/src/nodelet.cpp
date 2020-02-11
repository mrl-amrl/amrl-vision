#include <image_view/image_view.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace amrl_image_view
{

class image_view : public nodelet::Nodelet
{
private:
  ImageView *impl;

public:
  image_view() : impl(0) {}
  virtual ~image_view() { delete impl; }

private:
  void onInit()
  {
    impl = new ImageView(getNodeHandle(), getPrivateNodeHandle());
  }
};

} // namespace amrl_vision_aggregator

PLUGINLIB_EXPORT_CLASS(amrl_image_view::image_view, nodelet::Nodelet)