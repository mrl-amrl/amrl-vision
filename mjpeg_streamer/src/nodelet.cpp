#include <mjpeg_streamer/mjpeg_streamer.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace amrl_mjpeg_streamer
{

class mjpeg_streamer_nodelet : public nodelet::Nodelet
{
private:
  MJPEGStreamer *impl;

public:
  mjpeg_streamer_nodelet() : impl(0) {}
  virtual ~mjpeg_streamer_nodelet() { delete impl; }

private:
  void onInit()
  {
    impl = new MJPEGStreamer(getNodeHandle(), getPrivateNodeHandle());
  }
};

} // namespace amrl_mjpeg_streamer

PLUGINLIB_EXPORT_CLASS(amrl_mjpeg_streamer::mjpeg_streamer_nodelet, nodelet::Nodelet)