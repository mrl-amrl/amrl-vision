#include <qrcode_detection/qrcode_detection.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace amrl_qrcode_detection
{

class qrcode_detection : public nodelet::Nodelet
{
private:
  Node *impl;

public:
  qrcode_detection() : impl(0) {}
  virtual ~qrcode_detection() { delete impl; }

private:
  void onInit()
  {
    impl = new Node(getNodeHandle(), getPrivateNodeHandle());
  }
};

} // namespace amrl_qrcode_detection

PLUGINLIB_EXPORT_CLASS(amrl_qrcode_detection::qrcode_detection, nodelet::Nodelet)