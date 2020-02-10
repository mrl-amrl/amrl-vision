

#ifndef BARCODE_READER_H_
#define BARCODE_READER_H_

#include <cv_bridge/cv_bridge.h>
#include <vector>

namespace barcode
{

struct Point
{
  double x;
  double y;
};

struct Barcode
{
  std::string data;
  double x;
  double y;
  double z;
  Point corners[4];
  Barcode() : x(0),
              y(0),
              z(0)
  {
  }
};

/*
 *
 */
class BarcodeReader
{
private:
  std::vector<Barcode> barcodes;
  // Assuming camera with f=1
  double camera_horizontal_fov_deg_; // Degrees
  double barcode_size_;              // Meters
  double sensor_size_;

  void updateSensorSize();

public:
  BarcodeReader() : camera_horizontal_fov_deg_(60),
                    barcode_size_(0.16)
  {
    updateSensorSize();
  }
  virtual ~BarcodeReader();

  /**
   *
   * @param fov Camera horizontal field of view in degrees.
   * @todo Use camera calibration params
   * @return
   */
  BarcodeReader &setFOV(double fov);

  /**
   *
   * @param size Rectangular barcode size, in meters.
   * We assuming that all the barcodes we encounter would
   * be the same size. Could get rid of this requirement by encoding barcode's
   * size in the barcode itself.
   * @return
   */

  BarcodeReader &setBarcodeSize(double size);

  int parse(const cv_bridge::CvImageConstPtr &cv_img_ptr);

  std::vector<Barcode> &getBarcodes();
};

} /* namespace barcode */
#endif /* BARCODE_READER_H_ */
