
#include "qrcode_detection/qrcode_reader.h"
#include <Magick++.h>
#include <zbar.h>
#include <iostream>
#include <ros/ros.h>

namespace barcode
{

BarcodeReader::~BarcodeReader()
{
}

void BarcodeReader::updateSensorSize()
{
  sensor_size_ = 2 * tan(camera_horizontal_fov_deg_ / 180.0 * M_PI / 2);
}

BarcodeReader &BarcodeReader::setFOV(double fov)
{
  camera_horizontal_fov_deg_ = fov;
  updateSensorSize();
  return *this;
}

BarcodeReader &BarcodeReader::setBarcodeSize(double size)
{
  barcode_size_ = size;
  updateSensorSize();
  return *this;
}

int BarcodeReader::parse(const cv_bridge::CvImageConstPtr &cv_img_ptr)
{
  zbar::ImageScanner scanner;
  scanner.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);

  int width = cv_img_ptr->image.cols;
  int height = cv_img_ptr->image.rows;

  Magick::Blob blob(cv_img_ptr->image.ptr(0), width * height);

  zbar::Image image(width, height, "Y800", blob.data(), width * height);

  int n = scanner.scan(image);
  barcodes.clear();
  if (n > 0)
  {
    zbar::Image::SymbolIterator symbol = image.symbol_begin();
    for (; symbol != image.symbol_end(); ++symbol)
    {
      if (symbol->get_location_size() == 4)
      {
        Barcode barcode;
        barcode.data = symbol->get_data();

        // Get center coordinate by averaging four corners
        for (int i = 0; i < symbol->get_location_size(); i++)
        {
          barcode.x += symbol->get_location_x(i);
          barcode.y += symbol->get_location_y(i);
        }

        for (int i = 0; i < 4; i++)
        {
          barcode::Point p;
          p.x = symbol->get_location_x(i) / double(width);
          p.y = symbol->get_location_y(i) / double(height);
          barcode.corners[i] = p;
        }

        barcode.x /= symbol->get_location_size();
        barcode.y /= symbol->get_location_size();

        // Get diagonal length in pixels
        int dx0 = symbol->get_location_x(0) - symbol->get_location_x(2);
        int dx1 = symbol->get_location_x(1) - symbol->get_location_x(3);
        int dy0 = symbol->get_location_y(0) - symbol->get_location_y(2);
        int dy1 = symbol->get_location_y(1) - symbol->get_location_y(3);
        double d0 = dx0 * dx0 + dy0 * dy0;
        double d1 = dx1 * dx1 + dy1 * dy1;
        double diagonal_size = (sqrt(d0) + sqrt(d1)) / 2.0;

        // Calculate depth
        double sensor_resolution = sensor_size_ / width;
        double projection_ratio = M_SQRT2 * barcode_size_ / diagonal_size;

        barcode.z = projection_ratio / sensor_resolution;
        barcode.x = (barcode.x - width / 2) * projection_ratio;
        barcode.y = (barcode.y - height / 2) * projection_ratio;
        barcodes.push_back(barcode);
      }
    }
  }
  return n;
}

std::vector<Barcode> &barcode::BarcodeReader::getBarcodes()
{
  return barcodes;
}

} /* namespace barcode */
