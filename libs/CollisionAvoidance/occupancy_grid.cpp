/*  ------------------------------------------------------------------
    Copyright 2016 Camille Phiquepal
    email: camille.phiquepal@gmail.com

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */

#include <occupancy_grid.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

void show_img(const std::string & window_name, const cv::Mat & mat)
{
  cv::Mat normalized;
  if(mat.type() == CV_8U)
    normalized = mat;
  else
    cv::normalize(mat, normalized, 0, 1.0, cv::NORM_MINMAX);
  cv::flip(normalized, normalized, 0);
  imshow(window_name, normalized);
}

class SubPixAccessor
{
  int i_, j_;
  float weight_tl_, weight_tr_, weight_bl_, weight_br_;
  bool on_image_;
public:

  SubPixAccessor(int rows, int cols, double cell_size, double x, double y)
  {
    float float_i = 0.5 * rows + y / cell_size;
    float float_j = 0.5 * cols + x / cell_size;

    i_ = int(float_i);
    j_ = int(float_j);

    auto d_i = float_i - i_;
    auto d_j = float_j - j_;

    weight_tl_ = (1.0 - d_j) * (1.0 - d_i);
    weight_tr_ = (d_j)       * (1.0 - d_i);
    weight_bl_ = (1.0 - d_j) * (d_i);
    weight_br_ = (d_j)       * (d_i);

    on_image_= i_ >= 0 && i_ < rows -1 && j_ >= 0 && j_ < cols - 1;
  }

  double get_value(const cv::Mat & mat) const
  {
    return weight_tl_ * mat.at<float>(i_, j_) +
    weight_tr_ * mat.at<float>(i_, j_+1) +
    weight_bl_ * mat.at<float>(i_+1, j_) +
    weight_br_ * mat.at<float>(i_+1, j_+1);
  }

  bool on_image() const
  {
    return on_image_;
  }

  uint i() const { return i_; }
  uint j() const { return j_; }
};

void OccupancyGrid::computeDistanceMap()
{
  const int delta = 0;
  const int ddepth = -1;

  cv::distanceTransform(sensor_map_bw_, dist_, CV_DIST_L2, CV_DIST_MASK_PRECISE); //
  cv::Sobel(dist_, grad_x_, ddepth, 1, 0, 3, 1.0 / 8, delta, cv::BORDER_DEFAULT);
  cv::Sobel(dist_, grad_y_, ddepth, 0, 1, 3, 1.0 / 8, delta, cv::BORDER_DEFAULT);

  cv::distanceTransform(sensor_map_bw_inv_, dist_inv_, CV_DIST_L2, CV_DIST_MASK_PRECISE); //CV_DIST_L2
  cv::Sobel(dist_inv_, grad_x_inv_, ddepth, 1, 0, 3, 1.0 / 8, delta, cv::BORDER_DEFAULT);
  cv::Sobel(dist_inv_, grad_y_inv_, ddepth, 0, 1, 3, 1.0 / 8, delta, cv::BORDER_DEFAULT);

  if(1)
  {
    //      std::cout << dist_.type() << " val:" << dist_inv_.at<float>(150,147) << std::endl;
    //      std::cout << dist_.type() << " val:" << dist_inv_.at<float>(149,147) << std::endl;
    //      std::cout << grad_x_.at<float>(500,500) << std::endl;
    //      auto dx = dist_inv_.at<float>(150,147) - dist_inv_.at<float>(149,147);
    //      std::cout << "dx:" << dx << std::endl;
    //      std::cout << grad_x_.type() << " grad_x:" << " " << grad_x_inv_.at<float>(150,147) << std::endl;
    //      std::cout << grad_y_.type() << " grad_y:" << " " << grad_y_inv_.at<float>(149,147) << std::endl;

    //      show_img("Normal", sensor_map_bw_);
    //      show_img("Inverted", sensor_map_bw_inv_);
    //        show_img("Distance Transform Image", dist_);
    //        show_img("Gradient x Image", grad_x_);
    //        show_img("Gradient y Image", grad_y_);

    //      show_img("Distance Transform Image Inv", dist_inv_);
    //      show_img("Gradient x Image Inv", grad_x_inv_);
    //      show_img("Gradient y Image Inv", grad_y_inv_);

    //      cv::waitKey(1);
  }
}

void OccupancyGrid::setMapFromFile(const std::string & filename)
{
  cv::Mat sensor_map = cv::imread(filename.c_str());
  cv::flip(sensor_map, sensor_map, 0);
  cv::cvtColor(sensor_map, sensor_map_bw_, cv::COLOR_BGR2GRAY);

  cv::threshold(sensor_map_bw_, sensor_map_bw_,     127, 255, cv::THRESH_BINARY);
  cv::threshold(sensor_map_bw_, sensor_map_bw_inv_, 127, 255, cv::THRESH_BINARY_INV);

  computeDistanceMap();
}

void OccupancyGrid::setMap(const cv::Mat & sensor_map, const arr & map_center_position)
{
  map_center_position_ = map_center_position;

  cv::threshold(sensor_map, sensor_map_bw_,     127, 255, cv::THRESH_BINARY);
  cv::threshold(sensor_map_bw_, sensor_map_bw_inv_, 127, 255, cv::THRESH_BINARY_INV);

  computeDistanceMap();
}

void OccupancyGrid::phi(arr& y, arr& J, const rai::KinematicWorld& G)
{
  rai::Frame *object = G.getFrameByName(object_.c_str());

  arr pos, Jpos;
  G.kinematicsPos(pos, Jpos, object);

  y.resize(1);

  auto dist_info = get_distance_info(pos);

  y(0) = safety_distance_ - dist_info[0];

  if(&J)
  {
    J = zeros(dim_, Jpos.d1);

    J(0, 0) = -dist_info[1] * Jpos(0, 0);
    J(0, 1) = -dist_info[2] * Jpos(1, 1); //sin(theta) * Jpos(1, 1);
  }
}

std::vector<double> OccupancyGrid::get_distance_info(const arr & global_pos) const
{
  const auto x = global_pos(0) - map_center_position_(0);
  const auto y = global_pos(1) - map_center_position_(1);

  SubPixAccessor pix(dist_.rows, dist_.cols, cell_size_, x, y);

  double dist = 0;
  double gx = 0;
  double gy = 0;

  if( pix.on_image() )
  {
    auto d = pix.get_value(dist_);

    if(d>0)
    {
      auto grad_x = pix.get_value(grad_x_);
      auto grad_y = pix.get_value(grad_y_);

      dist = cell_size_ * (d);
      gx = (grad_x);
      gy = (grad_y);
    }
    else
    {
      auto d_inv = pix.get_value(dist_inv_);
      auto grad_x_inv = pix.get_value(grad_x_inv_);
      auto grad_y_inv = pix.get_value(grad_y_inv_);

      dist = cell_size_ * (- d_inv);
      gx = (- grad_x_inv);
      gy = (- grad_y_inv);
    }
  }
  else // not fully exact here, just tries to go back on closest point on map
  {
    double dx = 0;
    double dy = 0;
    if(pix.i() < 0)
    {
      dy = cell_size_ * pix.i();
      //gy = 1;
    }
    else if( pix.i() > dist_.rows )
    {
      dy = -cell_size_ * (pix.i() - dist_.rows);
      gy = -1;
    }

    if(pix.j() < 0)
    {
      dx = cell_size_ * pix.j();
      gx = 1;
    }
    else if( pix.j() > dist_.cols )
    {
      dx = -cell_size_ * (pix.j() - dist_.cols);
      gx = -1;
    }

    auto theta = std::atan2(-dy, -dx);
    dist = -sqrt(dx*dx+dy*dy);
    gx *= cos(theta);
    gy *= sin(theta);
  }

  return {dist, gx, gy};
}

void OccupancyGrid::convert(double x, double y, uint & i, uint & j, float & d_i, float & d_j) const
{
  float float_i = 0.5 * dist_.rows + y / cell_size_;
  float float_j = 0.5 * dist_.cols + x / cell_size_;

  i = uint(float_i);
  j = uint(float_j);

  d_i = float_i - i;
  d_j = float_j - j;
}

double OccupancyGrid::get_value_at(const cv::Mat & mat, uint i, uint j, float d_i, float d_j) const
{
  float weight_tl = (1.0 - d_j) * (1.0 - d_i);
  float weight_tr = (d_j)       * (1.0 - d_i);
  float weight_bl = (1.0 - d_j) * (d_i);
  float weight_br = (d_j)       * (d_i);

  return weight_tl * mat.at<float>(i, j) +
      weight_tr * mat.at<float>(i, j+1) +
      weight_bl * mat.at<float>(i+1, j) +
      weight_br * mat.at<float>(i+1, j+1);
}
