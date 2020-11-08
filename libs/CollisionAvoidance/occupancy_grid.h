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

#pragma once

#include <algorithm>
#include <math_utility.h>

#include <opencv2/core/core.hpp>

#include <Kin/feature.h>
#include <Kin/taskMaps.h>

struct CircularCage:Feature{

  CircularCage( const std::string & object, const arr & center, double radius, double safety_distance = 0.5 )
    : object_( object )
    , center_( center )
    , max_radius_( radius )
    , safety_distance_( safety_distance )
  {

  }

  virtual rai::String shortTag(const rai::KinematicWorld& G)
  {
    return rai::String("CircularCage");
  }

  virtual void phi(arr& y, arr& J, const rai::KinematicWorld& G) override
  {
    rai::Frame *object = G.getFrameByName(object_.c_str());

    arr pos, Jpos;
    G.kinematicsPos(pos, Jpos, object);

    y.resize(1);//zeros(dim_phi(Gs, t));

    //y(0) = pos(0) - 1;
    const auto radius = sqrt(pow(pos(0)-center_(0), 2.0) + pow(pos(1)-center_(1), 2.0));
    y(0) = safety_distance_ + radius - max_radius_;

    //std::cout << "y(0)" << y(0) << std::endl;

    if(&J)
    {
      J = zeros(dim_, Jpos.d1);
      auto theta = std::atan2(pos(1)-center_(1), pos(0)-center_(0));

      J(0, 0) = cos(theta) * Jpos(0, 0);
      J(0, 1) = sin(theta) * Jpos(1, 1);
    }
  }

  virtual uint dim_phi(const rai::KinematicWorld& K) override
  {
    return dim_;
  }

private:
  static const uint dim_ = 1;
  std::string object_;
  double max_radius_;
  arr center_;
  double safety_distance_;
};

struct OccupancyGrid:Feature{

  OccupancyGrid( const std::string & object, double safety_distance = 0.5 )
    : object_( object )
    , cell_size_(0.025)
    , safety_distance_(safety_distance)
    , map_center_position_(zeros(2))
  {

  }

  double cell_size() const { return cell_size_; }

  void computeDistanceMap();

  void setMapFromFile(const std::string & filename);

  void setMap(const cv::Mat & sensor_map, const arr & map_center_position);

  virtual rai::String shortTag(const rai::KinematicWorld& G)
  {
    return rai::String("OccupancyGrid");
  }

  virtual void phi(arr& y, arr& J, const rai::KinematicWorld& G) override;

  virtual uint dim_phi(const rai::KinematicWorld& K) override
  {
    return dim_;
  }

  std::vector<double> get_distance_info(const arr & global_pos) const;

private:

  void convert(double x, double y, uint & i, uint & j, float & d_i, float & d_j) const;

  double get_value_at(const cv::Mat & mat, uint i, uint j, float d_i, float d_j) const;

private:
  static const uint dim_ = 1;
  const std::string object_;
  const double cell_size_;
  const double safety_distance_;
  arr map_center_position_;
  cv::Mat sensor_map_bw_, sensor_map_bw_inv_, dist_, dist_inv_, grad_x_, grad_y_, grad_x_inv_, grad_y_inv_;
};

