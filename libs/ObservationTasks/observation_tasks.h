#pragma once

#include <Kin/feature.h>

// reach a given head pose
struct HeadPoseMap:Feature{

  virtual void phi(arr& y, arr& J, const rai::KinematicWorld& G );

  virtual uint dim_phi(const rai::KinematicWorld& G)
  {
    return dim_;
  }

  virtual rai::String shortTag(const rai::KinematicWorld& G)
  {
    return rai::String("HeadPoseMap");
  }

  static arr buildTarget( rai::Vector const& position, double yaw_deg );

private:
  static const uint dim_ = 7;
};

// to reach the visibility of a given object position
// it requires the pivotPoint, but the pivot point could be deduced by geometric reasoning in the constructor
struct HeadGetSight:Feature{

  HeadGetSight( const arr& objectPosition, const arr& pivotPoint );

  virtual void phi(arr& y, arr& J, const rai::KinematicWorld& G );

  virtual uint dim_phi(const rai::KinematicWorld& G)
  {
    return dim_;
  }

  virtual rai::String shortTag(const rai::KinematicWorld& G)
  {
    return rai::String("HeadGetSight");
  }

private:
  // parameters
  static const uint dim_ = 6;
  const arr objectPosition_;
  const arr pivotPoint_;
  const rai::Vector headViewingDirection_;  // is Vec to be compatible with the interface of G.kinematicsVec

  // state
  arr w1_;
  bool moveAroundPivotDefined_;
};

// same as HeadGetSight but using quaternions for the head orientation
struct HeadGetSightQuat:Feature{

  HeadGetSightQuat( const arr& objectPosition, const arr& pivotPoint );

  virtual void phi(arr& y, arr& J, const rai::KinematicWorld& G );

  virtual uint dim_phi(const rai::KinematicWorld& G)
  {
    return dim_;
  }

  virtual rai::String shortTag(const rai::KinematicWorld& G)
  {
    return rai::String("HeadGetSightQuat");
  }

private:
  // parameters
  static const uint dim_ = 8;
  const arr objectPosition_;
  const arr pivotPoint_;
  const rai::Vector headViewingDirection_;  // is Vec to be compatible with the interface of G.kinematicsVec

  // state
  arr w1_;
  arr targetQuat_;
  bool moveAroundPivotDefined_;
};

// Active get sight, we assume that one of the object of the scene is attached to one hand of the robot
// the task will optimize the hand position and the head position
struct ActiveGetSight:Feature{

  ActiveGetSight( rai::String const& sensorName,
                  rai::String const& containerName,
                  arr const& pivotPoint,
                  arr const& aimingDir, // sensor
                  double preferedDistance = 0.8 );

  virtual void phi( arr& y, arr& J, rai::KinematicWorld const& G );

  virtual uint dim_phi( rai::KinematicWorld const& G )
  {
    return dim_;
  }

  virtual rai::String shortTag( rai::KinematicWorld const& G )
  {
    return rai::String( "ActiveGetSight" );
  }

  // parameters
  static const uint dim_ = 7;
  const rai::String headName_;
  const rai::String containerName_;
  const rai::Vector aimingDir_;     // in sensor's shape frame
  const rai::Vector pivotPoint_;    // in container's frame
  double preferedDistance_;
};

