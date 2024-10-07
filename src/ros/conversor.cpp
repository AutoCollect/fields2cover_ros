//=============================================================================
//    Copyright (C) 2021-2022 Wageningen University - All Rights Reserved
//                     Author: Gonzalo Mier
//                        BSD-3 License
//=============================================================================

#include "ros/conversor.h"
#include <tf2/LinearMath/Quaternion.h>

namespace conversor
{
void ROS::to(const F2CPoint& _point, GeometryMsgs::Point32& _p32) {
  _p32.x = static_cast<float>(_point.getX());
  _p32.y = static_cast<float>(_point.getY());
  _p32.z = static_cast<float>(_point.getZ());
}

void ROS::to(const F2CPoint& _point, GeometryMsgs::Point& _p64) {
  _p64.x = _point.getX();
  _p64.y = _point.getY();
  _p64.z = _point.getZ();
}

void ROS::to(const F2CPoint& _point, const double& angle, GeometryMsgs::PoseStamped& _p64) {
  _p64.pose.position.x = _point.getX();
  _p64.pose.position.y = _point.getY();
  _p64.pose.position.z = _point.getZ();

  // Create a quaternion from yaw
  tf2::Quaternion quat;
  quat.setRPY(0, 0, (normalize(angle)));  // Roll = 0, Pitch = 0, Yaw = input yaw

  _p64.pose.orientation.x = quat.getX();
  _p64.pose.orientation.y = quat.getY();
  _p64.pose.orientation.z = quat.getZ();
  _p64.pose.orientation.w = quat.getW();
}

double ROS::normalize(const double& angle) {
  const double result = fmod(angle + M_PI, 2.0 * M_PI);
  if(result <= 0.0) return result + M_PI;
  return result - M_PI;
}

void ROS::to(const F2CCell& _poly,
    std::vector<GeometryMsgs::Polygon>& _ros_poly) {
  GeometryMsgs::Polygon ros_ring;
  to(_poly.getExteriorRing(), ros_ring);
  _ros_poly.push_back(ros_ring);
  const int n_in_rings = _poly.size() - 1;
  for (int i=0; i < n_in_rings; ++i) {
    ros_ring.points.clear();
    to(_poly.getInteriorRing(i), ros_ring);
    _ros_poly.push_back(ros_ring);
  }
}

void ROS::to(const F2CCells& _polys,
    std::vector<std::vector<GeometryMsgs::Polygon>>& _ros_polys) {
  for (auto&& p : _polys) {
    std::vector<GeometryMsgs::Polygon> poly;
    to(p, poly);
    _ros_polys.push_back(poly);
  }
}

void ROS::to(const F2CLineString& _line, NavMsgs::Path& _path) {
  GeometryMsgs::PoseStamped pose;
  for (auto&& p : _line) {
    to(p, pose.pose.position);
    _path.poses.push_back(pose);
  }
}

}  // namespace conversor

