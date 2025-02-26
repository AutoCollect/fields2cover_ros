#include <cmath>
#include <iostream>
#include <stdexcept>
#include <limits>

#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>

#include "toolpath_generator.hpp"

namespace {

// Helper function to compute Euclidean distance between two ToolPoints.
double distance(const ToolpathGenerator::ToolPoint &a,
                const ToolpathGenerator::ToolPoint &b) {
  return std::hypot(b.x - a.x, b.y - a.y);
}

}  // namespace

// --------------------- Constructor & Setters ---------------------
ToolpathGenerator::ToolpathGenerator(int smooth_number, double op_width,
                                     bool smooth_boundary)
    : smooth_number_(smooth_number),
      op_width_(op_width),
      smooth_boundary_(smooth_boundary),
      entry_d_0_(0.0),
      max_offsets_(1),
      spiral_reversed_(false),
      poly_name_("Toolpath") {
  offset_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/offset_polygon", 10, true);
  path_pub_   = nh_.advertise<visualization_msgs::MarkerArray>("/spiral_path",    10, true);
}

void ToolpathGenerator::setOperationWidth(const double &op_width) {
  op_width_ = op_width;
}

void ToolpathGenerator::setContour(const ToolPolyline &contour) {
  contour_ = contour;
  if (contour_.empty()) {
    throw std::runtime_error("Contour is empty.");
  }
  // Ensure the contour is closed.
  if (distance(contour_.front(), contour_.back()) > 1e-6) {
    contour_.push_back(contour_.front());
  }
}

void ToolpathGenerator::setPolygonName(const std::string &name) {
  poly_name_ = name;
}

void ToolpathGenerator::setMaxOffsets(const int &max_offsets) {
  max_offsets_ = max_offsets;
}

void ToolpathGenerator::setSpiralReversed(const bool &spiral_reversed) {
  spiral_reversed_ = spiral_reversed;
}

// --------------------- Helper Functions ---------------------
ToolpathGenerator::ToolPolyline ToolpathGenerator::generateContour(
    const ToolPoint &point, 
    const ToolPolyline &contour) {

  // ROS_ERROR("spiral_reversed = %d", int(spiral_reversed_));
  return generateContour(point, contour, spiral_reversed_);
}


ToolpathGenerator::ToolPolyline ToolpathGenerator::generateContour(
    const ToolPoint    &point, 
    const ToolPolyline &contour, const bool &is_reversed) {

  // Check empty
  if (contour.empty()) return {};

  // Work on a local copy so that the input contour remains unchanged.
  ToolPolyline poly = contour;

  // Compute squared Euclidean distance to avoid costly sqrt.
  auto sqDist = [&point](const ToolPoint &p) -> double {
    double dx = p.x - point.x;
    double dy = p.y - point.y;
    return dx * dx + dy * dy;
  };
    
  // Find the point in the contour closest to the given point.
  auto closestIt = std::min_element(poly.begin(), poly.end(),
    [&](const ToolPoint &a, const ToolPoint &b) {
      return sqDist(a) < sqDist(b);
  });
    
  // Rotate the polygon so that the closest point becomes the first element.
  std::rotate(poly.begin(), closestIt, poly.end());

  // If is_reversed is true, reverse the order (except the starting point)
  if (is_reversed) {
    std::reverse(poly.begin() + 1, poly.end());
  }

  ROS_ERROR("poly front: %f, %f", poly.front().x, poly.front().y);
  return poly;
}

double ToolpathGenerator::computeTotalLength(
    const ToolPolyline &points) const {
  double total = 0.0;
  for (size_t i = 0; i < points.size() - 1; ++i) {
    total += distance(points[i], points[i + 1]);
  }
  return total;
}

ToolpathGenerator::ToolPoint ToolpathGenerator::getPointFromOffset(
    double offset_val, const ToolPolyline &points) const {
  // Ensure the polyline is closed.
  ToolPolyline pts = points;
  if (distance(pts.front(), pts.back()) > 1e-6) {
    pts.push_back(pts.front());
  }

  double total_length = computeTotalLength(pts);
  double target_length = offset_val * total_length;
  double cum_length = 0.0;
  for (size_t i = 0; i < pts.size() - 1; ++i) {
    double seg_length = distance(pts[i], pts[i + 1]);
    if (cum_length + seg_length >= target_length) {
      double t = (target_length - cum_length) / seg_length;
      ToolPoint p;
      p.x = pts[i].x + t * (pts[i + 1].x - pts[i].x);
      p.y = pts[i].y + t * (pts[i + 1].y - pts[i].y);
      return p;
    }
    cum_length += seg_length;
  }
  return pts.back();
}

double ToolpathGenerator::findNearestParam(const ToolPoint &point,
                                             const ToolPolyline &points) const {
  if (points.size() < 2)
    return 0.0;
  double total_length = computeTotalLength(points);
  double best_param = 0.0;
  double best_dist = std::numeric_limits<double>::max();
  double acc_length = 0.0;
  for (size_t i = 0; i < points.size() - 1; ++i) {
    ToolPoint p = points[i];
    ToolPoint q = points[i + 1];
    double seg_length = distance(p, q);
    if (seg_length < 1e-6)
      continue;
    double t = ((point.x - p.x) * (q.x - p.x) + (point.y - p.y) * (q.y - p.y)) /
               (seg_length * seg_length);
    if (t < 0.0)
      t = 0.0;
    if (t > 1.0)
      t = 1.0;
    ToolPoint proj;
    proj.x = p.x + t * (q.x - p.x);
    proj.y = p.y + t * (q.y - p.y);
    double d = distance(point, proj);
    if (d < best_dist) {
      best_dist = d;
      best_param = (acc_length + t * seg_length) / total_length;
    }
    acc_length += seg_length;
  }
  return best_param;
}

ToolpathGenerator::ToolPolyline ToolpathGenerator::selectSubpolyline(
    const ToolPolyline &points, double d0, double d1) const {
  const double tol = 1e-6;
  ToolPolyline result;

  // If d0 and d1 are almost equal, return the single point.
  if (std::fabs(d0 - d1) < 1e-7) {
    result.push_back(getPointFromOffset(d0, points));
    return result;
  }

  // Start with the point at d0.
  result.push_back(getPointFromOffset(d0, points));

  // Compute total length and the cumulative normalized parameter vector.
  double total_length = computeTotalLength(points);
  size_t n = points.size();
  std::vector<double> vec_ds;
  vec_ds.push_back(0.0);
  double length = 0.0;
  for (size_t i = 0; i < n; ++i) {
    const ToolPoint &p = points[i];
    const ToolPoint &q = points[(i + 1) % n];
    length += distance(p, q);
    vec_ds.push_back(length / total_length);
  }

  if (d0 > d1) {
    // Backward traversal: loop from the end of vec_ds.
    for (int i = static_cast<int>(vec_ds.size()) - 1; i >= 0; --i) {
      if (vec_ds[i] < d0 && vec_ds[i] > d1) {
        ToolPoint v = getPointFromOffset(vec_ds[i], points);
        // Add the point if it isn't a duplicate of the last one.
        if (std::fabs(v.x - result.back().x) >= tol ||
            std::fabs(v.y - result.back().y) >= tol) {
          result.push_back(v);
        }
      }
      if (vec_ds[i] < d1)
        break;
    }
  } else {
    // Forward traversal: first add points with parameters less than d0.
    for (int i = static_cast<int>(vec_ds.size()) - 1; i >= 1; --i) {
      if (vec_ds[i] < d0) {
        ToolPoint v = getPointFromOffset(vec_ds[i], points);
        if (std::fabs(v.x - result.back().x) >= tol ||
            std::fabs(v.y - result.back().y) >= tol) {
          result.push_back(v);
        }
      }
    }
    // Then add points with parameters greater than d1.
    for (int i = static_cast<int>(vec_ds.size()) - 1; i >= 1; --i) {
      if (vec_ds[i] > d1) {
        ToolPoint v = getPointFromOffset(vec_ds[i], points);
        if (std::fabs(v.x - result.back().x) >= tol ||
            std::fabs(v.y - result.back().y) >= tol) {
          result.push_back(v);
        }
      }
    }
  }

  // Ensure the point at d1 is included.
  ToolPoint v_final = getPointFromOffset(d1, points);
  if (std::fabs(v_final.x - result.back().x) >= tol ||
      std::fabs(v_final.y - result.back().y) >= tol) {
    result.push_back(v_final);
  }

  // Remove duplicate first point if the first two are nearly identical.
  if (result.size() > 1 &&
      std::fabs(result[1].x - result[0].x) < tol &&
      std::fabs(result[1].y - result[0].y) < tol) {
    result.erase(result.begin());
  }

  return result;
}

std::optional<ToolpathGenerator::ToolPoint>
ToolpathGenerator::lineSegmentIntersection(const ToolPoint &l0, const ToolPoint &l1,
                                             const ToolPoint &p, const ToolPoint &q) const {
  double r_x = l1.x - l0.x, r_y = l1.y - l0.y;
  double s_x = q.x - p.x, s_y = q.y - p.y;
  double rxs = r_x * s_y - r_y * s_x;
  if (std::fabs(rxs) < 1e-9)
    return std::nullopt;  // Segments are parallel or collinear.
  double t = ((p.x - l0.x) * s_y - (p.y - l0.y) * s_x) / rxs;
  double u = ((p.x - l0.x) * r_y - (p.y - l0.y) * r_x) / rxs;
  if (u >= 0 && u <= 1)
    return ToolPoint{l0.x + t * r_x, l0.y + t * r_y};
  return std::nullopt;
}

double ToolpathGenerator::intersectParamWithLine(
    const ToolPolyline &points,
    const std::pair<ToolPoint, ToolPoint> &line) const {
  const ToolPoint &l0 = line.first;
  const ToolPoint &l1 = line.second;
  for (size_t i = 0; i < points.size() - 1; ++i) {
    auto inter = lineSegmentIntersection(l0, l1, points[i], points[i + 1]);
    if (inter.has_value())
      return findNearestParam(inter.value(), points);
  }
  return -1.0;
}

std::pair<ToolpathGenerator::ToolPoint, ToolpathGenerator::ToolPoint>
ToolpathGenerator::getRelatedLine(const ToolPoint &point,
                                  const ToolPolyline &points) const {
  double par = findNearestParam(point, points);
  ToolPoint nearest = getPointFromOffset(par, points);
  const double eps = 1e-4;
  double par1 = (par - eps < 0.0) ? 0.0 : par - eps;
  double par2 = (par + eps > 1.0) ? 1.0 : par + eps;
  ToolPoint p1 = getPointFromOffset(par1, points);
  ToolPoint p2 = getPointFromOffset(par2, points);
  double tx = p2.x - p1.x, ty = p2.y - p1.y;
  double norm = std::sqrt(tx * tx + ty * ty);
  if (norm > 1e-9) {
    tx /= norm;
    ty /= norm;
  } else {
    tx = 1.0;
    ty = 0.0;
  }
  return {nearest, {nearest.x + tx, nearest.y + ty}};
}

double ToolpathGenerator::advanceParameter(double d0, double op_width,
                                             const ToolPolyline &points) const {
  double total_length = computeTotalLength(points);
  if (total_length < 1e-9)
    return d0;
  double increment = op_width / total_length;
  return std::min(d0 + increment, 1.0);
}

double ToolpathGenerator::computeNextTurnParam(double d0, double op_width,
                                                 const ToolPolyline *points) const {
  if (points == nullptr)
    return std::min(d0 + op_width * 0.01, 1.0);
  return advanceParameter(d0, op_width, *points);
}

void ToolpathGenerator::blendPoints(ToolPolyline &current,
                                      const ToolPolyline &target) const {
  size_t n = std::min(current.size(), target.size());
  for (size_t i = 0; i < n; ++i) {
    double t = (n > 1) ? static_cast<double>(i) / (n - 1) : 0.0;
    current[i].x = current[i].x + t * (target[i].x - current[i].x);
    current[i].y = current[i].y + t * (target[i].y - current[i].y);
  }
}

void ToolpathGenerator::plotPath() const {
  // Wait until there is at least one subscriber.
  ros::Rate rate(10);
  while (path_pub_.getNumSubscribers() == 0 && ros::ok()) {
    rate.sleep();
  }

  visualization_msgs::MarkerArray marker_array;

  // --- Create a marker for the contour_ (cyan) ---
  visualization_msgs::Marker contour_marker;
  contour_marker.header.frame_id = "map";  // Change to your fixed frame if needed.
  contour_marker.header.stamp = ros::Time::now();
  contour_marker.ns = "contour";
  contour_marker.id = 0;
  contour_marker.type = visualization_msgs::Marker::LINE_STRIP;
  contour_marker.action = visualization_msgs::Marker::ADD;
  contour_marker.pose.orientation.w = 1.0;
  contour_marker.scale.x = 0.5;  // Line width
  // Set cyan color: (R=0, G=1, B=1, A=1)
  contour_marker.color.r = 0.0;
  contour_marker.color.g = 1.0;
  contour_marker.color.b = 1.0;
  contour_marker.color.a = 1.0;

  // Populate the marker with points from contour_
  for (const auto &pt : contour_) {
    geometry_msgs::Point p;
    p.x = pt.x;
    p.y = pt.y;
    p.z = 0.0;  // Assuming a 2D path in the XY-plane.
    contour_marker.points.push_back(p);
  }

  // --- Create a marker for the inward_spiral_path_ (green line strip) ---
  visualization_msgs::Marker spiral_marker;
  spiral_marker.header.frame_id = "map";
  spiral_marker.header.stamp = ros::Time::now();
  spiral_marker.ns = "entry_spiral";
  spiral_marker.id = 1;
  spiral_marker.type = visualization_msgs::Marker::LINE_STRIP;
  spiral_marker.action = visualization_msgs::Marker::ADD;
  spiral_marker.pose.orientation.w = 1.0;
  spiral_marker.scale.x = 0.5;  // Line width
  // Set green color: (R=0, G=1, B=0, A=1)
  spiral_marker.color.r = 0.0;
  spiral_marker.color.g = 1.0;
  spiral_marker.color.b = 1.0;
  spiral_marker.color.a = 1.0;

  // Populate the marker with points from inward_spiral_path_
  for (const auto &pt : inward_spiral_path_) {
    geometry_msgs::Point p;
    p.x = pt.x;
    p.y = pt.y;
    p.z = 0.0;
    spiral_marker.points.push_back(p);
  }

  // --- Create a marker for the first point of inward_spiral_path_ (blue big point) ---
  visualization_msgs::Marker first_point_marker;
  first_point_marker.header.frame_id = "map";
  first_point_marker.header.stamp = ros::Time::now();
  first_point_marker.ns = "inward_spiral_path_first_point";
  first_point_marker.id = 2;
  first_point_marker.type = visualization_msgs::Marker::SPHERE;
  first_point_marker.action = visualization_msgs::Marker::ADD;
  first_point_marker.pose.orientation.w = 1.0;
  // Set a larger scale for the sphere (big point)
  first_point_marker.scale.x = 3.0;
  first_point_marker.scale.y = 3.0;
  first_point_marker.scale.z = 3.0;
  // Set blue color: (R=0, G=0, B=1, A=1)
  first_point_marker.color.r = 0.0;
  first_point_marker.color.g = 0.0;
  first_point_marker.color.b = 1.0;
  first_point_marker.color.a = 1.0;
  if (!inward_spiral_path_.empty()) {
    first_point_marker.pose.position.x = inward_spiral_path_.front().x;
    first_point_marker.pose.position.y = inward_spiral_path_.front().y;
    first_point_marker.pose.position.z = 0.0;
  }

  // --- Create a marker for the last point of inward_spiral_path_ (red big point) ---
  visualization_msgs::Marker last_point_marker;
  last_point_marker.header.frame_id = "map";
  last_point_marker.header.stamp = ros::Time::now();
  last_point_marker.ns = "inward_spiral_path_last_point";
  last_point_marker.id = 3;
  // last_point_marker.type = visualization_msgs::Marker::SPHERE;
  last_point_marker.type = visualization_msgs::Marker::CUBE;
  last_point_marker.action = visualization_msgs::Marker::ADD;
  last_point_marker.pose.orientation.w = 1.0;
  // Set a larger scale for the sphere (big point)
  last_point_marker.scale.x = 3.0;
  last_point_marker.scale.y = 3.0;
  last_point_marker.scale.z = 3.0;
  // Set red color: (R=1, G=0, B=0, A=1)
  last_point_marker.color.r = 1.0;
  last_point_marker.color.g = 0.0;
  last_point_marker.color.b = 0.0;
  last_point_marker.color.a = 1.0;
  if (!inward_spiral_path_.empty()) {
    last_point_marker.pose.position.x = inward_spiral_path_.back().x;
    last_point_marker.pose.position.y = inward_spiral_path_.back().y;
    last_point_marker.pose.position.z = 0.0;
  }

  // Add all markers to the MarkerArray.
  marker_array.markers.push_back(contour_marker);
  marker_array.markers.push_back(spiral_marker);
  marker_array.markers.push_back(first_point_marker);
  marker_array.markers.push_back(last_point_marker);

  // Publish the marker array.
  path_pub_.publish(marker_array);
  ROS_INFO("Published updated path markers");
}

// Helper function to publish a marker for an offset polygon.
void ToolpathGenerator::publishOffsetMarkers() const {
  // Wait until there is at least one subscriber.
  ros::Rate rate(10);
  while (offset_pub_.getNumSubscribers() == 0 && ros::ok()) {
    rate.sleep();
  }

  visualization_msgs::MarkerArray marker_array;
  offset_pub_.publish(marker_array);

  for (size_t i = 0; i < offsets_.size(); ++i) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";  // Adjust the frame if needed.
    marker.header.stamp = ros::Time::now();
    marker.ns = "inward_offsets";
    marker.id = i;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.01;  // Line width

    // Vary the color slightly for each polygon (for clarity).
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    // Convert each polygon point back to meters and add to the marker.
    for (const auto &pt : offsets_[i]) {
      geometry_msgs::Point ros_pt;
      ros_pt.x = pt.x;
      ros_pt.y = pt.y;
      ros_pt.z = 0.0;
      marker.points.push_back(ros_pt);
    }
    // Optionally, close the polygon by adding the first point again.
    if (!marker.points.empty())
      marker.points.push_back(marker.points.front());

    marker_array.markers.push_back(marker);
  }

  // Publish all markers.
  offset_pub_.publish(marker_array);
  ROS_INFO("Published %lu inward offset polygons.", offsets_.size());
}

void ToolpathGenerator::deleteMarkers() const {
  visualization_msgs::MarkerArray delete_all;
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.action = 3;  // DELETEALL action
  delete_all.markers.push_back(marker);
  offset_pub_.publish(delete_all);
  path_pub_.publish(delete_all);
}

// --------------------- Modified computeOffsets ---------------------
std::vector<ToolpathGenerator::ToolPolyline>
ToolpathGenerator::computeFullOffsets(const double &op_width,
                                      const ToolPolyline &contour) const {
  // return results
  std::vector<ToolPolyline> offset_polygons;
  
  // first add contour into result
  offset_polygons.push_back(contour);

  // Convert contour to Clipper2 PathD.
  Clipper2Lib::PathD polygon;
  for (const auto &pt : contour) {
    polygon.push_back({pt.x, pt.y});
  }

  // Vector to store all the offset polygons (including the initial polygon).
  std::vector<Clipper2Lib::PathD> allPolygons;
  allPolygons.push_back(polygon);

  while (true) {
    // Use the last polygon generated.
    const Clipper2Lib::Path64 &currentPolygon =
        convertPathDtoPath64(allPolygons.back(), scale_);

    Clipper2Lib::ClipperOffset offsetter;
    offsetter.AddPath(currentPolygon, Clipper2Lib::JoinType::Round,
                      Clipper2Lib::EndType::Polygon);

    Clipper2Lib::Paths64 offsetPaths;
    offsetter.Execute(op_width, offsetPaths);

    // Stop if no further offset polygon is produced.
    if (offsetPaths.empty())
      break;

    // For efficiency, use the first resulting polygon.
    Clipper2Lib::Path64 offsetPolygon = offsetPaths[0];

    // Break if the offset polygon is too small or degenerate.
    if (offsetPolygon.empty())
      break;

    allPolygons.push_back(convertPath64toPathD(offsetPolygon, scale_));

    // Append the offset polygon.
    ToolPolyline offsetPolyline;
    for (const auto &p : allPolygons.back()) {
      offsetPolyline.push_back(ToolPoint{p.x, p.y});
    }

    // Ensure each offset polygon is closed.
    offsetPolyline.push_back(offsetPolyline.front());

    // Add into offset_polygons.
    offset_polygons.push_back(offsetPolyline);
  }

  return offset_polygons;
}

std::vector<ToolpathGenerator::ToolPolyline>
ToolpathGenerator::computeOffsets(const double &op_width,
                                  const ToolPolyline &contour) const {
  
  // ROS_ERROR("max offset: %d", max_offsets_);
  return computeOffsets(op_width, max_offsets_, contour);                                      
}

std::vector<ToolpathGenerator::ToolPolyline>
ToolpathGenerator::computeOffsets(const double &op_width,
                                  const int &max_offsets,
                                  const ToolPolyline &contour) const {
  // return results
  std::vector<ToolPolyline> offset_polygons;
  
  // first add contour into result
  offset_polygons.push_back(contour);

  // Check max offsets
  if (max_offsets <= 1)
    return offset_polygons;

  int offsets_counter = 1;

  // Convert contour to Clipper2 PathD.
  Clipper2Lib::PathD polygon;
  for (const auto &pt : contour) {
    polygon.push_back({pt.x, pt.y});
  }

  // Vector to store all the offset polygons (including the initial polygon).
  std::vector<Clipper2Lib::PathD> allPolygons;
  allPolygons.push_back(polygon);

  while (true) {
    // Use the last polygon generated.
    const Clipper2Lib::Path64 &currentPolygon =
        convertPathDtoPath64(allPolygons.back(), scale_);

    Clipper2Lib::ClipperOffset offsetter;
    offsetter.AddPath(currentPolygon, Clipper2Lib::JoinType::Round,
                      Clipper2Lib::EndType::Polygon);

    Clipper2Lib::Paths64 offsetPaths;
    offsetter.Execute(op_width, offsetPaths);

    // Stop if no further offset polygon is produced.
    if (offsetPaths.empty())
      break;

    // For efficiency, use the first resulting polygon.
    Clipper2Lib::Path64 offsetPolygon = offsetPaths[0];

    // Break if the offset polygon is too small or degenerate.
    if (offsetPolygon.empty())
      break;

    allPolygons.push_back(convertPath64toPathD(offsetPolygon, scale_));

    // Append the offset polygon.
    ToolPolyline offsetPolyline;
    for (const auto &p : allPolygons.back()) {
      offsetPolyline.push_back(ToolPoint{p.x, p.y});
    }

    // Ensure each offset polygon is closed.
    offsetPolyline.push_back(offsetPolyline.front());

    // Add into offset_polygons.
    offset_polygons.push_back(offsetPolyline);

    // Check max offsets
    offsets_counter ++;
    if (offsets_counter >= max_offsets)
      break;
  }

  return offset_polygons;
}

// --------------------- Public API Methods ---------------------
void ToolpathGenerator::archimedeanSpiral() {
  std::cout << "Starting archimedeanSpiral...\n";
  for (int i = 0; i < smooth_number_; ++i) {
    polygonSmoothing();
  }

  std::cout << "Outer contour vertices: " << contour_.size() << "\n";

  const double delta = op_width_ * scale_;
  offsets_ = computeOffsets(-delta, contour_);
  std::cout << "Number of computed offsets: " << offsets_.size() << "\n";

  inward_spiral_path_.clear();
  for (size_t i = 0; i < offsets_.size(); ++i) {
    const ToolPolyline &off = offsets_[i];
    double next_param = computeNextTurnParam(entry_d_0_, op_width_, &off);
    std::cout << "\n--- Processing offset " << i + 1 << " of "
              << offsets_.size() << " ---\n";
    std::cout << "  Computed turning parameter: " << next_param << "\n";
    ToolPolyline segment = selectSubpolyline(off, entry_d_0_, next_param);
    std::cout << "  Adding " << segment.size()
              << " points to toolpath segment\n";
    inward_spiral_path_.insert(inward_spiral_path_.end(), segment.begin(), segment.end());
    ToolPoint current_pt = getPointFromOffset(entry_d_0_, off);
    if (i != offsets_.size() - 1) {
      double new_d0 = findNearestParam(current_pt, off);
      std::cout << "  Updating entry parameter from " << entry_d_0_
                << " to " << new_d0 << "\n";
      entry_d_0_ = new_d0;
    }
  }

  if (!inward_spiral_path_.empty() &&
      std::fabs(inward_spiral_path_.back().x) < 1e-5 &&
      std::fabs(inward_spiral_path_.back().y) < 1e-5) {
    inward_spiral_path_.pop_back();
  }

  std::cout << "archimedeanSpiral completed.\n";
}

void ToolpathGenerator::archimedeanSpiralTrick() {
  std::cout << "Starting archimedeanSpiralTrick...\n";
  for (int i = 0; i < smooth_number_; ++i) {
    polygonSmoothing();
  }

  ToolPolyline contour1 = contour_;
  offsets_ = computeOffsets(op_width_, contour1);
  for (size_t i = 0; i < offsets_.size() - 1; ++i) {
    ToolPoint entry_p0 = getPointFromOffset(entry_d_0_, offsets_[i]);
    double entry_d1 = findNearestParam(entry_p0, offsets_[i + 1]);
    ToolPoint entry_p1 = getPointFromOffset(entry_d1, offsets_[i + 1]);
    ToolPolyline entry_half = selectSubpolyline(offsets_[i], entry_d_0_);
    auto related_line = getRelatedLine(entry_p1, offsets_[i + 1]);
    double entry_cutting_d = intersectParamWithLine(entry_half, related_line);
    double d0 = advanceParameter(entry_d_0_, op_width_, offsets_[i]);
    ToolPoint v = getPointFromOffset(d0, offsets_[i]);
    double new_d0 = findNearestParam(v, entry_half);
    if (entry_cutting_d > new_d0)
      entry_cutting_d = new_d0;
    ToolPolyline path_segment = selectSubpolyline(entry_half, 0.0, entry_cutting_d);
    inward_spiral_path_.insert(inward_spiral_path_.end(), path_segment.begin(),
                         path_segment.end());
    inward_spiral_path_.push_back(entry_p1);
    entry_d_0_ = entry_d1;
  }
  double final_d1 =
      computeNextTurnParam(entry_d_0_, op_width_, &offsets_.back());
  ToolPolyline final_segment = selectSubpolyline(offsets_.back(), entry_d_0_,
                                                  final_d1);
  inward_spiral_path_.insert(inward_spiral_path_.end(), final_segment.begin(),
                       final_segment.end());
  std::cout << "archimedeanSpiralTrick completed.\n";
}

void ToolpathGenerator::archimedeanSpiralSmooth() {
  std::cout << "Starting archimedeanSpiralSmooth...\n";
  for (int i = 0; i < smooth_number_; ++i) {
    polygonSmoothing();
  }

  ToolPolyline contour1 = contour_;
  offsets_ = computeOffsets(op_width_, contour1);
  for (size_t i = 0; i < offsets_.size() - 1; ++i) {
    ToolPolyline current_half = selectSubpolyline(offsets_[i], entry_d_0_);
    ToolPolyline next_half;
    for (const auto &pt : current_half) {
      double d = findNearestParam(pt, offsets_[i + 1]);
      next_half.push_back(getPointFromOffset(d, offsets_[i + 1]));
    }
    blendPoints(current_half, next_half);
    inward_spiral_path_.insert(inward_spiral_path_.end(), current_half.begin(),
                         current_half.end());
    ToolPoint entry_p0 = getPointFromOffset(entry_d_0_, offsets_[i]);
    entry_d_0_ = findNearestParam(entry_p0, offsets_[i + 1]);
  }
  std::cout << "archimedeanSpiralSmooth completed.\n";
}

void ToolpathGenerator::directlyPolygonSmoothing() {
  ToolPolyline outer = contour_;
  if (smooth_boundary_) {
    contourSmoothing(outer);
  }
  contour_ = outer;
}

void ToolpathGenerator::optimalDirection() {
  double sumx = 0.0, sumy = 0.0;
  for (const auto &pt : contour_) {
    sumx += pt.x;
    sumy += pt.y;
  }
  double cx = sumx / contour_.size();
  double cy = sumy / contour_.size();
  ToolPolyline new_coords;
  for (auto it = contour_.rbegin(); it != contour_.rend(); ++it) {
    double dx = it->x - cx, dy = it->y - cy;
    double angle = std::atan2(dy, dx) - M_PI / 2.0;
    double r = std::sqrt(dx * dx + dy * dy);
    new_coords.push_back({r * std::cos(angle), r * std::sin(angle)});
  }
  contour_ = new_coords;
}

void ToolpathGenerator::directlyContourSmoothing(ToolPolyline &contour) {
  size_t n = contour.size();
  if (n == 0)
    return;
  ToolPolyline smooth_contour;
  for (size_t i = 0; i < n; ++i) {
    ToolPoint p0 = contour[i % n];
    ToolPoint p1 = contour[(i + 1) % n];
    ToolPoint p2 = contour[(i + 2) % n];
    smooth_contour.push_back({(p0.x + p1.x + p2.x) / 3.0,
                               (p0.y + p1.y + p2.y) / 3.0});
  }
  contour = smooth_contour;
}

void ToolpathGenerator::contourSmoothing(ToolPolyline &contour) {
  ToolPolyline sub_contour;
  size_t n = contour.size();
  for (size_t i = 0; i < n; ++i) {
    ToolPoint p0 = contour[i];
    ToolPoint p1 = contour[(i + 1) % n];
    ToolPoint midpoint = {(p0.x + p1.x) / 2.0, (p0.y + p1.y) / 2.0};
    sub_contour.push_back(p0);
    sub_contour.push_back(midpoint);
  }
  for (size_t i = 1; i < sub_contour.size(); i += 2) {
    size_t prev = (i == 0 ? sub_contour.size() - 1 : i - 1);
    size_t next = (i + 1) % sub_contour.size();
    sub_contour[i].x = (sub_contour[prev].x + sub_contour[i].x + sub_contour[next].x) / 3.0;
    sub_contour[i].y = (sub_contour[prev].y + sub_contour[i].y + sub_contour[next].y) / 3.0;
  }
  contour = sub_contour;
}

void ToolpathGenerator::polygonSmoothing() {
  ToolPolyline outer = contour_;
  if (smooth_boundary_) {
    contourSmoothing(outer);
  }
  if (outer.front().x != outer.back().x || outer.front().y != outer.back().y) {
    outer.push_back(outer.front());
  }
  contour_ = outer;
}

void ToolpathGenerator::printPath() const {
  std::cout << "Toolpath Coordinates (" << poly_name_ << "):\n";
  for (const auto &pt : inward_spiral_path_) {
    std::cout << "(" << pt.x << ", " << pt.y << ")\n";
  }
}
