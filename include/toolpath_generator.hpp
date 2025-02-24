#ifndef TOOLPATH_GENERATOR_HPP
#define TOOLPATH_GENERATOR_HPP

/**
 * @file toolpath_generator.hpp
 * @brief ToolpathGenerator class for generating toolpaths based on an Archimedean spiral logic.
 *
 * This file defines the ToolpathGenerator class that supports smoothing of the contour,
 * computing inward offsets (using Clipper2), and generating toolpaths via three different methods.
 * It integrates with ROS for visualization and uses the Clipper2 library for geometric operations.
 *
 * Usage: Include this header to generate and smooth toolpaths based on a provided contour.
 */

#include <vector>
#include <string>
#include <optional>

#include <ros/ros.h>

// Include Clipper2 (ensure Clipper2 is installed and its include path is set correctly)
// Do not bring Clipper2Lib's names into the global namespace.
#include <clipper2/clipper.h>

/**
 * @brief ToolpathGenerator generates toolpaths based on an Archimedean spiral logic.
 *
 * This class supports smoothing of the contour, computing inward offsets (using Clipper2),
 * and generating toolpaths via three different methods.
 */
class ToolpathGenerator {
 public:
  // Nested types to avoid conflicts with other libraries.
  struct ToolPoint {
    double x;
    double y;
  };

  using ToolPolyline = std::vector<ToolPoint>;

  /**
   * @brief Construct a new ToolpathGenerator object.
   *
   * @param smooth_number Number of smoothing iterations to apply.
   * @param toolpath_size Offset (step) distance used for generating the toolpath.
   * @param smooth_boundary Whether to smooth the outer boundary.
   */
  ToolpathGenerator(int smooth_number, double toolpath_size, bool smooth_boundary = true);

  /// Set operation width
  void setToolpathSize(const double &toolpath_size);

  /// Set the outer contour (must be a closed polyline).
  void setContour(const ToolPolyline &contour);

  /// Set the polygon name.
  void setPolygonName(const std::string &name);

  /// Generate the toolpath using the Archimedean spiral method.
  void archimedeanSpiral();

  /// Alternative spiral generation using intersection “tricks.”
  void archimedeanSpiralTrick();

  /// Generate a smooth toolpath by blending segments between offsets.
  void archimedeanSpiralSmooth();

  /// Directly smooth the polygon (outer boundary only; holes not implemented).
  void directlyPolygonSmoothing();

  /// Reorient the polygon in an optimal direction.
  void optimalDirection();

  /// Directly smooth a contour by averaging each triplet of consecutive points.
  void directlyContourSmoothing(ToolPolyline &contour);

  /// Smooth a contour by subdividing and averaging adjacent points.
  void contourSmoothing(ToolPolyline &contour);

  /// Smooth the entire polygon (outer contour) and reorient it.
  void polygonSmoothing();

  /// Print the generated toolpath coordinates.
  void printPath() const;

  void plotPath() const;

  ToolPolyline getEntrySpiral() const { return entry_spiral_; }

  std::vector<ToolPolyline> getOffsets() const { return offsets_; }

  /// Helper function to publish a marker for an offset polygon.
  void publishOffsetMarkers() const;

  void deleteMarkers() const;

  Clipper2Lib::Path64 convertPathDtoPath64(const Clipper2Lib::PathD &pathD,
                                            const double &scale) const {
    Clipper2Lib::Path64 path64;
    path64.reserve(pathD.size());
    for (const auto &pt : pathD) {
      Clipper2Lib::Point64 pt64;
      // Multiply by scale and round to the nearest integer.
      pt64.x = static_cast<int64_t>(std::round(pt.x * scale));
      pt64.y = static_cast<int64_t>(std::round(pt.y * scale));
      path64.push_back(pt64);
    }
    return path64;
  }

  Clipper2Lib::PathD convertPath64toPathD(const Clipper2Lib::Path64 &path64,
                                          const double &scale) const {
    Clipper2Lib::PathD pathD;
    pathD.reserve(path64.size());
    for (const auto &pt : path64) {
      // Divide by the scale factor to convert to double precision units.
      Clipper2Lib::PointD ptD;
      ptD.x = static_cast<double>(pt.x) / scale;
      ptD.y = static_cast<double>(pt.y) / scale;
      pathD.push_back(ptD);
    }
    return pathD;
  }

 private:
  const double scale_ = 1000.0;  // Path64 scale factor

  ToolPolyline entry_spiral_;         // final toolpath.
  double entry_d_0_;                  // Current parameter along the polyline.
  std::vector<ToolPolyline> offsets_; // Computed inward offset polylines.
  std::string poly_name_;             // Polygon name.

  int smooth_number_;
  double toolpath_size_;
  bool smooth_boundary_;
  ToolPolyline contour_;              // The outer contour.

  ros::NodeHandle nh_;                // ROS node handle.
  mutable ros::Publisher offset_pub_; // ROS publisher for offset polygons visualization.
  mutable ros::Publisher path_pub_;   // ROS publisher for path visualization.

  // --------------------- Helper Functions ---------------------

  /// Computes the total Euclidean length of a polyline.
  double computeTotalLength(const ToolPolyline &points) const;

  /// Returns a point on a closed polyline corresponding to the fractional parameter offset_val.
  ToolPoint getPointFromOffset(double offset_val, const ToolPolyline &points) const;

  /// Finds the parameter (in [0,1]) along the polyline that is closest to a given point.
  double findNearestParam(const ToolPoint &point, const ToolPolyline &points) const;

  /// Selects a sub-polyline of 'points' between parameters d0 and d1.
  ToolPolyline selectSubpolyline(const ToolPolyline &points, double d0,
                                 double d1 = 1.0) const;

  /// Computes the intersection point between two line segments; returns std::nullopt if none.
  std::optional<ToolPoint> lineSegmentIntersection(const ToolPoint &l0, const ToolPoint &l1,
                                                     const ToolPoint &p, const ToolPoint &q) const;

  /// Computes the parameter (in [0,1]) for the first intersection between a polyline and an infinite line.
  double intersectParamWithLine(const ToolPolyline &points,
                                const std::pair<ToolPoint, ToolPoint> &line) const;

  /// Computes a local tangent line at the point on the polyline nearest to 'point'.
  std::pair<ToolPoint, ToolPoint> getRelatedLine(const ToolPoint &point,
                                                 const ToolPolyline &points) const;

  /// Advances the parameter along the polyline by a distance equal to toolpath_size.
  double advanceParameter(double d0, double toolpath_size,
                          const ToolPolyline &points) const;

  /// Computes the next turning parameter along the polyline.
  double computeNextTurnParam(double d0, double toolpath_size,
                              const ToolPolyline *points = nullptr) const;

  /// Blends the points in 'current' with corresponding points in 'target' (in place).
  void blendPoints(ToolPolyline &current, const ToolPolyline &target) const;

  /// Computes successive inward offsets using Clipper2.
  std::vector<ToolPolyline> computeOffsets(double toolpath_size,
                                           const ToolPolyline &contour) const;
};

#endif // TOOLPATH_GENERATOR_HPP
