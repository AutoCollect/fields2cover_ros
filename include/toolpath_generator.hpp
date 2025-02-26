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
   * @param op_width Offset (step) distance used for generating the toolpath.
   * @param smooth_boundary Whether to smooth the outer boundary.
   */
  ToolpathGenerator(int smooth_number, double op_width, bool smooth_boundary = true);

  /// Set operation width
  void setOperationWidth(const double &op_width);

  /// Set the outer contour (must be a closed polyline).
  void setContour(const ToolPolyline &contour);

  /// Set the contour offset
  void setContourOffset(const double &contour_offset);

  /// Set the polygon name.
  void setPolygonName(const std::string &name);

  /// Set the max offsets threshold
  void setMaxOffsets(const int &max_offsets);

  /// Set the spiral entry point.
  void setSpiralEntryPoint(const ToolPoint &spiral_entry_point);

  /// Set contour resample step.
  void setContourResampleStep(const double &resample_step);

  /// Set bool flag for spiral contour clock-wise or anti-clockwise
  void setSpiralReversed(const bool &spiral_reversed);

  /**
   * @brief Resamples a polygon contour to achieve uniform spacing between points.
   *
   * This function generates a new polygon contour such that the distance between 
   * consecutive points does not exceed a specified maximum spacing (sampleDistance). 
   * If the gap between two original vertices is larger than sampleDistance, intermediate 
   * points are added using linear interpolation. If an interpolated point lies within a 
   * given tolerance of an original vertex, that vertex is used instead to preserve sharp features.
   *
   * @param contour The original polygon contour.
   * @param sampleDistance The desired maximum distance between consecutive points.
   * @param closeContour If true, the output polygon will be closed by connecting the last point back to the first.
   * @param tolerance Optional tolerance used to decide if an interpolated point is close enough to an original vertex 
   *                  (default: std::numeric_limits<double>::epsilon() * 100).
   * @return ToolpathGenerator::ToolPolyline The uniformly resampled polygon contour.
   */
  /// resample contour
  ToolPolyline resampleContour(
    const ToolPolyline &contour, 
    const double sampleDistance, 
    bool closeContour = false,
    double tolerance = std::numeric_limits<double>::epsilon() * 100) const;

  /// given an point, re-generate/re-arrange contour, so that:
  /// 1. the first point of contour is the closest point to given point
  /// 2. think about clockwise and anti-clockwise case
  ToolPolyline generateContour(const ToolPoint &point, const ToolPolyline &contour) const;

  ToolPolyline generateContour(const ToolPoint &point, const ToolPolyline &contour, 
                               const bool &is_reversed) const;

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

  ToolPolyline getEntrySpiral() const { return inward_spiral_path_; }

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

  // --------------------- Constant ---------------------

  const double scale_ = 1000.0;       // Path64 scale factor

  // --------------------- Spiral Path Main Input ---------------------
  
  ToolPolyline contour_;              // The outer contour.
  std::string poly_name_;             // Polygon name.

  // --------------------- Spiral Path Params ---------------------
  
  double contour_offset_;             // spiral contour offset relative to upath contour
  double op_width_;                   // operation width
  int max_offsets_;                   // max offsets number threshold.
  ToolPoint spiral_entry_point_;      // spiral entry point
  double contour_resample_step_;      // contour resample step
  bool spiral_reversed_;              // spiral contour clock-wise or anti-clockwise flag
  
  int smooth_number_;
  bool smooth_boundary_;

  // --------------------- Spiral Path Output ---------------------
  
  ToolPolyline inward_spiral_path_;   // final inward spiral toolpath.
  std::vector<ToolPolyline> offsets_; // Computed inward offset polylines.
  double entry_d_0_;                  // Current parameter along the polyline.

  // --------------------- ROS Publisher ---------------------

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

  /// Advances the parameter along the polyline by a distance equal to op_width.
  double advanceParameter(double d0, double op_width,
                          const ToolPolyline &points) const;

  /// Computes the next turning parameter along the polyline.
  double computeNextTurnParam(double d0, double op_width,
                              const ToolPolyline *points = nullptr) const;

  /// Blends the points in 'current' with corresponding points in 'target' (in place).
  void blendPoints(ToolPolyline &current, const ToolPolyline &target) const;

  /// Computes successive inward offsets using Clipper2.
  std::vector<ToolPolyline> computeFullOffsets(const double &op_width,
                                               const ToolPolyline &contour) const;

  /// Computes successive inward offsets using Clipper2.
  std::vector<ToolPolyline> computeOffsets(const double &op_width,
                                           const ToolPolyline &contour) const;

  /// Computes successive inward offsets using Clipper2.
  std::vector<ToolPolyline> computeOffsets(const double &op_width,
                                           const int &max_offsets, 
                                           const ToolPolyline &contour) const;
};

#endif // TOOLPATH_GENERATOR_HPP
