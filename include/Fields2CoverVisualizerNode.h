//=============================================================================
//    Copyright (C) 2021-2022 Wageningen University - All Rights Reserved
//                     Author: Gonzalo Mier
//                        BSD-3 License
//=============================================================================

#ifndef FIELDS2COVER_ROS_VISUALIZER_NODE_H_
#define FIELDS2COVER_ROS_VISUALIZER_NODE_H_

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geographic_msgs/GeoPoint.h>

#include <fields2cover_ros/F2CConfig.h>
#include <fields2cover.h>

#include "toolpath_generator.hpp"

namespace fields2cover_ros {

  /**
   * @class VisualizerNode
   * @brief A class to visualize fields and paths in ROS.
   */
  class VisualizerNode {
    public:
      /**
       * @brief Initialize the VisualizerNode.
       */
      void init_VisualizerNode();

      /**
       * @brief Publish topics related to field visualization.
       */
      void publish_topics(void);

      /**
       * @brief Callback function for dynamic reconfigure.
       * @param config Configuration parameters.
       * @param level Reconfiguration level.
       */
      void rqt_callback(fields2cover_ros::F2CConfig &config, uint32_t level);

      /**
       * @brief Save the current map.
       */
      void saveMap();

      /**
       * @brief Save the current path.
       * @param path Path to write with default filename
       */
       void savePath(const std::vector<geometry_msgs::PoseStamped>& path);

    private:

      ros::NodeHandle private_node_handle_ { "~" };      ///< Private node handle for ROS.
      ros::NodeHandle public_node_handle_;               ///< Public node handle for ROS.

      //===================================================
      // ROS Path Publishers
      //===================================================
      ros::Publisher field_polygon_publisher_;           ///< Publisher for 2D/3D field border from GPS.
      ros::Publisher field_2d_border_publisher_;         ///< Publisher for 2D field border from GPS with 0 elevation
      ros::Publisher field_no_headlands_publisher_;      ///< Publisher for fields inner headlands.
      ros::Publisher field_swaths_publisher_;            ///< Publisher for field swaths (U turn part).
      // fixed pattern upath trajectory result
      ros::Publisher fixed_pattern_plan_pose_array_pub_; ///< Publisher for fixed pattern plan poses.
      // 2d occupancy grid map
      ros::Publisher map_pub_;                           ///< Publisher for the map.

      nav_msgs::OccupancyGrid occupancy_grid_;  ///< Occupancy grid for the map.
      sensor_msgs::NavSatFix gps_;              ///< GPS data.
      f2c::Transform transf_;                   ///< Transformation utility.

      //===================================================
      // Field2Cover param
      //===================================================

      F2CFields fields_;             ///< Fields data.
      F2CRobot robot_{2.1, 2.5};     ///< Robot configuration.
      double m_swath_angle_;         ///< config.swath_angle
      double m_headland_width_;      ///< config.headland_width

      bool automatic_angle_ {false}; ///< Flag for automatic angle calculation.
      int  sg_objective_   {0};      ///< Objective for the SG algorithm.
      int  opt_turn_type_  {0};      ///< Type of turn optimization.
      int  opt_route_type_ {0};      ///< Type of route optimization.

      //===================================================
      // Save to File
      //===================================================
      // cache file path
      bool is_cache_mode_;           ///< Flag for cache mode.
      std::string cache_directory_;  ///< Directory for cache files.

      // filed file path
      std::string field_file_path_;  ///< Path to the field file.
      int path_file_seq_ = 0;        ///< Sequence number for path files.

      // path reverse flag
      bool reverse_path_ {false};    ///< Flag to reverse the path.

      // U path waypoints interpolation gap
      double interp_step_ = 0.01;    ///< Interpolation step for U path waypoints.

      // trajectory publish frame id
      std::string frame_id_;         ///< Frame ID for trajectory publishing.

      // first gps transfrom from utm frame to map frame
      geometry_msgs::PoseStamped gps2map_transform_; ///< Transform from GPS to map frame.

      //===================================================
      // Spiral Path Param
      //===================================================
      /// spiral path
      ToolpathGenerator* tp_gen_;    ///< spiral path generator

      bool m_spiral_path_ = {false}; ///< flag to spiral path generation

      /**
       * @brief generate single inward spiral given a 2d contour
       * @param contour a 2d contour.
       */
      void generateSingleInwardSpiral(const geometry_msgs::PolygonStamped& contour);

      /**
       * @brief generate u turn swaths into headlands
       * @param no_headlands no_headlands.
       * @return F2CPath sparse path points.
       */
      F2CPath generateSwaths(F2CCell no_headlands);

      /**
       * @brief interpolate waypoints
       * @param path F2CPath.
       * @return waypoints.
       */

      std::vector<geometry_msgs::PoseStamped> interpolateWaypoints(const F2CPath& path);

      /**
       * @brief Transform GPS coordinates to map frame coordinates.
       * @param gps_point GPS point to transform.
       * @return Transformed pose in map frame.
       */
      geometry_msgs::PoseStamped transformGPStoMap(const geographic_msgs::GeoPoint& gps_point);

      /**
       * @brief Transform points in a PolygonStamped message.
       * @param poseTransform Transformation pose.
       * @param polygon Polygon to transform.
       */
      void transformPoints(const geometry_msgs::PoseStamped& poseTransform, geometry_msgs::PolygonStamped& polygon);

      /**
       * @brief Transform points in a path to Marker coordinates.
       * @param poseTransform Transformation pose.
       * @param path Path to transform.
       * @param marker Marker to store transformed points.
       */
      void transformPoints(const geometry_msgs::PoseStamped& poseTransform, const F2CPath& path, visualization_msgs::Marker& marker);

      /**
       * @brief Transform a vector of poses.
       * @param poseTransform Transformation pose.
       * @param poseVec Vector of poses to transform.
       */
      void transformPoses (const geometry_msgs::PoseStamped& poseTransform, std::vector<geometry_msgs::PoseStamped>& poseVec);

      /**
       * @brief publish gps 2d border.
       * @param border border polygon.
       */
      void publish_2d_gps_border(const geometry_msgs::PolygonStamped& border);

      /**
       * @brief Publish a fixed pattern plan.
       * @param path Path to publish.
       * @param pub Publisher to use.
       */
      void publishFixedPatternPlan(const std::vector<geometry_msgs::PoseStamped>& path, const ros::Publisher& pub);

      /**
       * @brief Publish fixed pattern waypoints.
       * @param path Path to publish.
       * @param pub Publisher to use.
       */
      void publishFixedPatternWayPoints(const std::vector<geometry_msgs::PoseStamped>& path, const ros::Publisher& pub);

      /**
       * @brief Convert a PoseStamped message to a Point message.
       * @param pose_stamped PoseStamped message to convert.
       * @return Converted Point message.
       */
      geometry_msgs::Point poseStampedToPoint(const geometry_msgs::PoseStamped& pose_stamped);

      /**
       * @brief Interpolate points between two poses.
       * @param start_point Starting pose.
       * @param end_point Ending pose.
       * @param num_samples Number of samples to interpolate.
       * @param frame_id Frame ID for the interpolated points.
       * @param timestamp Timestamp for the interpolated points.
       * @param interp_path Vector to store interpolated points.
       */
      void interpolatePoints(const geometry_msgs::PoseStamped& start_point, 
                             const geometry_msgs::PoseStamped& end_point, 
                             const int& num_samples,
                             const std::string& frame_id,
                             const ros::Time& timestamp,
                             std::vector<geometry_msgs::PoseStamped>& interp_path);

      /**
       * @brief Interpolate between two poses.
       * @param p0 Starting pose.
       * @param p1 Ending pose.
       * @param t Interpolation factor.
       * @param frame_id Frame ID for the interpolated pose.
       * @param timestamp Timestamp for the interpolated pose.
       * @return Interpolated pose.
       */
      geometry_msgs::PoseStamped interpolate(const geometry_msgs::PoseStamped& p0,
                                             const geometry_msgs::PoseStamped& p1,
                                             const double& t,
                                             const std::string& frame_id,
                                             const ros::Time& timestamp);

      /**
       * @brief Write a path to a file.
       * @param plan Path to write.
       * @param path_file Output file stream.
       */
      void writePathToFile(const std::vector<geometry_msgs::PoseStamped>& plan, std::ofstream& path_file);


      /**
       * @brief Initialize the occupancy grid.
       * @param origin_x X coordinate of the origin.
       * @param origin_y Y coordinate of the origin.
       * @param width Width of the grid.
       * @param height Height of the grid.
       * @param resolution Resolution of the grid.
       */
      void initializeGrid(double origin_x, double origin_y, int width, int height, double resolution);
      
      /**
       * @brief generate 2D grid map.
       * @param border 2D border polygon vertices
       */
      void generateGrid(const geometry_msgs::PolygonStamped& border);

      /**
       * @brief Reverse the orientation of a pose.
       * @param pose Pose to reverse.
       */
      void reverseOrientation(geometry_msgs::PoseStamped& pose);

      /**
       * @brief Parse a GeoJSON file to extract a pose.
       * @param geojson_file Path to the GeoJSON file.
       * @param pose_msg Pose message to store the extracted pose.
       * @return True if parsing was successful, false otherwise.
       */
      bool parseGeoJsonPose(const std::string& geojson_file, geometry_msgs::PoseStamped& pose_msg);
  };
}

#endif  // FIELDS2COVER_ROS_VISUALIZER_NODE_H_
