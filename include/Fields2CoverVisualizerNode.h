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
#include <geometry_msgs/Point.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geographic_msgs/GeoPoint.h>
#include <nav_msgs/Odometry.h>

#include <fields2cover_ros/F2CConfig.h>
#include <fields2cover.h>

#include "toolpath_generator.hpp"

// Bring the nested types into our local scope for convenience.
using ToolPolyline = ToolpathGenerator::ToolPolyline;
using ToolPoint = ToolpathGenerator::ToolPoint;

namespace fields2cover_ros {

  /**
   * @class VisualizerNode
   * @brief A class to visualize fields and paths in ROS.
   */
  class VisualizerNode {
    public:

      ~VisualizerNode() {
        m_spiral_path_.clear();
        m_uturn_path_.clear();
        m_transition_path_.clear();
        if (tp_gen_) {
          delete tp_gen_;    // Calls the destructor and deallocates memory
          tp_gen_ = nullptr; // Prevents dangling pointer issues
        }
      }

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

      
      void processPaths();

    private:

      ros::NodeHandle private_node_handle_ { "~" };      ///< Private node handle for ROS.
      ros::NodeHandle public_node_handle_;               ///< Public node handle for ROS.

      //===================================================
      // ROS Path Publishers
      //===================================================

      ros::Publisher field_contour_publisher_;           ///< Publisher for 2D/3D filed border and headland contours

      ros::Publisher field_swaths_publisher_;            ///< Publisher for field swaths (U turn part).

      ros::Publisher merge_paths_publisher_;             ///< polyline connection btween spiral path and upath
      // fixed pattern upath trajectory result
      ros::Publisher fixed_pattern_plan_pose_array_pub_; ///< Publisher for fixed pattern plan poses.
      // 2d occupancy grid map
      ros::Publisher map_pub_;                           ///< Publisher for the map.

      nav_msgs::OccupancyGrid occupancy_grid_;  ///< Occupancy grid for the map.
      sensor_msgs::NavSatFix gps_;              ///< GPS data.
      f2c::Transform transf_;                   ///< Transformation utility.

      //===================================================
      // ROS Subscriber
      //===================================================

      ros::Subscriber odom_sub_;
      nav_msgs::Odometry latest_odom_;
    
      // Callback to update the latest odometry data
      void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        latest_odom_ = *msg;
      }

      //===================================================
      // Field2Cover param
      //===================================================
      // upath
      bool m_active_u_path_ {false}; ///< falg to U path generation
      F2CFields fields_;             ///< Fields data.
      // a vehicle to fertilize a field, 
      // with 2 m width and a 6 m operational width
      F2CRobot robot_{2.0, 6.0};     ///< Robot configuration.
      double m_swath_angle_;         ///< config.swath_angle
      double m_headland_width_;      ///< config.headland_width
      // u path reverse flag
      bool reverse_u_path_  {false}; ///< config.reverse_path
      bool automatic_angle_ {false}; ///< Flag for automatic angle calculation.

      int  sg_objective_   {0};      ///< Objective for the SG algorithm.
      int  opt_turn_type_  {0};      ///< Type of turn optimization.
      int  opt_route_type_ {0};      ///< Type of route optimization.

      //===================================================
      // Save to File
      //===================================================

      bool m_save_path_ {false}; ///< Save path file

      std::vector<geometry_msgs::Point> m_spiral_path_;
      std::vector<geometry_msgs::Point> m_uturn_path_;
      std::vector<geometry_msgs::Point> m_transition_path_;
  
      // cache file path
      bool is_cache_mode_;            ///< Flag for cache mode.
      std::string cache_directory_;   ///< Directory for cache files.

      // filed file path
      std::string field_file_path_;   ///< Path to the field file.
      int path_file_seq_ = 0;         ///< Sequence number for path files.

      // U path waypoints interpolation gap
      double interp_step_ = 0.01;     ///< Interpolation step for U path waypoints.

      double m_interp_dist_step_    = 0.05;  ///< linear interpolation distance step.
      double m_interp_angular_step_ = 0.1;   ///< linear interpolation angular step 

      // trajectory publish frame id
      std::string frame_id_;          ///< Frame ID for trajectory publishing.

      // first gps transfrom from utm frame to map frame
      geometry_msgs::PoseStamped gps2map_transform_; ///< Transform from GPS to map frame.

      //===================================================
      // Spiral Path Param
      //===================================================
      /// spiral path
      ToolpathGenerator* tp_gen_;           ///< spiral path generator

      bool m_active_spiral_path_ {false};   ///< flag to spiral path generation

      int m_spiral_trim_num_ = 0;           ///< Spiral trim num

      //===================================================
      // Other Param
      //===================================================
      /// flag for path merge
      bool  m_active_merge_path_ {false}; ///< flag for polyline connection btween spiral path and upath

      /**
       * @brief generate fields 3D/2D contour and headland
       *        1. 3D border GPS contour publish
       *        2. 2D border GPS contour publish
       *        3. headland contour publish
       * @param fields a F2CFields struct.
       * @param border_polygon a F2CFields struct.
       * @param headland_polygon a F2CFields struct.
       */
      F2CCell generateFieldsContour(const F2CFields& fields,
                                    geometry_msgs::PolygonStamped& border_polygon);
      /**
       * @brief generate single inward spiral given a 2d contour
       * @param contour a 2d contour.
       */
      std::vector<geometry_msgs::Point> generateSingleInwardSpiral(const geometry_msgs::PolygonStamped& contour);

      /**
       * @brief generate u turn swaths into headlands
       * @param no_headlands no_headlands.
       * @return sparse path points std::vector<geometry_msgs::Point>.
       */
      std::vector<geometry_msgs::Point> generateSwaths(F2CCell no_headlands);

      /**
       * @brief merge spiral path and u path
       */
       std::vector<geometry_msgs::Point> mergePaths(const std::vector<geometry_msgs::Point>& spiral_path, 
                                                    const std::vector<geometry_msgs::Point>& uturn_path);

      /**
       * @brief generate 2D grid map.
       * @param border 2D border polygon vertices
       */
      void generateGrid(const geometry_msgs::PolygonStamped& border);

      /**
       * @brief convert toolPolyline to ros points
       * @param toolPolyline ToolPolyline.
       * @return ros points std::vector<geometry_msgs::Point>.
       */
      std::vector<geometry_msgs::Point> convertToRosPoints(const ToolPolyline& toolPolyline);

      /**
       * @brief interpolate waypoints
       * @param path F2CPath.
       * @return waypoints.
       */
      std::vector<geometry_msgs::PoseStamped> interpolateWaypoints(const F2CPath& path);

      /**
       * @brief interpolate waypoints
       * @param path std::vector<geometry_msgs::Point>.
       * @return waypoints.
       */
      std::vector<geometry_msgs::PoseStamped> interpolateWaypoints(const std::vector<geometry_msgs::Point>& path);

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
      void transformPoints(const geometry_msgs::PoseStamped& poseTransform, F2CPath& path, visualization_msgs::Marker& marker);

      /**
       * @brief Transform a vector of poses.
       * @param poseTransform Transformation pose.
       * @param poseVec Vector of poses to transform.
       */
      void transformPoses (const geometry_msgs::PoseStamped& poseTransform, std::vector<geometry_msgs::PoseStamped>& poseVec);

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

      // Generate a Bézier curve with a specified number of points.
      std::vector<geometry_msgs::Point> generateBezierCurve(const geometry_msgs::Point& p0,
                                                            const geometry_msgs::Point& p1,
                                                            const geometry_msgs::Point& p2,
                                                            const geometry_msgs::Point& p3,
                                                            int num_points);

      // Compute a unit vector from point 'from' to point 'to'
      geometry_msgs::Point computeUnitVector(const geometry_msgs::Point& from, const geometry_msgs::Point& to);

      // Compute and return the smooth transition curve between spiral_path and uturn_path.
      std::vector<geometry_msgs::Point> computeTransitionCurve(const std::vector<geometry_msgs::Point>& spiral_path, 
                                                               const std::vector<geometry_msgs::Point>& uturn_path);
  };
}

#endif  // FIELDS2COVER_ROS_VISUALIZER_NODE_H_