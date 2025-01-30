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

namespace fields2cover_ros {

  class VisualizerNode {
    public:
      void init_VisualizerNode();
      void publish_topics(void);
      void rqt_callback(fields2cover_ros::F2CConfig &config, uint32_t level);
      void saveMap();
    private:
      ros::NodeHandle private_node_handle_ { "~" };
      ros::NodeHandle public_node_handle_;

      ros::Publisher field_polygon_publisher_;
      ros::Publisher field_no_headlands_publisher_;
      ros::Publisher field_swaths_publisher_;
      ros::Publisher map_pub_;

      ros::Publisher field_2d_border_publisher_;
      nav_msgs::OccupancyGrid occupancy_grid_;

      sensor_msgs::NavSatFix gps_;

      f2c::Transform transf_;

      F2CFields fields_;
      F2CRobot robot_{2.1, 2.5};
      F2COptim optim_;

      bool automatic_angle_ {false};
      int sg_objective_ {0};
      int opt_turn_type_ {0};
      int opt_route_type_ {0};

      // filed file path
      std::string field_file_path_;
      int path_file_seq_ = 0;

      // path reverse flag
      bool reverse_path_ {false};

      // U path waypoints interpolation gap
      double interp_step_ = 0.01;

      // trajectory publish frame id
      std::string frame_id_;

      // first gps transfrom from utm frame to map frame
      geometry_msgs::PoseStamped gps2map_transform_;

      // fixed pattern global plan points
      ros::Publisher fixed_pattern_plan_pose_array_pub_;

      // transform GPS point to map frame coord pose
      geometry_msgs::PoseStamped transformGPStoMap(const geographic_msgs::GeoPoint& gps_point);

      // transform PolygonStamped points coord
      void transformPoints(const geometry_msgs::PoseStamped& poseTransform, geometry_msgs::PolygonStamped& polygon);
      
      // transform Path -> Marker points coord 
      void transformPoints(const geometry_msgs::PoseStamped& poseTransform, const F2CPath& path, visualization_msgs::Marker& marker);

      // transform poseVec
      void transformPoses (const geometry_msgs::PoseStamped& poseTransform, std::vector<geometry_msgs::PoseStamped>& poseVec);

      void publishFixedPatternPlan     (const std::vector<geometry_msgs::PoseStamped>& path, const ros::Publisher& pub);
      void publishFixedPatternWayPoints(const std::vector<geometry_msgs::PoseStamped>& path, const ros::Publisher& pub);

      geometry_msgs::Point poseStampedToPoint(const geometry_msgs::PoseStamped& pose_stamped);

      void interpolatePoints(const geometry_msgs::PoseStamped& start_point, 
                             const geometry_msgs::PoseStamped& end_point, 
                             const int& num_samples,
                             const std::string& frame_id,
                             const ros::Time& timestamp,
                             std::vector<geometry_msgs::PoseStamped>& interp_path);

      geometry_msgs::PoseStamped interpolate(const geometry_msgs::PoseStamped& p0,
                                             const geometry_msgs::PoseStamped& p1,
                                             const double& t,
                                             const std::string& frame_id,
                                             const ros::Time& timestamp);

      void writePathToFile(const std::vector<geometry_msgs::PoseStamped>& plan, std::ofstream& path_file);

      void initializeGrid(double origin_x, double origin_y, int width, int height, double resolution);

      // reverse orientation
      void reverseOrientation(geometry_msgs::PoseStamped& pose);

      bool parseGeoJsonPose(const std::string& geojson_file, geometry_msgs::PoseStamped& pose_msg);
  };
}

#endif  // FIELDS2COVER_ROS_VISUALIZER_NODE_H_
