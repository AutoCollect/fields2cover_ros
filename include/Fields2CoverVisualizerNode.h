//=============================================================================
//    Copyright (C) 2021-2022 Wageningen University - All Rights Reserved
//                     Author: Gonzalo Mier
//                        BSD-3 License
//=============================================================================

#ifndef FIELDS2COVER_ROS_VISUALIZER_NODE_H_
#define FIELDS2COVER_ROS_VISUALIZER_NODE_H_

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/PolygonStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <dynamic_reconfigure/server.h>
#include <fields2cover_ros/F2CConfig.h>
#include <fields2cover.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>

#include <geometry_msgs/PointStamped.h>

namespace fields2cover_ros {

  class VisualizerNode {
    public:
      void init_VisualizerNode();
      void publish_topics(void);
      void rqt_callback(fields2cover_ros::F2CConfig &config, uint32_t level);

    private:
      ros::NodeHandle private_node_handle_ { "~" };
      ros::NodeHandle public_node_handle_;

      ros::Publisher field_polygon_publisher_;
      ros::Publisher field_no_headlands_publisher_;
      ros::Publisher field_swaths_publisher_;
      ros::Publisher map_pub_;

      ros::Publisher traj_2d_marker_pub_;
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

      // path reverse flag
      bool reverse_path_ {false};

      // U path waypoints interpolation gap
      double interp_step_ = 0.01;

      // trajectory publish frame id
      std::string frame_id_;

      geometry_msgs::PointStamped map_point_stamped;

      // fixed pattern global plan 
      // std::vector<geometry_msgs::PoseStamped> fixed_pattern_plan_;
      ros::Publisher fixed_pattern_plan_publisher_;

      // fixed pattern global plan points
      ros::Publisher fixed_pattern_plan_pose_array_pub_;

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
      int path_file_seq_ = 0;
      std::string path_file_dir_;

      void initializeGrid(double origin_x, double origin_y, int width, int height, double resolution);

      // reverse orientation
      void reverseOrientation(geometry_msgs::PoseStamped& pose);
  };
}

#endif  // FIELDS2COVER_ROS_VISUALIZER_NODE_H_
