//=============================================================================
//    Copyright (C) 2021-2022 Wageningen University - All Rights Reserved
//                     Author: Gonzalo Mier
//                        BSD-3 License
//=============================================================================


#include "Fields2CoverVisualizerNode.h"
#include "ros/conversor.h"

#include <fstream>
#include <iostream>
#include <dynamic_reconfigure/server.h>
#include <fields2cover_ros/F2CConfig.h>
#include <nav_msgs/Path.h> // for fixed pattern plan topic publish
#include <geometry_msgs/PoseArray.h>
#include <tf2/LinearMath/Quaternion.h>

#include <spline.h>  // Include the tk::spline header
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <deque>


using namespace std;


namespace fields2cover_ros {
    
  void VisualizerNode::init_VisualizerNode() {
    field_polygon_publisher_      = public_node_handle_.advertise<geometry_msgs::PolygonStamped>("/field/border", 1, true);
    field_no_headlands_publisher_ = public_node_handle_.advertise<geometry_msgs::PolygonStamped>("/field/no_headlands", 1, true);
    field_gps_publisher_          = public_node_handle_.advertise<sensor_msgs::NavSatFix>       ("/gps/fix", 1, true);

    traj_3d_marker_pub_           = public_node_handle_.advertise<visualization_msgs::Marker>   ("/field/traj_3d_marker", 1, true);
    traj_2d_marker_pub_           = public_node_handle_.advertise<visualization_msgs::Marker>   ("/field/traj_2d_marker", 1, true);
    map_pub_                      = public_node_handle_.advertise<nav_msgs::OccupancyGrid>      ("/map", 1, true);

    // Publisher for PoseArray
    fixed_pattern_plan_pose_array_pub_ = public_node_handle_.advertise<geometry_msgs::PoseArray>("/waypoints", 10, true);

    std::string field_file;
    private_node_handle_.getParam("field_file", field_file);
    private_node_handle_.getParam("plan_file_dir", path_file_dir_);

  
    f2c::Parser::importGml(field_file, fields_);

    f2c::Transform::transform(fields_[0], "EPSG:27200");

    robot_.cruise_speed = 2.0;
    robot_.setMinRadius(2.0);
    double headland_width = 3.0 * robot_.op_width;
  }

  geometry_msgs::Quaternion calculateOrientation(const geometry_msgs::Point32& p1, const geometry_msgs::Point32& p2) {
    // Calculate direction vector
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;

    // Calculate yaw (2D direction)
    double yaw = std::atan2(dy, dx);

    // Convert yaw and pitch to a quaternion
    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0, yaw);

    // Convert tf2::Quaternion to geometry_msgs::Quaternion
    geometry_msgs::Quaternion orientation;
    orientation.x = quat.x();
    orientation.y = quat.y();
    orientation.z = quat.z();
    orientation.w = quat.w();
    return orientation;
  }

  void VisualizerNode::initializeGrid(double origin_x, double origin_y, int width, int height, double resolution) {

    occupancy_grid_.header.frame_id = "map";
    // Set the grid resolution (meters per cell)
    occupancy_grid_.info.resolution = resolution;

    // Set the size of the occupancy grid
    occupancy_grid_.info.width = width;
    occupancy_grid_.info.height = height;

    // Set the origin of the occupancy grid
    occupancy_grid_.info.origin.position.x = origin_x;
    occupancy_grid_.info.origin.position.y = origin_y;
    occupancy_grid_.info.origin.position.z = 0.0;
    occupancy_grid_.info.origin.orientation.w = 1.0;  // No rotation

    // Initialize the grid data, all cells are unknown (-1)
    occupancy_grid_.data.resize(width * height, -1);
  }

  geometry_msgs::PoseStamped interpolate(const geometry_msgs::PoseStamped& p0,
                                         const geometry_msgs::PoseStamped& p1,
                                         const double& t) {

    geometry_msgs::PoseStamped result;
    result.pose.position.x = (1 - t) * p0.pose.position.x + t * p1.pose.position.x;
    result.pose.position.y = (1 - t) * p0.pose.position.y + t * p1.pose.position.y;
    result.pose.position.z = 0.0;


    // Interpolate orientation (quaternion linear interpolation, not spherical)
    result.pose.orientation.x = (1 - t) * p0.pose.orientation.x + t * p1.pose.orientation.x;
    result.pose.orientation.y = (1 - t) * p0.pose.orientation.y + t * p1.pose.orientation.y;
    result.pose.orientation.z = (1 - t) * p0.pose.orientation.z + t * p1.pose.orientation.z;
    result.pose.orientation.w = (1 - t) * p0.pose.orientation.w + t * p1.pose.orientation.w;

    // Normalize quaternion
    double norm = std::sqrt(
        result.pose.orientation.x * result.pose.orientation.x +
        result.pose.orientation.y * result.pose.orientation.y +
        result.pose.orientation.z * result.pose.orientation.z +
        result.pose.orientation.w * result.pose.orientation.w);

    result.pose.orientation.x /= norm;
    result.pose.orientation.y /= norm;
    result.pose.orientation.z /= norm;
    result.pose.orientation.w /= norm;

    return result;
  }

  void interpolatePoints(const geometry_msgs::PoseStamped& start_point, 
                         const geometry_msgs::PoseStamped& end_point, 
                         const int& num_samples,
                         std::vector<geometry_msgs::PoseStamped>& interp_path) {

    for (int i = 0; i <= num_samples; ++i) {
      double t = static_cast<double>(i) / num_samples; // Normalized value of t in [0, 1]
      interp_path.push_back(interpolate(start_point, end_point, t));
    }

    ROS_INFO("interp wpt size: %d", int(interp_path.size()));
  }


  // Function to perform cubic spline interpolation on the trajectory with orientation
  std::vector<geometry_msgs::PoseStamped> smoothTrajectory(const std::vector<geometry_msgs::PoseStamped>& origin_traj,
                                                           double interpolation_distance) {

    std::vector<geometry_msgs::PoseStamped> smoothed_traj;

    if (origin_traj.size() < 2) {
        ROS_WARN("Trajectory has less than 2 points; no smoothing performed.");
        return origin_traj;
    }

    // Step 1: Extract positions and calculate cumulative distances
    std::vector<double> distances; // Cumulative distances (parameter t)
    std::vector<double> xs;        // x positions
    std::vector<double> ys;        // y positions

    double cumulative_distance = 0.0;
    distances.push_back(cumulative_distance);
    xs.push_back(origin_traj[0].pose.position.x);
    ys.push_back(origin_traj[0].pose.position.y);

    for (size_t i = 1; i < origin_traj.size(); ++i) {
        double dx = origin_traj[i].pose.position.x - origin_traj[i - 1].pose.position.x;
        double dy = origin_traj[i].pose.position.y - origin_traj[i - 1].pose.position.y;
        cumulative_distance += std::sqrt(dx * dx + dy * dy);
        distances.push_back(cumulative_distance);
        xs.push_back(origin_traj[i].pose.position.x);
        ys.push_back(origin_traj[i].pose.position.y);
    }

    // Step 2: Create spline objects for x(t) and y(t)
    tk::spline spline_x;
    tk::spline spline_y;

    spline_x.set_points(distances, xs);
    spline_y.set_points(distances, ys);

    // Step 3: Generate interpolated points at desired intervals with orientation
    double total_distance = cumulative_distance;
    double t = 0.0;

    while (t <= total_distance) {
      double x = spline_x(t);
      double y = spline_y(t);

      // Approximate derivatives using finite differences
      double delta_t = 0.01; // Small increment to approximate derivative
      double t_forward = t + delta_t;
      if (t_forward > total_distance) {
        t_forward = total_distance;
      }
      double x_forward = spline_x(t_forward);
      double y_forward = spline_y(t_forward);

      double dx = x_forward - x;
      double dy = y_forward - y;

      double yaw = std::atan2(dy, dx); // Compute yaw angle

      // Convert yaw to quaternion
      tf2::Quaternion q;
      q.setRPY(0, 0, yaw);
      geometry_msgs::Quaternion quat_msg = tf2::toMsg(q);

      // Create the PoseStamped message
      geometry_msgs::PoseStamped pose_stamped;
      pose_stamped.header = origin_traj[0].header; // Use the header from the original data
      pose_stamped.pose.position.x = x;
      pose_stamped.pose.position.y = y;
      pose_stamped.pose.position.z = 0.0; // Set z = 0 or interpolate if you have z data
      pose_stamped.pose.orientation = quat_msg;

      smoothed_traj.push_back(pose_stamped);

      t += interpolation_distance;
    }

    // Ensure the last point is included
    if (t < total_distance + interpolation_distance) {
      double x = spline_x(total_distance);
      double y = spline_y(total_distance);

      // Approximate derivatives at the end point
      double delta_t = -0.01; // Use backward difference at the end
      double t_backward = total_distance + delta_t;
      if (t_backward < 0.0) {
          t_backward = 0.0;
      }
      double x_backward = spline_x(t_backward);
      double y_backward = spline_y(t_backward);

      double dx = x - x_backward;
      double dy = y - y_backward;

      double yaw = std::atan2(dy, dx);

      tf2::Quaternion q;
      q.setRPY(0, 0, yaw);
      geometry_msgs::Quaternion quat_msg = tf2::toMsg(q);

      geometry_msgs::PoseStamped pose_stamped;
      pose_stamped.header = origin_traj.back().header;
      pose_stamped.pose.position.x = x;
      pose_stamped.pose.position.y = y;
      pose_stamped.pose.position.z = 0.0;
      pose_stamped.pose.orientation = quat_msg;

      smoothed_traj.push_back(pose_stamped);
    }

    return smoothed_traj;
  }

  // Function to calculate Euclidean distance between two PoseStamped positions
  double calculateDistance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) {
      return std::sqrt(std::pow(p1.x - p2.x, 2) +
                      std::pow(p1.y - p2.y, 2) +
                      std::pow(p1.z - p2.z, 2));
  }


  // Function to downsample trajectory based on distance
  std::vector<geometry_msgs::PoseStamped> downsampleTrajectory(const std::vector<geometry_msgs::PoseStamped>& trajectory, double min_distance) {
    std::vector<geometry_msgs::PoseStamped> downsampled;

    if (trajectory.empty()) return downsampled;

    // Always include the first point
    downsampled.push_back(trajectory[0]);

    // Iterate through the trajectory
    for (size_t i = 1; i < trajectory.size(); ++i) {
        const auto& last_point = downsampled.back().pose.position;
        const auto& current_point = trajectory[i].pose.position;

        // Calculate the distance to the last added point
        double distance = calculateDistance(last_point, current_point);

        // Add the current point if it's farther than the minimum distance
        if (distance >= min_distance) {
            downsampled.push_back(trajectory[i]);
        }
    }

    return downsampled;
  }

  void VisualizerNode::publish_topics(void) {
    auto gps = transf_.getRefPointInGPS(fields_[0]);
    gps_.longitude = gps.getX();
    gps_.latitude = gps.getY();
    gps_.altitude = gps.getZ();
    gps_.header.stamp = ros::Time::now();
    gps_.header.frame_id = "map";
    field_gps_publisher_.publish(gps_);

    auto f = fields_[0].field.clone();

    //================================================
    geometry_msgs::PolygonStamped polygon_st;
    conversor::ROS::to(f.getCellBorder(0), polygon_st.polygon);

    // Create a marker
    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = "map"; // Change to your frame
    line_strip.header.stamp = ros::Time::now();
    line_strip.ns = "traj_3d_marker";
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w = 1.0;

    line_strip.id = 0;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;

    // Set the line color (RGB + alpha)
    line_strip.color.r = 1.0;
    line_strip.color.g = 0.0;
    line_strip.color.b = 0.0;
    line_strip.color.a = 1.0;

    // Set the line width
    line_strip.scale.x = 0.1; // Line width

    // Add points to the marker
    for (const auto& point : polygon_st.polygon.points) {
        geometry_msgs::Point p;
        p.x = point.x;
        p.y = point.y;
        p.z = point.z;
        line_strip.points.push_back(p);
    }

    // traj_3d_marker_pub_.publish(line_strip);
    line_strip.points.clear();

    line_strip.header.stamp = ros::Time::now();
    line_strip.ns = "traj_2d_marker";
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w = 1.0;

    line_strip.id = 0;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;

    // Set the line color (RGB + alpha)
    line_strip.color.r = 0.0;
    line_strip.color.g = 0.0;
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

    // Set the line width
    line_strip.scale.x = 0.1; // Line width

    double angle_rad = 0.0 * M_PI / 180.0;
    double cos_angle = std::cos(angle_rad);
    double sin_angle = std::sin(angle_rad);

    for (size_t i = 0; i <= polygon_st.polygon.points.size() - 1; ++i) {

      geometry_msgs::Point p;
      p.x = polygon_st.polygon.points[i].x * cos_angle - polygon_st.polygon.points[i].y * sin_angle;
      p.y = polygon_st.polygon.points[i].x * sin_angle + polygon_st.polygon.points[i].y * cos_angle;
      polygon_st.polygon.points[i].x = p.x;
      polygon_st.polygon.points[i].y = p.y;
    }

    // Add points to the marker
    for (const auto& point : polygon_st.polygon.points) {
        geometry_msgs::Point p;
        p.x = point.x;
        p.y = point.y;
        p.z = 0.0;
        line_strip.points.push_back(p);
    }
    //================================================
    //================================================

    // //----------------------------------------------------------
    // Calculate the bounding box of the polygon
    double min_x = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double min_y = std::numeric_limits<double>::max();
    double max_y = std::numeric_limits<double>::lowest();

    for (const auto& point : polygon_st.polygon.points) {
      if (point.x < min_x) min_x = point.x;
      if (point.x > max_x) max_x = point.x;
      if (point.y < min_y) min_y = point.y;
      if (point.y > max_y) max_y = point.y;
    }

    // Calculate the width, height, and origin of the occupancy grid
    double width_m = max_x - min_x;
    double height_m = max_y - min_y;
    double resolution = 0.05; // 0.05 meter per cell, can be adjusted as needed

    int grid_width = static_cast<int>(width_m / resolution) + 1;
    int grid_height = static_cast<int>(height_m / resolution) + 1;

    // Initialize the occupancy grid
    initializeGrid(min_x, min_y, grid_width, grid_height, resolution);

    // Publish the updated occupancy grid
    map_pub_.publish(occupancy_grid_);

    //----------------------------------------------------------
    //----------------------------------------------------------

    std::vector<geometry_msgs::PoseStamped> origin_traj;
    for (const auto& point : polygon_st.polygon.points) {
        geometry_msgs::PoseStamped p;
        p.pose.position.x = point.x;
        p.pose.position.y = point.y;
        p.pose.position.z = 0.0;
        origin_traj.push_back(p);
    }

    // Downsample the trajectory
    double min_distance = 0.5; // Minimum distance in meters
    std::vector<geometry_msgs::PoseStamped> denois_traj = downsampleTrajectory(origin_traj, min_distance);
    std::vector<geometry_msgs::PoseStamped> interp_traj = smoothTrajectory(denois_traj, 0.05);
    publishFixedPatternWayPoints(interp_traj, fixed_pattern_plan_pose_array_pub_);

    //----------------------------------------------------------
    //----------------------------------------------------------

    std::ofstream path_file;
    std::string path_file_name = path_file_dir_ + "a2b_path_sample_" + std::to_string(path_file_seq_++) + ".txt";
    path_file.open(path_file_name);

    for (auto& wpt : interp_traj) {
      path_file << wpt.pose.position.x    << " "
                << wpt.pose.position.y    << " "
                << wpt.pose.position.z    << " "
                << wpt.pose.orientation.x << " "
                << wpt.pose.orientation.y << " "
                << wpt.pose.orientation.z << " "
                << wpt.pose.orientation.w << std::endl;
    }

    path_file.close();
    ROS_INFO("%s generated", path_file_name.c_str());

    //================================================
    //================================================

    traj_2d_marker_pub_.publish(line_strip);
    line_strip.points.clear();
    polygon_st.polygon.points.clear();

    //================================================
    // f2c::hg::ConstHL hl_gen_;
    // F2CCell no_headlands = hl_gen_.generateHeadlands(f, optim_.headland_width).getGeometry(0);

    // geometry_msgs::PolygonStamped polygon_st;
    // polygon_st.header.stamp = ros::Time::now();
    // polygon_st.header.frame_id = "map";
    // conversor::ROS::to(f.getCellBorder(0), polygon_st.polygon);
    // field_polygon_publisher_.publish(polygon_st);
    // polygon_st.polygon.points.clear();
    
    // geometry_msgs::PolygonStamped polygon_st2;
    // polygon_st2.header.stamp = ros::Time::now();
    // polygon_st2.header.frame_id = "map";
    // conversor::ROS::to(no_headlands.getGeometry(0), polygon_st2.polygon);
    // field_no_headlands_publisher_.publish(polygon_st2);
    // polygon_st2.polygon.points.clear();

    //================================================
  }

  void VisualizerNode::publishFixedPatternWayPoints(const std::vector<geometry_msgs::PoseStamped>& path, const ros::Publisher& pub) {

    // Create a PoseArray and populate it from the vector
    geometry_msgs::PoseArray pose_array;
    pose_array.header.frame_id = "map";
    pose_array.header.stamp = ros::Time::now();

    for (const auto& pose_stamped : path) {
      pose_array.poses.push_back(pose_stamped.pose); // Add only the pose part
    }

    // Publish the PoseArray
    pub.publish(pose_array);
  }

  void VisualizerNode::rqt_callback(fields2cover_ros::F2CConfig &config, uint32_t level) {
    robot_.op_width = config.op_width;
    robot_.setMinRadius(config.turn_radius);
    optim_.best_angle = config.swath_angle;
    optim_.headland_width = config.headland_width;
    automatic_angle_ = config.automatic_angle;
    sg_objective_ = config.sg_objective;
    opt_turn_type_ = config.turn_type;
    opt_route_type_ = config.route_type;
    publish_topics();
  }
}


