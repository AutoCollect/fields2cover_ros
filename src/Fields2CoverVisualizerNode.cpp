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
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>  // for converting quaternions

#include <geodesy/utm.h>
#include <geometry_msgs/Point.h>
#include <tf2_ros/transform_listener.h>

using namespace std;


namespace fields2cover_ros {
    
  void VisualizerNode::init_VisualizerNode() {
    field_polygon_publisher_      = public_node_handle_.advertise<geometry_msgs::PolygonStamped>("/field/border",       1, true);
    field_2d_border_publisher_    = public_node_handle_.advertise<visualization_msgs::Marker>   ("/field/border_2d",    1, true);

    field_no_headlands_publisher_ = public_node_handle_.advertise<geometry_msgs::PolygonStamped>("/field/no_headlands", 1, true);
    field_swaths_publisher_       = public_node_handle_.advertise<visualization_msgs::Marker>   ("/field/swaths",       1, true);

    // Publisher for PoseArray
    fixed_pattern_plan_pose_array_pub_ = public_node_handle_.advertise<geometry_msgs::PoseArray>("/waypoints", 10, true);

    // Publisher for the occupancy grid map
    map_pub_ = public_node_handle_.advertise<nav_msgs::OccupancyGrid>("/map", 1, true);

    //----------------------------------------------------------
    // field file
    private_node_handle_.param<std::string>("field_file_path", field_file_path_, "/path/to/json");
    std::string field_file = field_file_path_ + "/gps_polygon.json";
    ROS_INFO("[Debug] ROS field file: %s", field_file.c_str());

    // U path waypoints interpolation gap
    private_node_handle_.param("interp_step", interp_step_, 0.01);
    ROS_ERROR("[Debug] interp_step: %f", interp_step_);

    // trajectory publish frame id
    private_node_handle_.param<std::string>("frame_id", frame_id_, std::string("map"));

    // f2c::Parser::importGml(field_file, fields_);
    f2c::Parser::importJson(field_file, fields_);

    fields_[0].setEPSGCoordSystem(4326);
    // Transform the field to the desired coordinate system
    f2c::Transform::transform(fields_[0], "EPSG:32760");

    //----------------------------------------------------------
    // auto ref_point = fields_[0].getRefPoint();
    // ROS_ERROR("%f, %f, %f", ref_point.getX(), ref_point.getY(), ref_point.getZ());
    auto ref_gps_point = f2c::Transform::getRefPointInGPS(fields_[0]);
    ROS_ERROR("%f, %f, %f", ref_gps_point.getX(), ref_gps_point.getY(), ref_gps_point.getZ());

    // Convert GPS (latitude, longitude) to GeoPoint
    geographic_msgs::GeoPoint gps_point;
    gps_point.longitude = ref_gps_point.getX();
    gps_point.latitude  = ref_gps_point.getY();
    gps_point.altitude  = ref_gps_point.getZ();

    gps2map_transform_ = transformGPStoMap(gps_point);
    //----------------------------------------------------------

    robot_.cruise_speed = 2.0;
    robot_.setMinRadius(2.0);
    double headland_width = 3.0*robot_.op_width;
  }

  void VisualizerNode::initializeGrid(double origin_x, double origin_y, int width, int height, double resolution) {

    occupancy_grid_.header.frame_id = frame_id_;
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

  void VisualizerNode::publish_topics(void) {

    //----------------------------------------------------------
    // border GPS contour publish
    auto f = fields_[0].field.clone();
    geometry_msgs::PolygonStamped polygon_st;
    polygon_st.header.stamp = ros::Time::now();
    polygon_st.header.frame_id = frame_id_;

    // transform polygon points coord
    conversor::ROS::to(f.getCellBorder(0), polygon_st.polygon);
    transformPoints(gps2map_transform_, polygon_st);

    field_polygon_publisher_.publish(polygon_st);
    //----------------------------------------------------------
    // calculate 2d GPS and create a marker
    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = frame_id_; // Change to your frame
    line_strip.header.stamp = ros::Time::now();
    line_strip.ns = "2d_border_marker";
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w = 1.0;

    line_strip.id = 0;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;

    // Set the line color (RGB + alpha)
    line_strip.color.r = 0.0;
    line_strip.color.g = 1.0;
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

    // Set the line width
    line_strip.scale.x = 0.2; // Line width

    // Add points to the marker
    for (const auto& point : polygon_st.polygon.points) {
      geometry_msgs::Point p;
      p.x = point.x;
      p.y = point.y;
      p.z = 0.0;
      line_strip.points.push_back(p);
    }

    field_2d_border_publisher_.publish(line_strip);
    //----------------------------------------------------------
    // occupancy grid 2D map creation

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
    double width_m    = max_x - min_x;
    double height_m   = max_y - min_y;
    double resolution = 0.05; // 0.05 meter per cell, can be adjusted as needed

    int grid_width  = static_cast<int>(width_m / resolution)  + 1;
    int grid_height = static_cast<int>(height_m / resolution) + 1;

    // Initialize the occupancy grid
    initializeGrid(min_x, min_y, grid_width, grid_height, resolution);

    // Mark the polygon points in the occupancy grid
    for (const auto& point : polygon_st.polygon.points) {
      int grid_x = static_cast<int>((point.x - min_x) / resolution);
      int grid_y = static_cast<int>((point.y - min_y) / resolution);

      // Ensure the coordinates are within bounds
      if (grid_x >= 0 && grid_x < occupancy_grid_.info.width &&
        grid_y >= 0 && grid_y < occupancy_grid_.info.height) {
        
        int index = grid_y * occupancy_grid_.info.width + grid_x;
        occupancy_grid_.data[index] = 100;  // Mark the cell as occupied
      }
    }

    // Publish the updated occupancy grid
    map_pub_.publish(occupancy_grid_);
    //----------------------------------------------------------
    polygon_st.polygon.points.clear();
    //----------------------------------------------------------
    // no_headlands publisher

    f2c::hg::ConstHL hl_gen_;
    F2CCell no_headlands = hl_gen_.generateHeadlands(f, optim_.headland_width).getGeometry(0);

    geometry_msgs::PolygonStamped polygon_st2;
    polygon_st2.header.stamp = ros::Time::now();
    polygon_st2.header.frame_id = frame_id_;

    // transfrom no_headlands points coord
    conversor::ROS::to(no_headlands.getGeometry(0), polygon_st2.polygon);
    transformPoints(gps2map_transform_, polygon_st2);

    field_no_headlands_publisher_.publish(polygon_st2);
    //----------------------------------------------------------
    polygon_st2.polygon.points.clear();
    //----------------------------------------------------------
    // swaths path generation

    F2CSwaths swaths;
    f2c::sg::BruteForce swath_gen_;
    if (automatic_angle_) {
      switch (sg_objective_) {
        case 0 : {
          f2c::obj::SwathLength obj;
          swaths = swath_gen_.generateBestSwaths(obj, robot_.op_width, no_headlands);
          break;
        }
        case 1 : {
          f2c::obj::NSwath obj;
          swaths = swath_gen_.generateBestSwaths(obj, robot_.op_width, no_headlands);
          break;
        }
        case 2 : {
          f2c::obj::FieldCoverage obj;
          swaths = swath_gen_.generateBestSwaths(obj, robot_.op_width, no_headlands);
          break;
        }
      }
    }
    else {
      swaths = swath_gen_.generateSwaths(optim_.best_angle, robot_.op_width, no_headlands);
    }

    F2CSwaths route;
    switch (opt_route_type_) {
      case 0 : {
        f2c::rp::BoustrophedonOrder swath_sorter;
        route = swath_sorter.genSortedSwaths(swaths);
        break;
      }
      case 1 : {
        f2c::rp::SnakeOrder swath_sorter;
        route = swath_sorter.genSortedSwaths(swaths);
        break;
      }
      case 2 : {
        f2c::rp::SpiralOrder swath_sorter(6);
        route = swath_sorter.genSortedSwaths(swaths);
        break;
      }
      case 3 : {
        f2c::rp::SpiralOrder swath_sorter(4);
        route = swath_sorter.genSortedSwaths(swaths);
        break;
      }
    }

    F2CPath path;
    f2c::pp::PathPlanning path_planner;

    switch(opt_turn_type_) {
      case 0 : {
        f2c::pp::DubinsCurves turn;
        path = path_planner.searchBestPath(robot_, route, turn);
        break;
      }
      case 1 : {
        f2c::pp::DubinsCurvesCC turn;
        path = path_planner.searchBestPath(robot_, route, turn);
        break;
      }
      case 2 : {
        f2c::pp::ReedsSheppCurves turn;
        path = path_planner.searchBestPath(robot_, route, turn);
        break;
      }
      case 3 : {
        f2c::pp::ReedsSheppCurvesHC turn;
        path = path_planner.searchBestPath(robot_, route, turn);
        break;
      }
    }

    visualization_msgs::Marker marker_swaths;
    marker_swaths.header.frame_id = frame_id_;
    marker_swaths.header.stamp = ros::Time::now();
    marker_swaths.action = visualization_msgs::Marker::ADD;
    marker_swaths.pose.orientation.w = 1.0;
    // marker_swaths.type = visualization_msgs::Marker::LINE_STRIP;
    marker_swaths.type = visualization_msgs::Marker::POINTS;
    marker_swaths.scale.x = 0.5;
    marker_swaths.scale.y = 0.5;
    marker_swaths.scale.z = 0.1;

    marker_swaths.color.r = 0.0;   // Red
    marker_swaths.color.g = 0.0;   // Green
    marker_swaths.color.b = 1.0;   // Blue (adjust to get the exact brightness you want)
    marker_swaths.color.a = 1.0;   // Full opacity

    // Transform each point in the polygon
    transformPoints(gps2map_transform_, path, marker_swaths);

    field_swaths_publisher_.publish(marker_swaths);
    //========================================================
    // interpolation with waypoints

    geometry_msgs::PoseStamped pre_wpt;
    pre_wpt.pose.position.x = path.states[0].point.getX();
    pre_wpt.pose.position.y = path.states[0].point.getY();

    double wpt_gap_thresh_hold = 1.0;
    std::vector<geometry_msgs::PoseStamped> fixed_pattern_plan;

    for (auto&& s : path.states) {
      geometry_msgs::PoseStamped cur_wpt;
      cur_wpt.header.frame_id = frame_id_;
      cur_wpt.header.stamp = marker_swaths.header.stamp;

      conversor::ROS::to(s.point, s.angle, cur_wpt);
      double dist = std::hypot((pre_wpt.pose.position.x - cur_wpt.pose.position.x), 
                               (pre_wpt.pose.position.y - cur_wpt.pose.position.y));

      if (dist > wpt_gap_thresh_hold) {
        int num_samples = dist / interp_step_;
        std::vector<geometry_msgs::PoseStamped> interp_path;
        interpolatePoints(pre_wpt, cur_wpt, num_samples, cur_wpt.header.frame_id, cur_wpt.header.stamp, interp_path);
        if (!interp_path.empty() && interp_path.size() > 0) {
          // Add the interp_path to the end of the marker_swaths.points vector
          std::vector<geometry_msgs::Point> interp_pts;
          for (auto& wpt : interp_path) {
            interp_pts.push_back(poseStampedToPoint(wpt));
          }
          fixed_pattern_plan.insert(fixed_pattern_plan.end(), interp_path.begin(), interp_path.end());
        }
      }

      fixed_pattern_plan.push_back(cur_wpt);
      // update pre wpt for next loop
      pre_wpt = cur_wpt;
    }

    //----------------------------------------------------------
    // Transform each point in the polygon
    transformPoses(gps2map_transform_, fixed_pattern_plan);
    //----------------------------------------------------------

    if (reverse_path_) {
      ROS_ERROR("reverse path");
      // reserve orientation
      for (auto& wpt : fixed_pattern_plan) {
        reverseOrientation(wpt);
      }
      std::reverse(fixed_pattern_plan.begin(), fixed_pattern_plan.end());
    }
    //----------------------------------------------------------
    // publish topics
    publishFixedPatternWayPoints(fixed_pattern_plan, fixed_pattern_plan_pose_array_pub_);
    //========================================================
    // save path file
    std::ofstream path_file;
    std::string path_file_name = field_file_path_ + "/u_path_" + std::to_string(path_file_seq_++) + ".txt";
    path_file.open(path_file_name);
    writePathToFile(fixed_pattern_plan, path_file);
    path_file.close();
    ROS_INFO("%s generated", path_file_name.c_str());
    //========================================================
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
    reverse_path_ = config.reverse_path;
    publish_topics();
  }

  void VisualizerNode::publishFixedPatternPlan(const std::vector<geometry_msgs::PoseStamped>& path, const ros::Publisher& pub) {

    //create a message for the plan
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());

    gui_path.header.frame_id = frame_id_;
    gui_path.header.stamp    = ros::Time::now();

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for (unsigned int i = 0; i < path.size(); i++) {
      gui_path.poses[i] = path[i];
    }

    pub.publish(gui_path);
  }


  void VisualizerNode::publishFixedPatternWayPoints(const std::vector<geometry_msgs::PoseStamped>& path, const ros::Publisher& pub) {

    // Create a PoseArray and populate it from the vector
    geometry_msgs::PoseArray pose_array;
    pose_array.header.frame_id = frame_id_;
    pose_array.header.stamp = ros::Time::now();

    for (const auto& pose_stamped : path) {
      pose_array.poses.push_back(pose_stamped.pose); // Add only the pose part
    }

    // Publish the PoseArray
    pub.publish(pose_array);
  }

  geometry_msgs::Point VisualizerNode::poseStampedToPoint(const geometry_msgs::PoseStamped& pose_stamped) {
    geometry_msgs::Point point;

    // Extract position from PoseStamped and assign to Point
    point.x = pose_stamped.pose.position.x;
    point.y = pose_stamped.pose.position.y;
    point.z = pose_stamped.pose.position.z;

    return point;
  }

  // simple linear interpolation
  void VisualizerNode::interpolatePoints(const geometry_msgs::PoseStamped& start_point, 
                                         const geometry_msgs::PoseStamped& end_point, 
                                         const int& num_samples,
                                         const std::string& frame_id,
                                         const ros::Time& timestamp,
                                         std::vector<geometry_msgs::PoseStamped>& interp_path) {
    for (int i = 0; i <= num_samples; ++i) {
      double t = static_cast<double>(i) / num_samples;  // Normalized value of t in [0, 1]
      interp_path.push_back(interpolate(start_point, end_point, t, frame_id, timestamp));
    }

    ROS_INFO("interp wpt size: %d", int(interp_path.size()));
  }

  // Function to perform linear interpolation between two 2D points
  geometry_msgs::PoseStamped VisualizerNode::interpolate(const geometry_msgs::PoseStamped& p0,
                                                         const geometry_msgs::PoseStamped& p1,
                                                         const double& t,
                                                         const std::string& frame_id,
                                                         const ros::Time &timestamp) {
    geometry_msgs::PoseStamped result;

    result.header.frame_id = frame_id;
    result.header.stamp    = timestamp;

    result.pose.position.x = (1 - t) * p0.pose.position.x + t * p1.pose.position.x;
    result.pose.position.y = (1 - t) * p0.pose.position.y + t * p1.pose.position.y;
    result.pose.position.z = 0.0;

    result.pose.orientation = p0.pose.orientation;
    return result;
  }

  void VisualizerNode::writePathToFile(const std::vector<geometry_msgs::PoseStamped>& plan, std::ofstream& path_file) {
    for (auto& wpt : plan) {
      path_file << wpt.pose.position.x    << " "
                << wpt.pose.position.y    << " "
                << wpt.pose.position.z    << " "
                << wpt.pose.orientation.x << " "
                << wpt.pose.orientation.y << " "
                << wpt.pose.orientation.z << " "
                << wpt.pose.orientation.w << std::endl;
    }
  }

  void VisualizerNode::reverseOrientation(geometry_msgs::PoseStamped& pose) {
    // Extract the quaternion from the pose
    tf2::Quaternion q_orig, q_rot, q_new;
    tf2::fromMsg(pose.pose.orientation, q_orig);

    // Create a quaternion representing a 180-degree rotation around the Z-axis (yaw axis)
    q_rot.setRPY(0, 0, M_PI);  // Roll = 0, Pitch = 0, Yaw = 180 degrees (PI radians)

    // Combine the original orientation with the 180-degree rotation
    q_new = q_rot * q_orig;
    q_new.normalize();

    // Set the new orientation in the pose
    pose.pose.orientation = tf2::toMsg(q_new);
  }

  // transform GPS point to map frame coord pose
  geometry_msgs::PoseStamped VisualizerNode::transformGPStoMap(const geographic_msgs::GeoPoint& gps_point) {
    
    // Convert GeoPoint to UTMPoint
    geodesy::UTMPoint utm_point(gps_point);
    ROS_ERROR("%f, %f, %f", utm_point.easting, utm_point.northing, utm_point.altitude);

    // TF buffer and listener
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    
    // GPS in utm frame coord
    geometry_msgs::PoseStamped utm_pose_stamped;
    // GPS in map frame coord
    geometry_msgs::PoseStamped map_pose_stamped;

    // Create a stamped point in the 'utm' frame
    utm_pose_stamped.header.stamp = ros::Time::now();
    utm_pose_stamped.header.frame_id = "utm";
    utm_pose_stamped.pose.position.x = utm_point.easting;
    utm_pose_stamped.pose.position.y = utm_point.northing;
    utm_pose_stamped.pose.position.z = utm_point.altitude;
    utm_pose_stamped.pose.orientation.x = 0.0;
    utm_pose_stamped.pose.orientation.y = 0.0;
    utm_pose_stamped.pose.orientation.z = 0.0;
    utm_pose_stamped.pose.orientation.w = 1.0;

    // Transform the UTM point to the 'map' frame using TF
    try {
      // Attempt to transform from "utm" to "map" within 1 second
      map_pose_stamped = tfBuffer.transform(utm_pose_stamped, "map", ros::Duration(2.0));
      ROS_INFO("UTM -> map transform succeeded!");
      ROS_INFO_STREAM("map_frame_pose: ");
      ROS_INFO_STREAM("  position.x = " << map_pose_stamped.pose.position.x);
      ROS_INFO_STREAM("  position.y = " << map_pose_stamped.pose.position.y);
      ROS_INFO_STREAM("  position.z = " << map_pose_stamped.pose.position.z);
      ROS_INFO_STREAM("  orientation.x = " << map_pose_stamped.pose.orientation.x);
      ROS_INFO_STREAM("  orientation.y = " << map_pose_stamped.pose.orientation.y);
      ROS_INFO_STREAM("  orientation.z = " << map_pose_stamped.pose.orientation.z);
      ROS_INFO_STREAM("  orientation.w = " << map_pose_stamped.pose.orientation.w);
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN_STREAM("Failed to transform UTM to map: " << ex.what());
    }

    return map_pose_stamped;
  }

  // transform PolygonStamped points coord
  void VisualizerNode::transformPoints(const geometry_msgs::PoseStamped& poseTransform, geometry_msgs::PolygonStamped& polygon) {

    tf2::Quaternion q(
      poseTransform.pose.orientation.x,
      poseTransform.pose.orientation.y,
      poseTransform.pose.orientation.z,
      poseTransform.pose.orientation.w
    );
    tf2::Vector3 t(
      poseTransform.pose.position.x,
      poseTransform.pose.position.y,
      poseTransform.pose.position.z
    );

    tf2::Transform transform;
    transform.setOrigin(t);
    transform.setRotation(q);

    // Transform each point in the polygon
    for (auto & pt : polygon.polygon.points) {
      // Original point
      tf2::Vector3 p_in(pt.x, pt.y, pt.z);

      // Apply the transform
      tf2::Vector3 p_out = transform * p_in;

      pt.x = p_out.x();
      pt.y = p_out.y();
      pt.z = p_out.z();
    }
  }

  // transform Path -> Marker points coord
  void VisualizerNode::transformPoints(const geometry_msgs::PoseStamped& poseTransform, const F2CPath& path, visualization_msgs::Marker& marker) {

    tf2::Quaternion q(
      poseTransform.pose.orientation.x,
      poseTransform.pose.orientation.y,
      poseTransform.pose.orientation.z,
      poseTransform.pose.orientation.w
    );
    tf2::Vector3 t(
      poseTransform.pose.position.x,
      poseTransform.pose.position.y,
      poseTransform.pose.position.z
    );

    tf2::Transform transform;
    transform.setOrigin(t);
    transform.setRotation(q);

    // Transform each point in the polygon
    for (auto&& s : path.states) {
      geometry_msgs::Point ros_p;
      conversor::ROS::to(s.point, ros_p);

      // Original point
      tf2::Vector3 p_in(ros_p.x, ros_p.y, ros_p.z);
      // Apply the transform
      tf2::Vector3 p_out = transform * p_in;

      ros_p.x = p_out.x();
      ros_p.y = p_out.y();
      ros_p.z = p_out.z();

      marker.points.push_back(ros_p);
    }
  }

  // transform poseVec
  void VisualizerNode::transformPoses (const geometry_msgs::PoseStamped& poseTransform, std::vector<geometry_msgs::PoseStamped>& poseVec) {

    tf2::Quaternion q(
      poseTransform.pose.orientation.x,
      poseTransform.pose.orientation.y,
      poseTransform.pose.orientation.z,
      poseTransform.pose.orientation.w
    );
    tf2::Vector3 t(
      poseTransform.pose.position.x,
      poseTransform.pose.position.y,
      poseTransform.pose.position.z
    );

    tf2::Transform transform;
    transform.setOrigin(t);
    transform.setRotation(q);

    for (auto & pose_stamped : poseVec) { // Use reference to modify in place
      // Convert to tf2::Transform
      tf2::Transform tf_pose;
      tf2::fromMsg(pose_stamped.pose, tf_pose);

      // Apply the transformation
      tf2::Transform transformed_tf_pose = transform * tf_pose;

      // Convert back to geometry_msgs::Pose
      geometry_msgs::Pose transformed_pose;
      transformed_pose.position.x = transformed_tf_pose.getOrigin().x();
      transformed_pose.position.y = transformed_tf_pose.getOrigin().y();
      transformed_pose.position.z = transformed_tf_pose.getOrigin().z();

      tf2::Quaternion q = transformed_tf_pose.getRotation();
      transformed_pose.orientation.x = q.x();
      transformed_pose.orientation.y = q.y();
      transformed_pose.orientation.z = q.z();
      transformed_pose.orientation.w = q.w();

      // Update the original pose_stamped
      pose_stamped.pose = transformed_pose;
    }
  }
}