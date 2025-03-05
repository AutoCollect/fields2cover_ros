//=============================================================================
//    Copyright (C) 2021-2022 Wageningen University - All Rights Reserved
//                     Author: Gonzalo Mier
//                        BSD-3 License
//=============================================================================

#include <fstream>
#include <iostream>

#include <sys/stat.h>   // For stat()
#include <cstdio>       // For remove()

#include <nlohmann/json.hpp>
#include <omp.h>        // OpenMP is enabled
#include <geodesy/utm.h>

#include <Eigen/Geometry>

#include <fields2cover_ros/F2CConfig.h>

#include <dynamic_reconfigure/server.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>  // for converting quaternions
#include <tf2_ros/transform_listener.h>

#include <nav_msgs/Path.h> // for fixed pattern plan topic publish
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>

#include "Fields2CoverVisualizerNode.h"
#include "ros/conversor.h"

using json = nlohmann::json;
using namespace std;

namespace fields2cover_ros {
    
  void VisualizerNode::init_VisualizerNode() {

    field_contour_publisher_      = public_node_handle_.advertise<visualization_msgs::MarkerArray>("/field/contours",     10, true);

    field_swaths_publisher_       = public_node_handle_.advertise<visualization_msgs::MarkerArray>("/field/swaths",       10, true);

    // publisher merge paths connection
    merge_paths_publisher_        = public_node_handle_.advertise<visualization_msgs::Marker>     ("/field/merge_path",   10, true);

    // Publisher for PoseArray
    fixed_pattern_plan_pose_array_pub_ = public_node_handle_.advertise<geometry_msgs::PoseArray>("/waypoints", 10, true);

    // Publisher for the occupancy grid map
    map_pub_ = public_node_handle_.advertise<nav_msgs::OccupancyGrid>("/map", 1, true);

    // Subscriber
    odom_sub_ = public_node_handle_.subscribe("/odom_global", 10, &VisualizerNode::odomCallback, this);

    //----------------------------------------------------------
    // field file
    //----------------------------------------------------------
    // load cache directory
    private_node_handle_.param<bool>       ("cache_mode",       is_cache_mode_,   false);
    private_node_handle_.param<std::string>("cache_directory",  cache_directory_, "/tmp");
    // field file directory
    private_node_handle_.param<std::string>("field_file_path", field_file_path_, "/path/to/json");

    if (is_cache_mode_) {
      field_file_path_ = cache_directory_;
    }

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
    // fetch gps2map_transform
    if (!parseGeoJsonPose(field_file, gps2map_transform_)) {
      auto ref_gps_point = f2c::Transform::getRefPointInGPS(fields_[0]);
      ROS_ERROR("%f, %f, %f", ref_gps_point.getX(), ref_gps_point.getY(), ref_gps_point.getZ());

      // Convert GPS (latitude, longitude) to GeoPoint
      geographic_msgs::GeoPoint gps_point;
      gps_point.longitude = ref_gps_point.getX();
      gps_point.latitude  = ref_gps_point.getY();
      gps_point.altitude  = ref_gps_point.getZ();

      gps2map_transform_ = transformGPStoMap(gps_point);
    }
    //----------------------------------------------------------
    // F2CRobot
    //----------------------------------------------------------
    // getCruiseVel/setCruiseVel: get/set the speed of the vehicle when traveling through the field.
    robot_.setCruiseVel(2.0);
    // getTurnVel/setTurnVel: get/set the speed of the vehicle when making turns or going through the headlands.
    // robot_.setTurnVel(0.5);
    // getMinTurningRadius/setMinTurningRadius and getMaxCurv/setMaxCurv: 
    // get/set the minimum turning radius or the maximum curvature, respectively. 
    // Both are saved as the same parameter, as maximum curvature is the inverse of the minimum turning radius.
    robot_.setMaxCurv(1 / 3.0); // 1 / radius: 1 / 3 m = 0.5
    //----------------------------------------------------------
    // create a new ToolpathGenerator object
    tp_gen_ = new ToolpathGenerator();
  }


  void VisualizerNode::publish_topics(void) {

    //----------------------------------------------------------
    // clear temp paths cache
    //----------------------------------------------------------
    m_spiral_path_.clear();
    m_uturn_path_.clear();
    m_transition_path_.clear();
    std::vector<geometry_msgs::PoseStamped> empty_plan;
    publishFixedPatternWayPoints(empty_plan, fixed_pattern_plan_pose_array_pub_);
    //----------------------------------------------------------
    // define border polygon
    geometry_msgs::PolygonStamped border_polygon;
    //----------------------------------------------------------
    // Field Contour Generation
    F2CCell no_headlands = generateFieldsContour(fields_, border_polygon);
    //----------------------------------------------------------
    // occupancy grid 2D map creation & publish
    generateGrid(border_polygon);
    //----------------------------------------------------------
    // single inward spiral trajectory generation & publish
    m_spiral_path_ = generateSingleInwardSpiral(border_polygon);
    //----------------------------------------------------------
    // clear polygon cache
    border_polygon.polygon.points.clear();
    //----------------------------------------------------------
    // u-turn swaths generation
    m_uturn_path_ = generateSwaths(no_headlands);
    //----------------------------------------------------------
    // generate transition curve for merging spiral path and u_path, then publish
    m_transition_path_ = mergePaths(m_spiral_path_, m_uturn_path_);
    //----------------------------------------------------------
  }

  void VisualizerNode::processPaths() {

    //----------------------------------------------------------
    // define plan
    std::vector<geometry_msgs::PoseStamped> fixed_pattern_plan;
    //----------------------------------------------------------
    // interpolate paths
    //----------------------------------------------------------
    if (!m_spiral_path_.empty() && !m_uturn_path_.empty() && !m_transition_path_.empty()) {
      std::vector<geometry_msgs::Point> path;
      // append all paths
      // A.insert(A.end(), B.begin(), B.end());
      path.insert(path.end(),     m_spiral_path_.begin(),     m_spiral_path_.end());
      path.insert(path.end(), m_transition_path_.begin(), m_transition_path_.end());
      path.insert(path.end(),      m_uturn_path_.begin(),      m_uturn_path_.end());
      fixed_pattern_plan = interpolateWaypoints(path);
      path.clear();
      ROS_ERROR("[Debug] multi_headlands_path");
    }
    else if (!m_spiral_path_.empty() && m_uturn_path_.empty() && m_transition_path_.empty()) {
      fixed_pattern_plan = interpolateWaypoints(m_spiral_path_);
      ROS_ERROR("[Debug] publish spiral_path");
    }
    else if (m_spiral_path_.empty() && !m_uturn_path_.empty() && m_transition_path_.empty()) {
      fixed_pattern_plan = interpolateWaypoints(m_uturn_path_);
      ROS_ERROR("[Debug] publish uturn_path");
    }
    else {
      std::vector<geometry_msgs::PoseStamped> empty_plan;
      publishFixedPatternWayPoints(empty_plan, fixed_pattern_plan_pose_array_pub_);  
      ROS_ERROR("[Debug] publish an empty plan inside m_save_path");
    }

    ROS_ERROR("fixed_pattern_plan size: %d", int(fixed_pattern_plan.size()));

    //----------------------------------------------------------
    // save plan
    //----------------------------------------------------------
    // if (m_save_path_) {
    //   savePath(fixed_pattern_plan);
    // }

  }


  void VisualizerNode::rqt_callback(fields2cover_ros::F2CConfig &config, uint32_t level) {

    //========================================================
    // spiral params
    //========================================================
    m_active_spiral_path_ = config.spiral_path;
    m_spiral_trim_num_    = config.spiral_trim_num;

    // set spiral offset
    tp_gen_->setContourOffset      (config.spiral_headland_width);
    tp_gen_->setOperationWidth     (config.operational_width);
    tp_gen_->setMaxOffsets         (config.spiral_offset_num);
    tp_gen_->setContourResampleStep(config.resample_step);
    tp_gen_->setSpiralReversed     (config.spiral_reversed);
    tp_gen_->setReferenceOffset    (config.reference_offset);

    // Use the cached odometry data to set the spiral entry point
    double x = latest_odom_.pose.pose.position.x;
    double y = latest_odom_.pose.pose.position.y;
    tp_gen_->setSpiralEntryPoint(ToolPoint{x, y});
    // tp_gen_->setSpiralEntryPoint(ToolPoint{0.0, 0.0});
    //========================================================
    // upath params
    //========================================================
    m_active_u_path_ = config.u_path;

    // getCovWidth/setCovWidth: get/set the coverage width of the robot, 
    // also called operational width. 
    // This parameter defines the width of the swaths in the field.
    robot_.setCovWidth(config.operational_width);

    // getMinTurningRadius/setMinTurningRadius and getMaxCurv/setMaxCurv: get/set the minimum turning radius or the maximum curvature, 
    // respectively. Both are saved as the same parameter, as maximum curvature is the inverse of the minimum turning radius.
    if (config.turn_radius != 0.0) {
      robot_.setMaxCurv(1.0 / config.turn_radius);
    }

    m_swath_angle_    = config.swath_angle;
    m_headland_width_ = config.headland_width;
    automatic_angle_  = config.automatic_angle;
    sg_objective_     = config.sg_objective;
    opt_turn_type_    = config.turn_type;
    opt_route_type_   = config.route_type;
    reverse_u_path_   = config.upath_reversed;
    //========================================================
    // common params
    //========================================================

    m_active_merge_path_ = config.merge_path;
    m_save_path_         = config.save_path;

    //========================================================
    // path generation
    if (!m_save_path_) {
      publish_topics();
    }
    else {
      processPaths();
    }
  }

  F2CCell VisualizerNode::generateFieldsContour(const F2CFields& fields,
                                                geometry_msgs::PolygonStamped& border_polygon) {

    auto f = fields[0].getField().clone();
    //----------------------------------------------------------
    // border
    conversor::ROS::to(f.getCellBorder(0), border_polygon.polygon);
    transformPoints(gps2map_transform_, border_polygon);
    //----------------------------------------------------------
    // headland
    f2c::hg::ConstHL hl_gen;
    geometry_msgs::PolygonStamped headland_polygon;
    F2CCell no_headlands = hl_gen.generateHeadlands(f, m_headland_width_).getGeometry(0);
    conversor::ROS::to(no_headlands.getGeometry(0), headland_polygon.polygon);
    transformPoints(gps2map_transform_, headland_polygon);
    //----------------------------------------------------------
    // publish contours
    visualization_msgs::MarkerArray marker_array;

    // --- Create border_3d contour ---

    visualization_msgs::Marker border_3d;
    border_3d.header.frame_id = frame_id_;
    border_3d.ns = "border_3d_contour";
    border_3d.id = 0;
    border_3d.header.stamp = ros::Time::now();
    border_3d.action = visualization_msgs::Marker::ADD;
    border_3d.pose.orientation.w = 1.0;
    // border_3d.type = visualization_msgs::Marker::LINE_STRIP;
    border_3d.type = visualization_msgs::Marker::POINTS;
    border_3d.scale.x = 0.5;
    border_3d.scale.y = 0.5;
    border_3d.scale.z = 0.1;

    border_3d.color.r = 0.0;   // Red
    border_3d.color.g = 0.0;   // Green
    border_3d.color.b = 1.0;   // Blue (adjust to get the exact brightness you want)
    border_3d.color.a = 1.0;   // Full opacity

    // Add points to the marker
    for (const auto& point : border_polygon.polygon.points) {
      geometry_msgs::Point p;
      p.x = point.x;
      p.y = point.y;
      p.z = point.z;
      border_3d.points.push_back(p);
    }

    // --- Create border_2d contour ---

    visualization_msgs::Marker border_2d;
    border_2d.header.frame_id = frame_id_; // Change to your frame
    border_2d.header.stamp = ros::Time::now();
    border_2d.ns = "border_2d_contour";
    border_2d.action = visualization_msgs::Marker::ADD;
    border_2d.pose.orientation.w = 1.0;

    border_2d.id = 1;
    border_2d.type = visualization_msgs::Marker::LINE_STRIP;

    // Set the line color (RGB + alpha)
    border_2d.color.r = 1.0;
    border_2d.color.g = 1.0;
    border_2d.color.b = 1.0;
    border_2d.color.a = 1.0;

    // Set the line width
    border_2d.scale.x = 0.2; // Line width

    // Add points to the marker
    for (const auto& point : border_polygon.polygon.points) {
      geometry_msgs::Point p;
      p.x = point.x;
      p.y = point.y;
      p.z = 0.0;
      border_2d.points.push_back(p);
    }

    // --- Create headland contour ---

    visualization_msgs::Marker headland_marker;
    headland_marker.header.frame_id = frame_id_; // Change to your frame
    headland_marker.header.stamp = ros::Time::now();
    headland_marker.ns = "headland_marker";
    headland_marker.action = visualization_msgs::Marker::ADD;
    headland_marker.pose.orientation.w = 1.0;

    headland_marker.id = 2;
    headland_marker.type = visualization_msgs::Marker::LINE_STRIP;

    // Set the headland color (RGB + alpha)
    headland_marker.color.r = 1.0;
    headland_marker.color.g = 1.0;
    headland_marker.color.b = 0.0;
    headland_marker.color.a = 1.0;
    
    // Set the line width
    headland_marker.scale.x = 0.2; // Line width

    // Add points to the marker
    for (const auto& point : headland_polygon.polygon.points) {
      geometry_msgs::Point p;
      p.x = point.x;
      p.y = point.y;
      p.z = 0.0;
      headland_marker.points.push_back(p);
    }

    //-----------------------------------
    marker_array.markers.push_back(border_3d);
    marker_array.markers.push_back(border_2d);
    marker_array.markers.push_back(headland_marker);

    // publish fields contour and headland
    field_contour_publisher_.publish(marker_array);
    //-----------------------------------
    return no_headlands;
  }

  std::vector<geometry_msgs::Point> VisualizerNode::generateSwaths(F2CCell no_headlands) {

    if (!m_active_u_path_) {
      visualization_msgs::MarkerArray delete_all;
      visualization_msgs::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = ros::Time::now();
      marker.action = visualization_msgs::Marker::DELETEALL;  // DELETEALL action
      delete_all.markers.push_back(marker);
      field_swaths_publisher_.publish(delete_all);
      std::vector<geometry_msgs::Point> empty_upath;
      return empty_upath;
    }

    // swaths path generation
    F2CSwaths swaths;
    f2c::sg::BruteForce swath_gen;
    if (automatic_angle_) {
      switch (sg_objective_) {
        case 0 : {
          f2c::obj::SwathLength obj;
          swaths = swath_gen.generateBestSwaths(obj, robot_.getCovWidth(), no_headlands);
          break;
        }
        case 1 : {
          f2c::obj::NSwath obj;
          swaths = swath_gen.generateBestSwaths(obj, robot_.getCovWidth(), no_headlands);
          break;
        }
        case 2 : {
          f2c::obj::FieldCoverage obj;
          swaths = swath_gen.generateBestSwaths(obj, robot_.getCovWidth(), no_headlands);
          break;
        }
      }
    }
    else {
      swaths = swath_gen.generateSwaths(m_swath_angle_, robot_.getCovWidth(), no_headlands);
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
        path = path_planner.planPath(robot_, route, turn);
        break;
      }
      case 1 : {
        f2c::pp::DubinsCurvesCC turn;
        path = path_planner.planPath(robot_, route, turn);
        break;
      }
      case 2 : {
        f2c::pp::ReedsSheppCurves turn;
        path = path_planner.planPath(robot_, route, turn);
        break;
      }
      case 3 : {
        f2c::pp::ReedsSheppCurvesHC turn;
        path = path_planner.planPath(robot_, route, turn);
        break;
      }
    }

    visualization_msgs::MarkerArray marker_array;

    // --- Create  upath  ---
    visualization_msgs::Marker marker_swaths;
    marker_swaths.header.frame_id = frame_id_;
    marker_swaths.ns = "upath";
    marker_swaths.id = 0;
    marker_swaths.header.stamp = ros::Time::now();
    marker_swaths.action = visualization_msgs::Marker::ADD;
    marker_swaths.pose.orientation.w = 1.0;
    marker_swaths.type = visualization_msgs::Marker::LINE_STRIP;
    // marker_swaths.type = visualization_msgs::Marker::POINTS;
    marker_swaths.scale.x = 0.5;
    marker_swaths.scale.y = 0.5;
    marker_swaths.scale.z = 0.1;

    marker_swaths.color.r = 0.0;   // Red
    marker_swaths.color.g = 1.0;   // Green
    marker_swaths.color.b = 0.0;   // Blue (adjust to get the exact brightness you want)
    marker_swaths.color.a = 0.5;   // Full opacity

    // Transform each point in the polygon
    transformPoints(gps2map_transform_, path, marker_swaths);

    // check reverse Upath
    if (reverse_u_path_) {
      std::reverse(marker_swaths.points.begin(), marker_swaths.points.end());
    }

    // --- Create a marker for the first point of upath (blue big point) ---
    visualization_msgs::Marker first_point_marker;
    first_point_marker.header.frame_id = frame_id_;
    first_point_marker.header.stamp = ros::Time::now();
    first_point_marker.ns = "upath_first_point";
    first_point_marker.id = 1;
    // first_point_marker.type = visualization_msgs::Marker::SPHERE;
    first_point_marker.type   = visualization_msgs::Marker::CUBE;
    first_point_marker.action = visualization_msgs::Marker::ADD;
    first_point_marker.pose.orientation.w = 1.0;
    // Set a larger scale for the sphere (big point)
    first_point_marker.scale.x = 3.0;
    first_point_marker.scale.y = 3.0;
    first_point_marker.scale.z = 3.0;
    // Set blue color: (R=0, G=0, B=1, A=1)
    first_point_marker.color.r = 1.0;
    first_point_marker.color.g = 0.0;
    first_point_marker.color.b = 0.0;
    first_point_marker.color.a = 1.0;
    if (!marker_swaths.points.empty()) {
      first_point_marker.pose.position.x = marker_swaths.points.front().x;
      first_point_marker.pose.position.y = marker_swaths.points.front().y;
      first_point_marker.pose.position.z = 0.0;
    }

    // --- Create a marker for the last point of upath (red big point) ---
    visualization_msgs::Marker last_point_marker;
    last_point_marker.header.frame_id = frame_id_;
    last_point_marker.header.stamp = ros::Time::now();
    last_point_marker.ns = "upath_last_point";
    last_point_marker.id = 2;
    last_point_marker.type = visualization_msgs::Marker::SPHERE;
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
    if (!marker_swaths.points.empty()) {
      last_point_marker.pose.position.x = marker_swaths.points.back().x;
      last_point_marker.pose.position.y = marker_swaths.points.back().y;
      last_point_marker.pose.position.z = 0.0;
    }

    // --- publish swaths ---
    marker_array.markers.push_back(marker_swaths);
    marker_array.markers.push_back(first_point_marker);
    marker_array.markers.push_back(last_point_marker);

    field_swaths_publisher_.publish(marker_array);

    // --- return ---
    return std::move(marker_swaths.points);
  }


  std::vector<geometry_msgs::Point> VisualizerNode::generateSingleInwardSpiral(const geometry_msgs::PolygonStamped& contour) {

    if (!m_active_spiral_path_) {
      tp_gen_->deleteMarkers();
      std::vector<geometry_msgs::Point> empty_spiral_path;
      return empty_spiral_path;
    }
    else {
      tp_gen_->deleteMarkers ();
      //============================================
      ToolPolyline polygon;
      for (const auto& point : contour.polygon.points) {
        polygon.push_back(ToolPoint {point.x, point.y});
      }
      std::string name = "Paddock_Test";
      std::cout << "\n==== Processing Polygon: " << name << " ====\n";
      //============================================
      // 1. raw polygon data
      // tp_gen_->setPolygonName(name);
      // tp_gen_->setContour(polygon);
      //============================================
      // 2. re-generate/re-arrange contour sequence before offset
      //   a. contour sequence start point is close to the given point
      //   b. closewise or anti-clockwise direction
      //--------------------------------------------
      // tp_gen_->setPolygonName(name);
      // const ToolPolyline &polygon_contour = 
      //   tp_gen_->generateContour(polygon.front(), polygon);
      // tp_gen_->setContour(polygon_contour);
      //============================================
      // 3. resampling + relocate
      tp_gen_->setPolygonName(name);
      // tp_gen_->setSpiralEntryPoint(ToolPoint{0.0, 0.0});
      tp_gen_->setContour(polygon);
      //============================================
      try {
          tp_gen_->archimedeanSpiral();
          tp_gen_->trimSpiralPath(m_spiral_trim_num_);
          tp_gen_->plotPath();
      } catch (const std::exception &e) {
          std::cerr << "Error in processing polygon " << name << ": " << e.what() << "\n";
      }

      // --- return ---
      return convertToRosPoints(tp_gen_->getEntrySpiral());
    }
  }

  std::vector<geometry_msgs::Point> VisualizerNode::mergePaths(
                                  const std::vector<geometry_msgs::Point>& spiral_path, 
                                  const std::vector<geometry_msgs::Point>& uturn_path) {

    std::vector<geometry_msgs::Point> transitionCurve;

    if (m_active_spiral_path_ && m_active_u_path_ && m_active_merge_path_) {

      transitionCurve = computeTransitionCurve(spiral_path, uturn_path);

      // create a merge marker
      visualization_msgs::Marker merge_paths_marker;
      merge_paths_marker.header.frame_id = frame_id_; // Change to your frame
      merge_paths_marker.header.stamp = ros::Time::now();
      merge_paths_marker.ns = "merge_marker";
      merge_paths_marker.action = visualization_msgs::Marker::ADD;
      merge_paths_marker.type = visualization_msgs::Marker::LINE_STRIP;
      merge_paths_marker.pose.orientation.w = 1.0;
      merge_paths_marker.id = 0;

      // Set the line width
      merge_paths_marker.scale.x = 0.5; // Line width

      // Set the line color (RGB yellow + alpha)
      merge_paths_marker.color.r = 1.0;
      merge_paths_marker.color.g = 0.0;
      merge_paths_marker.color.b = 1.0;
      merge_paths_marker.color.a = 1.0;

      // Add point to the marker
      merge_paths_marker.points = transitionCurve;

      // publish merge marker
      merge_paths_publisher_.publish(merge_paths_marker);
    }
    else {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = ros::Time::now();
      marker.action = visualization_msgs::Marker::DELETEALL;
      merge_paths_publisher_.publish(marker);
    }

    return transitionCurve;
  }

  // Helper function: generate a cubic Bézier curve given 4 control points and a desired resolution.
  std::vector<geometry_msgs::Point> VisualizerNode::generateBezierCurve(const geometry_msgs::Point& p0,
                                                                        const geometry_msgs::Point& p1,
                                                                        const geometry_msgs::Point& p2,
                                                                        const geometry_msgs::Point& p3,
                                                                        int num_points) 
  {
    std::vector<geometry_msgs::Point> curve;
    curve.reserve(num_points + 1);
    
    // Convert the cubic Bézier to polynomial form: p(t) = a*t^3 + b*t^2 + c*t + d
    double ax = -p0.x + 3 * p1.x - 3 * p2.x + p3.x;
    double bx = 3 * p0.x - 6 * p1.x + 3 * p2.x;
    double cx = -3 * p0.x + 3 * p1.x;
    double dx = p0.x;
    
    double ay = -p0.y + 3 * p1.y - 3 * p2.y + p3.y;
    double by = 3 * p0.y - 6 * p1.y + 3 * p2.y;
    double cy = -3 * p0.y + 3 * p1.y;
    double dy = p0.y;
    
    double az = -p0.z + 3 * p1.z - 3 * p2.z + p3.z;
    double bz = 3 * p0.z - 6 * p1.z + 3 * p2.z;
    double cz = -3 * p0.z + 3 * p1.z;
    double dz = p0.z;
    
    double dt = 1.0 / num_points;
    double dt2 = dt * dt;
    double dt3 = dt2 * dt;
    
    // Forward differences for x coordinate:
    double x = dx;
    double dx1 = cx * dt + bx * dt2 + ax * dt3;
    double dx2 = 2 * bx * dt2 + 6 * ax * dt3;
    double dx3 = 6 * ax * dt3;
    
    // Forward differences for y coordinate:
    double y = dy;
    double dy1 = cy * dt + by * dt2 + ay * dt3;
    double dy2 = 2 * by * dt2 + 6 * ay * dt3;
    double dy3 = 6 * ay * dt3;
    
    // Forward differences for z coordinate:
    double z = dz;
    double dz1 = cz * dt + bz * dt2 + az * dt3;
    double dz2 = 2 * bz * dt2 + 6 * az * dt3;
    double dz3 = 6 * az * dt3;
    
    for (int i = 0; i <= num_points; ++i) {
        geometry_msgs::Point pt;
        pt.x = x;
        pt.y = y;
        pt.z = z;
        curve.push_back(pt);
        
        x += dx1;
        dx1 += dx2;
        dx2 += dx3;
        
        y += dy1;
        dy1 += dy2;
        dy2 += dy3;
        
        z += dz1;
        dz1 += dz2;
        dz2 += dz3;
    }
    
    return curve;
  }

  // Helper function: computes a unit vector from 'from' to 'to'
  geometry_msgs::Point VisualizerNode::computeUnitVector(const geometry_msgs::Point& from, 
                                                         const geometry_msgs::Point& to) 
  {
    geometry_msgs::Point vec;
    vec.x = to.x - from.x;
    vec.y = to.y - from.y;
    vec.z = to.z - from.z;
    double norm = std::hypot(vec.x, vec.y, vec.z);

    if (norm > 1e-6) {
      vec.x /= norm;
      vec.y /= norm;
      vec.z /= norm;
    }
    return vec;
  }

  // Improved function: Compute a smooth transition curve (using a cubic Bézier curve)
  // between the end of the spiral_path and the start of the uturn_path.
  std::vector<geometry_msgs::Point> VisualizerNode::computeTransitionCurve(
      const std::vector<geometry_msgs::Point>& spiral_path,
      const std::vector<geometry_msgs::Point>& uturn_path) {

    std::vector<geometry_msgs::Point> transitionCurve;
    if (spiral_path.empty() || uturn_path.empty())
    return transitionCurve;

    // p0: last point of the spiral path.
    geometry_msgs::Point p0 = spiral_path.back();
    // p3: first point of the U-turn path.
    geometry_msgs::Point p3 = uturn_path.front();

    // Compute heading at the end of the spiral using its last two points.
    geometry_msgs::Point p_prev = (spiral_path.size() >= 2) ? spiral_path[spiral_path.size() - 2] : spiral_path.back();
    geometry_msgs::Point headingSpiral = computeUnitVector(p_prev, p0);

    // Compute heading at the beginning of the U-turn using its first two points.
    geometry_msgs::Point p_next = (uturn_path.size() >= 2) ? uturn_path[1] : uturn_path.front();
    geometry_msgs::Point headingUturn = computeUnitVector(p3, p_next);

    // Calculate the straight-line distance between p0 and p3 using std::hypot for 3D distance
    double dx = p3.x - p0.x;
    double dy = p3.y - p0.y;
    double dz = p3.z - p0.z;
    double distance = std::hypot(dx, dy, dz); // C++17 style
  
    // Compute the angle between the two headings.
    double dotProduct = headingSpiral.x * headingUturn.x +
                        headingSpiral.y * headingUturn.y +
                        headingSpiral.z * headingUturn.z;
    // Clamp the dot product to avoid numerical issues.
    dotProduct = std::max(-1.0, std::min(1.0, dotProduct));
    double angle = std::acos(dotProduct);

    // Adaptively set the control distance:
    // Use a base scale (e.g., 0.25) and add extra length proportional to the angle.
    double controlScale = 0.25 + 0.5 * (angle / M_PI);
    double controlDist = distance * controlScale;

    // Define the control points for the Bézier curve.
    geometry_msgs::Point p1;
    p1.x = p0.x + headingSpiral.x * controlDist;
    p1.y = p0.y + headingSpiral.y * controlDist;
    p1.z = p0.z + headingSpiral.z * controlDist;

    geometry_msgs::Point p2;
    p2.x = p3.x - headingUturn.x * controlDist;
    p2.y = p3.y - headingUturn.y * controlDist;
    p2.z = p3.z - headingUturn.z * controlDist;

    // Optionally, choose a finer resolution for sharper transitions.
    int num_points = (angle > 0.5) ? 20 : 10;

    // Generate the Bézier transition curve.
    transitionCurve = generateBezierCurve(p0, p1, p2, p3, num_points);

    return transitionCurve;
  }

  std::vector<geometry_msgs::Point> VisualizerNode::convertToRosPoints(const ToolPolyline& toolPolyline) {
    std::vector<geometry_msgs::Point> rosPoints;
    rosPoints.resize(toolPolyline.size());  // Preallocate memory

    std::transform(toolPolyline.begin(), toolPolyline.end(), rosPoints.begin(),
                   [](const ToolPoint& tp) {
                       geometry_msgs::Point p;
                       p.x = tp.x;
                       p.y = tp.y;
                       p.z = 0.0;  // z is zero
                       return p;
                   });
    return rosPoints;
  }


  std::vector<geometry_msgs::PoseStamped> VisualizerNode::interpolateWaypoints(const std::vector<geometry_msgs::Point>& path) {
    std::vector<geometry_msgs::PoseStamped> fixed_pattern_plan;
    if (path.size() < 2)
      return fixed_pattern_plan;  // Nothing to interpolate if there is only one point
  
    // Process each segment from path[i] to path[i+1]
    for (size_t i = 0; i < path.size() - 1; ++i) {
      const geometry_msgs::Point& start = path[i];
      const geometry_msgs::Point& end   = path[i + 1];
  
      double dx = end.x - start.x;
      double dy = end.y - start.y;
      double dz = end.z - start.z;
      double segment_length = std::sqrt(dx * dx + dy * dy + dz * dz);
      if (segment_length < 1e-6)
        continue;  // Skip if the segment is too short
  
      // Compute the yaw for the current segment
      double current_yaw = std::atan2(dy, dx);
      // Convert the yaw into a quaternion using Eigen
      Eigen::Quaterniond q_start(Eigen::AngleAxisd(current_yaw, Eigen::Vector3d::UnitZ()));
  
      // Determine the target orientation using the next segment (if available)
      double next_yaw = current_yaw;
      if (i < path.size() - 2) {
        const geometry_msgs::Point& next_point = path[i + 2];
        double dx_next = next_point.x - end.x;
        double dy_next = next_point.y - end.y;
        next_yaw = std::atan2(dy_next, dx_next);
      }
      Eigen::Quaterniond q_end(Eigen::AngleAxisd(next_yaw, Eigen::Vector3d::UnitZ()));
  
      // Compute step counts for position and orientation interpolation
      // (angular difference is still used to choose an appropriate number of steps)
      double delta_yaw = next_yaw - current_yaw;
      while (delta_yaw > M_PI)  delta_yaw -= 2 * M_PI;
      while (delta_yaw < -M_PI) delta_yaw += 2 * M_PI;
  
      int steps_pos = std::ceil(segment_length / m_interp_dist_step_);
      int steps_ang = (std::fabs(delta_yaw) > 1e-6) ? std::ceil(std::fabs(delta_yaw) / m_interp_angular_step_) : 1;
      int steps = std::max(steps_pos, steps_ang);
      steps = std::max(steps, 1);  // Ensure at least one step
  
      // For the very first segment, add the starting pose.
      if (i == 0) {
        geometry_msgs::PoseStamped ps;
        ps.pose.position = start;
        // Convert Eigen::Quaterniond to tf2::Quaternion
        tf2::Quaternion tf_q_start(q_start.x(), q_start.y(), q_start.z(), q_start.w());
        // Now convert to a geometry_msgs::Quaternion message
        ps.pose.orientation = tf2::toMsg(tf_q_start);
        fixed_pattern_plan.push_back(ps);
      }
  
      // Interpolate along the segment.
      // For each step, compute linear interpolation for position and slerp for orientation.
      for (int step = 1; step <= steps; ++step) {
        double t = static_cast<double>(step) / steps;
        geometry_msgs::PoseStamped ps;
        ps.pose.position.x = start.x + t * dx;
        ps.pose.position.y = start.y + t * dy;
        ps.pose.position.z = start.z + t * dz;
  
        // Use slerp to smoothly interpolate between q_start and q_end
        Eigen::Quaterniond q_interp = q_start.slerp(t, q_end);
        // Convert Eigen::Quaterniond to tf2::Quaternion
        tf2::Quaternion tf_q_interp(q_interp.x(), q_interp.y(), q_interp.z(), q_interp.w());
        // Now convert to a geometry_msgs::Quaternion message
        ps.pose.orientation = tf2::toMsg(tf_q_interp);
        fixed_pattern_plan.push_back(ps);
      }
    }
  
    // Publish topics (assumes publishFixedPatternWayPoints is defined elsewhere)
    publishFixedPatternWayPoints(fixed_pattern_plan, fixed_pattern_plan_pose_array_pub_);

    return fixed_pattern_plan;
  }


  std::vector<geometry_msgs::PoseStamped> VisualizerNode::interpolateWaypoints(const F2CPath& path) {
    // interpolation with waypoints
    geometry_msgs::PoseStamped pre_wpt;
    path.getStates()[0].point.getX();
    pre_wpt.pose.position.x = path.getStates()[0].point.getX();
    pre_wpt.pose.position.y = path.getStates()[0].point.getY();
    double wpt_gap_thresh_hold = 1.0;
    std::vector<geometry_msgs::PoseStamped> fixed_pattern_plan;

    for (auto&& s : path.getStates()) {
      geometry_msgs::PoseStamped cur_wpt;
      cur_wpt.header.frame_id = frame_id_;
      cur_wpt.header.stamp = ros::Time::now();

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

    // if (reverse_path_) {
    //   ROS_ERROR("reverse path");
    //   // reserve orientation
    //   for (auto& wpt : fixed_pattern_plan) {
    //     reverseOrientation(wpt);
    //   }
    //   std::reverse(fixed_pattern_plan.begin(), fixed_pattern_plan.end());
    // }
    
    // publish topics
    publishFixedPatternWayPoints(fixed_pattern_plan, fixed_pattern_plan_pose_array_pub_);

    // interpolated waypoints
    return fixed_pattern_plan;
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

    // If the path is not empty, add poses. Otherwise, leave pose_array.poses empty.
    if (!path.empty()) {
      for (const auto& pose_stamped : path) {
        pose_array.poses.push_back(pose_stamped.pose);
      }
    }
    
    // Publish the (possibly empty) PoseArray so RViz will update its display
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
                << 0                      << " "
                // << wpt.pose.position.z << " "
                << wpt.pose.orientation.x << " "
                << wpt.pose.orientation.y << " "
                << wpt.pose.orientation.z << " "
                << wpt.pose.orientation.w << std::endl;
    }
  }

  void VisualizerNode::savePath(const std::vector<geometry_msgs::PoseStamped>& path) {
    if (m_save_path_ && !path.empty()) {
      std::ofstream path_file;
      std::string path_file_name;
  
      if (!is_cache_mode_) {
        path_file_name = field_file_path_ + "/u_path_" + std::to_string(path_file_seq_++) + ".txt";
      }
      else { // cache mode
        path_file_name = field_file_path_ + "/path.txt";
  
        // Check if file exists
        struct stat buffer;
        if (stat(path_file_name.c_str(), &buffer) == 0) {
          // File already exists; remove it
          if (std::remove(path_file_name.c_str()) != 0) {
            ROS_ERROR("Failed to remove existing file: %s", path_file_name.c_str());
            // Optionally, handle the error (return / exit / etc.)
          } else {
            ROS_INFO("Removed existing file: %s", path_file_name.c_str());
          }
        }
      }
  
      path_file.open(path_file_name);
      writePathToFile(path, path_file);
      path_file.close();
      ROS_INFO("%s generated", path_file_name.c_str());  
    } 
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

  void VisualizerNode::generateGrid(const geometry_msgs::PolygonStamped& border) {

    // Calculate the bounding box of the polygon
    double min_x = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double min_y = std::numeric_limits<double>::max();
    double max_y = std::numeric_limits<double>::lowest();

    for (const auto& point : border.polygon.points) {
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
    for (const auto& point : border.polygon.points) {
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
  }


  // Helper function to write the occupancy grid into PGM + YAML.
  void VisualizerNode::saveMap() {
    // 1. Create filenames for .pgm and .yaml
    const std::string mapdatafile = field_file_path_ + "/map.pgm";
    const std::string yamlFile    = field_file_path_ + "/map.yaml";

    // 2. Open the PGM file
    std::ofstream out(mapdatafile.c_str(), std::ios::out | std::ios::binary);
    if (!out)
    {
      throw std::runtime_error("Could not save map file to " + mapdatafile);
    }

    // 3. Write the PGM header
    //    "P5" => binary PGM, width, height, and max grayscale value (255).
    unsigned int width  = occupancy_grid_.info.width;
    unsigned int height = occupancy_grid_.info.height;
    out << "P5\n" << width << " " << height << "\n255\n";

    // 4. Write data to the PGM file
    //    - Typically:
    //      0   => Occupied (100 in map.data)
    //      254 => Free (0 in map.data)
    //      205 => Unknown (-1 in map.data)
    //
    //    The grid is stored row-major starting at (0,0) at the lower-left corner of the map.
    //    But in the PGM we typically save top row first => we must invert row order.
    
    for (int y = height - 1; y >= 0; --y)
    {
      for (unsigned int x = 0; x < width; ++x)
      {
        int i = x + y * width;
        int8_t val = occupancy_grid_.data[i];

        unsigned char pixel = 205; // default for unknown
        if (val == 0)
        {
          pixel = 254; // free
        }
        else if (val == 100)
        {
          pixel = 0;   // occupied
        }
        // else -1 or other => unknown => 205

        out.write(reinterpret_cast<char*>(&pixel), sizeof(unsigned char));
      }
    }

    out.close();

    // 5. Generate a .yaml file describing the map
    //    - resolution: float
    //    - origin: [x, y, theta]
    //    - negate, occupied_thresh, free_thresh
    //    - image: name of the .pgm

    std::ofstream yaml(yamlFile.c_str());
    if (!yaml)
    {
      throw std::runtime_error("Could not save map YAML to " + yamlFile);
    }

    yaml << "image: " << "map.pgm" << "\n";
    yaml << "resolution: " << occupancy_grid_.info.resolution << "\n";
    // origin is in the format [x, y, theta].
    yaml << "origin: [" 
        << occupancy_grid_.info.origin.position.x << ", "
        << occupancy_grid_.info.origin.position.y << ", "
        << tf::getYaw(occupancy_grid_.info.origin.orientation) << "]\n";
    yaml << "negate: 0\n";
    yaml << "occupied_thresh: 0.65\n";
    yaml << "free_thresh: 0.196\n";

    yaml.close();

    ROS_INFO_STREAM("Map saved:\n\t" << mapdatafile << "\n\t" << yamlFile);
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
    // Precompute translation components.
    const double t_x = poseTransform.pose.position.x;
    const double t_y = poseTransform.pose.position.y;
    // Note: The z translation is ignored since we force z to 0.
  
    // Precompute rotation matrix elements from the quaternion.
    tf2::Quaternion q(
      poseTransform.pose.orientation.x,
      poseTransform.pose.orientation.y,
      poseTransform.pose.orientation.z,
      poseTransform.pose.orientation.w
    );
    tf2::Matrix3x3 m(q);
    const double r00 = m[0][0], r01 = m[0][1], r02 = m[0][2];
    const double r10 = m[1][0], r11 = m[1][1], r12 = m[1][2];
  
    // Optionally, for a large number of points, consider using parallelization.
    #pragma omp parallel for
    for (size_t i = 0; i < polygon.polygon.points.size(); ++i) {
      auto &pt = polygon.polygon.points[i];
      // Compute the new x and y coordinates directly.
      double new_x = t_x + r00 * pt.x + r01 * pt.y + r02 * pt.z;
      double new_y = t_y + r10 * pt.x + r11 * pt.y + r12 * pt.z;
  
      pt.x = new_x;
      pt.y = new_y;
      pt.z = 0.0;  // Force z to zero.
    }
  }
  
  // transform Path -> Marker points coord
  void VisualizerNode::transformPoints(const geometry_msgs::PoseStamped& poseTransform, F2CPath& path, visualization_msgs::Marker& marker) {
    const size_t nPoints = path.size();
    
    // Preallocate marker points vector.
    marker.points.clear();
    marker.points.resize(nPoints);
    
    // Precompute translation.
    const double t_x = poseTransform.pose.position.x;
    const double t_y = poseTransform.pose.position.y;
    // Even if poseTransform.pose.position.z exists, we force z to 0.
    
    // Precompute the rotation matrix once.
    tf2::Quaternion q(
      poseTransform.pose.orientation.x,
      poseTransform.pose.orientation.y,
      poseTransform.pose.orientation.z,
      poseTransform.pose.orientation.w
    );
    tf2::Matrix3x3 m(q);
    const double r00 = m[0][0], r01 = m[0][1], r02 = m[0][2];
    const double r10 = m[1][0], r11 = m[1][1], r12 = m[1][2];
    // We ignore the third row as we force z to zero.
  
    // Retrieve the states (assumed to be in a contiguous container like std::vector).
    auto& states = path.getStates();
  
    // Use OpenMP for parallel processing if the dataset is large.
    #pragma omp parallel for
    for (int i = 0; i < static_cast<int>(nPoints); ++i) {
      auto& state = states[i];
      
      // Get original coordinates.
      const double x = state.point.getX();
      const double y = state.point.getY();
      const double z = state.point.getZ();
      
      // Compute transformed coordinates directly.
      const double new_x = t_x + r00 * x + r01 * y + r02 * z;
      const double new_y = t_y + r10 * x + r11 * y + r12 * z;
      const double new_z = 0.0;  // Force z to 0.
      
      // Update the state point.
      state.point.setX(new_x);
      state.point.setY(new_y);
      state.point.setZ(new_z);
      
      // Update the marker point at the same index.
      marker.points[i].x = new_x;
      marker.points[i].y = new_y;
      marker.points[i].z = new_z;
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

  bool VisualizerNode::parseGeoJsonPose(const std::string& geojson_file,
                                        geometry_msgs::PoseStamped& pose_msg)
  {
    // Open the file
    std::ifstream file(geojson_file);
    if (!file.is_open()) {
      ROS_ERROR("Failed to open GeoJSON file: %s", geojson_file.c_str());
      return false;
    }

    // Read the file contents into a JSON object
    json geojson;
    try {
      file >> geojson;
    } catch (const json::parse_error& e) {
      ROS_ERROR("Error parsing JSON file: %s", e.what());
      return false;
    }

    // Check if "features" array exists and has at least one entry
    if (!geojson.contains("features") || !geojson["features"].is_array() || geojson["features"].empty()) {
      ROS_ERROR("GeoJSON does not contain 'features' array or it is empty.");
      return false;
    }

    // Access the first feature
    json feature = geojson["features"][0];

    // Ensure "properties" exist
    if (!feature.contains("properties")) {
      ROS_ERROR("Missing 'properties' in GeoJSON feature.");
      return false;
    }

    json properties = feature["properties"];

    // Ensure "Position" and "Orientation" exist in properties
    if (!properties.contains("Position") || !properties.contains("Orientation")) {
      ROS_ERROR("Missing 'Position' or 'Orientation' inside 'properties'.");
      return false;
    }

    try {
      // Extract Position from properties
      pose_msg.pose.position.x = properties["Position"].value("x", 0.0);
      pose_msg.pose.position.y = properties["Position"].value("y", 0.0);
      pose_msg.pose.position.z = properties["Position"].value("z", 0.0);

      // Extract Orientation from properties
      pose_msg.pose.orientation.w = properties["Orientation"].value("w", 1.0);
      pose_msg.pose.orientation.x = properties["Orientation"].value("x", 0.0);
      pose_msg.pose.orientation.y = properties["Orientation"].value("y", 0.0);
      pose_msg.pose.orientation.z = properties["Orientation"].value("z", 0.0);

      ROS_INFO_STREAM("Successfully extracted Pose from GeoJSON: ");
      ROS_INFO_STREAM("  position.x = " << pose_msg.pose.position.x);
      ROS_INFO_STREAM("  position.y = " << pose_msg.pose.position.y);
      ROS_INFO_STREAM("  position.z = " << pose_msg.pose.position.z);
      ROS_INFO_STREAM("  orientation.x = " << pose_msg.pose.orientation.x);
      ROS_INFO_STREAM("  orientation.y = " << pose_msg.pose.orientation.y);
      ROS_INFO_STREAM("  orientation.z = " << pose_msg.pose.orientation.z);
      ROS_INFO_STREAM("  orientation.w = " << pose_msg.pose.orientation.w);

      // Set timestamp and frame_id
      pose_msg.header.stamp = ros::Time::now();
      pose_msg.header.frame_id = frame_id_;
      return true;
    } catch (const json::exception& e) {
      ROS_ERROR("Error extracting data from JSON: %s", e.what());
      return false;
    }
  }
}