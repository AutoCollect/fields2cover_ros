//=============================================================================
//    Copyright (C) 2021-2022 Wageningen University - All Rights Reserved
//                     Author: Gonzalo Mier
//                        BSD-3 License
//=============================================================================


#include "Fields2CoverVisualizerNode.h"
#include "ros/conversor.h"
#include <tf2/LinearMath/Quaternion.h>

#include <fstream>
#include <iostream>
#include <cmath>
#include <algorithm>

#include <dynamic_reconfigure/server.h>
#include <fields2cover_ros/F2CConfig.h>

#include <geometry_msgs/Point32.h>
using namespace std;


namespace fields2cover_ros {
    
    void VisualizerNode::init_VisualizerNode() {

      //==============================================================
      field_polygon_publisher_         = public_node_handle_.advertise<geometry_msgs::PolygonStamped>("/field/border",       1, true);
      field_no_headlands_publisher_    = public_node_handle_.advertise<geometry_msgs::PolygonStamped>("/field/no_headlands", 1, true);
      field_gps_publisher_             = public_node_handle_.advertise<sensor_msgs::NavSatFix>       ("/gps/fix",            1, true);
      field_swaths_publisher_          = public_node_handle_.advertise<visualization_msgs::Marker>   ("/field/swaths",       1, true);
      field_swaths_pt_publisher_       = public_node_handle_.advertise<visualization_msgs::Marker>   ("/field/swaths_pt",    1, true);
      field_swath_start_publisher_     = public_node_handle_.advertise<visualization_msgs::Marker>   ("/field/swaths_start", 1, true);
      field_swath_end_publisher_       = public_node_handle_.advertise<visualization_msgs::Marker>   ("/field/swaths_end",   1, true);
      path_start_publisher_            = public_node_handle_.advertise<visualization_msgs::Marker>   ("/field/path_start",   1, true);
      //==============================================================
      polygon_file_ = "/home/patrick/u_turn_ws/data/metalform_carpark.txt";
      std::ifstream polygon_file(polygon_file_);
      double position_x, position_y, position_z;
      polygon_.header.frame_id = "map";
      //==============================================================
      f2c::types::LinearRing border;
      f2c::types::Field field;
      f2c::types::Cell cell;
      //==============================================================

      while (polygon_file >> position_x >> position_y >> position_z)
      {
        // std::cout << position_x << " " 
        //           << position_y << " " 
        //           << position_z << std::endl;

        geometry_msgs::Point32 pt;
        pt.x = position_x;
        pt.y = position_y;
        pt.z = position_z;
        polygon_.polygon.points.push_back(pt);

        //==============================================================
        border.addPoint(position_x, position_y);

      }
      //==============================================================
      border.addPoint(border.StartPoint());
      cell.addRing(border);
      field.field.addGeometry(cell);
      area_ = field.field;
      // F2CRobot robot (2.0, 500.0);
    }

    double VisualizerNode::normalize_angle(double angle) {
      const double result = fmod(angle + M_PI, 2.0 * M_PI);
      if(result <= 0.0) return result + M_PI;
        return result - M_PI;
    }

    // simple linear interpolation
    void VisualizerNode::interpolation(const double start_x, const double start_y, 
                                       const double end_x,   const double end_y,
                                       const double step,
                                       std::vector<double>& interp_x, std::vector<double>& interp_y) {

      double new_x = std::min(start_x, end_x) + step;
      double max_x = std::max(start_x, end_x);

      bool reverse = false;

      if (std::min(start_x, end_x) == end_x) {
        reverse = true;
      }

      while(new_x < max_x) {
        /* Linear Interpolation */
        //  yp = y0 + ((y1-y0) / (x1-x0)) * (xp - x0);
        double new_y = start_y + ((end_y - start_y) / (end_x - start_x)) * (new_x - start_x);
        interp_x.push_back(new_x);
        interp_y.push_back(new_y);
        new_x += step;
      }

      if (reverse) {
        std::reverse(interp_x.begin(), interp_x.end());
        std::reverse(interp_y.begin(), interp_y.end());
      }

      ROS_INFO("interp size: %d", int(interp_x.size()));
    }

    void VisualizerNode::publish_topics(void) {

      // auto gps = transf_.getRefPointInGPS(fields_[0]);
      // gps_.longitude = gps.getX();
      // gps_.latitude = gps.getY();
      // gps_.altitude = gps.getZ();
      // gps_.header.stamp = ros::Time::now();
      // gps_.header.frame_id = "base_link";
      // field_gps_publisher_.publish(gps_);
      // auto f = fields_[0].field.clone();
      // f2c::hg::ConstHL hl_gen_;
      // F2CCell no_headlands = hl_gen_.generateHeadlands(f, optim_.headland_width).getGeometry(0);

      // geometry_msgs::PolygonStamped polygon_st;

      // polygon_st.header.stamp = ros::Time::now();
      // polygon_st.header.frame_id = "map";
      // conversor::ROS::to(f.getCellBorder(0), polygon_st.polygon);
      //==============================================================
      polygon_.header.stamp = ros::Time::now();
      field_polygon_publisher_.publish(polygon_);
      // polygon_st.polygon.points.clear();
      //==============================================================
      f2c::hg::ConstHL const_hl;
      // F2CCells no_headlands = const_hl.generateHeadlands(area_, optim_.headland_width);
      // conversor::ROS::to(no_headlands.getCellBorder(0), polygon_headland_.polygon);

      F2CCell no_headlands = const_hl.generateHeadlands(area_, optim_.headland_width).getGeometry(0);
      conversor::ROS::to(no_headlands.getGeometry(0), polygon_headland_.polygon);
      polygon_headland_.header.frame_id = "map";
      polygon_headland_.header.stamp = ros::Time::now();
      field_no_headlands_publisher_.publish(polygon_headland_);
      polygon_headland_.polygon.points.clear();
      //==============================================================

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
        swaths = swath_gen_.generateSwaths(optim_.best_angle,
            robot_.op_width, no_headlands);
      }

      //==============================================================
      visualization_msgs::Marker marker_swaths_start;
      marker_swaths_start.header.frame_id    = "map";
      marker_swaths_start.header.stamp       = ros::Time::now();
      marker_swaths_start.action             = visualization_msgs::Marker::ADD;
      marker_swaths_start.pose.orientation.w = 1.0;
      marker_swaths_start.type    = visualization_msgs::Marker::POINTS;
      marker_swaths_start.scale.x = 0.1;
      marker_swaths_start.scale.y = 0.1;
      marker_swaths_start.scale.z = 0.1;
      marker_swaths_start.color.g = 1.0;
      marker_swaths_start.color.a = 1.0;

      geometry_msgs::Point ros_swaths_start_point;
      for (auto swath : swaths) {
	      conversor::ROS::to(swath.startPoint(), ros_swaths_start_point);
        marker_swaths_start.points.push_back(ros_swaths_start_point);
      }
      //==============================================================
      visualization_msgs::Marker marker_swaths_end;
      marker_swaths_end.header.frame_id    = "map";
      marker_swaths_end.header.stamp       = marker_swaths_start.header.stamp;
      marker_swaths_end.action             = visualization_msgs::Marker::ADD;
      marker_swaths_end.pose.orientation.w = 1.0;
      marker_swaths_end.type    = visualization_msgs::Marker::POINTS;
      marker_swaths_end.scale.x = 0.1;
      marker_swaths_end.scale.y = 0.1;
      marker_swaths_end.scale.z = 0.1;
      marker_swaths_end.color.g = 1.0;
      marker_swaths_end.color.r = 1.0;
      marker_swaths_end.color.a = 1.0;

      geometry_msgs::Point ros_swaths_end_point;
      for (auto swath : swaths) {
	      conversor::ROS::to(swath.endPoint(), ros_swaths_end_point);
        marker_swaths_end.points.push_back(ros_swaths_end_point);
        // ROS_INFO("points: %d", int(swath.getNumPoints()));
      }
      //==============================================================

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

      //========================================================
      // for (auto&& s : path.states) {
      //   double normalized_angle = normalize_angle(s.angle);
      //   std::cout << s.point.getX()   << " "
      //             << s.point.getY()   << " "
      //             << s.point.getZ()   << " "
      //             << normalized_angle << std::endl;
      // }
      //========================================================
      std::string path_file_name = "/home/patrick/u_turn_ws/data/path_sample_" + std::to_string(path_file_seq_++) + ".txt";
      path_file_.open(path_file_name);
      //========================================================
      visualization_msgs::Marker marker_swaths_pt;
      marker_swaths_pt.header.frame_id = "map";
      marker_swaths_pt.header.stamp = marker_swaths_start.header.stamp;
      marker_swaths_pt.action  = visualization_msgs::Marker::ADD;
      marker_swaths_pt.pose.orientation.w = 1.0;
      marker_swaths_pt.type    = visualization_msgs::Marker::POINTS;
      marker_swaths_pt.scale.x = 0.05;
      marker_swaths_pt.scale.y = 0.05;
      marker_swaths_pt.scale.z = 0.05;
      marker_swaths_pt.color.r = 1.0;
      marker_swaths_pt.color.a = 1.0;
      //========================================================
      int pt_counter = 0;
      int gap_counter = 0;
      double gap_thresh_hold = 1.0;
      double interp_step = 0.01;

      double pre_x = 0.0;
      double pre_y = 0.0;
      double pre_z = 0.0;

      double pre_quat_x = 0.0;
      double pre_quat_y = 0.0;
      double pre_quat_z = 0.0;
      double pre_quat_w = 0.0;

      for (auto&& s : path.states) {
        double normalized_angle = normalize_angle(s.angle);
        tf2::Quaternion quat; // Create this quaternion from roll/pitch/yaw (in radians)
        quat.setRPY(0, 0, normalized_angle);
        //========================================================
        if (pt_counter > 0) {
          // calculate distace
          double dx = pre_x - double(s.point.getX());
          double dy = pre_y - double(s.point.getY());
          double dist = std::hypot(dx, dy);
          // ROS_INFO("dx = %f, dy = %f, dist = %f", dx, dy, dist);
          if (dist > gap_thresh_hold) {
            //========================================================
            // point linear interpolation
            std::vector<double> interp_x;
            std::vector<double> interp_y;
            interpolation(pre_x, pre_y, s.point.getX(), s.point.getY(), interp_step, interp_x, interp_y);
            //========================================================
            gap_counter ++;
            //========================================================
            if (!interp_x.empty() && !interp_y.empty() && interp_x.size() > 0 && interp_y.size() > 0) {
              for (int i = 0; i< interp_x.size(); i++) {
                
                geometry_msgs::Point pt;
                pt.x = interp_x[i];
                pt.y = interp_y[i];
                pt.z = pre_z;
                marker_swaths_pt.points.push_back(pt);

                // write to file
                path_file_ << interp_x[i]   << " "
                           << interp_y[i]   << " "
                           << pre_z         << " "
                           << pre_quat_x    << " "
                           << pre_quat_y    << " "
                           << pre_quat_z    << " "
                           << pre_quat_w    << std::endl;
              }
            }
          }
        }
        //========================================================
        // otherwise update the previous points
        pt_counter ++;

        pre_x = s.point.getX();
        pre_y = s.point.getY();
        pre_z = s.point.getZ();

        pre_quat_x = quat.getX();
        pre_quat_y = quat.getY();
        pre_quat_z = quat.getZ();
        pre_quat_w = quat.getW();

        //========================================================
        geometry_msgs::Point pt;
        conversor::ROS::to(s.point, pt);
        marker_swaths_pt.points.push_back(pt);
        //========================================================
        // write to file
        path_file_ << s.point.getX()   << " "
                   << s.point.getY()   << " "
                   << s.point.getZ()   << " "
                   << quat.getX()      << " "
                   << quat.getY()      << " "
                   << quat.getZ()      << " "
                   << quat.getW()      << std::endl;
        //========================================================
      }
      path_file_.close();
      ROS_INFO("gap_counter = %d", gap_counter);
      ROS_INFO("Gen Path DONE");
      //========================================================
      //========================================================
      // visualization part
      visualization_msgs::Marker marker_swaths;
      marker_swaths.header.frame_id = "map";
      marker_swaths.header.stamp = marker_swaths_start.header.stamp;
      marker_swaths.action = visualization_msgs::Marker::ADD;
      marker_swaths.pose.orientation.w = 1.0;
      marker_swaths.type    = visualization_msgs::Marker::LINE_STRIP;
      marker_swaths.scale.x = 0.05;
      marker_swaths.scale.y = 0.05;
      marker_swaths.scale.z = 0.05;
      marker_swaths.color.b = 1.0;
      marker_swaths.color.a = 1.0;
      //========================================================
      // visualization_msgs::Marker marker_swaths_pt;
      // marker_swaths_pt.header.frame_id = "map";
      // marker_swaths_pt.header.stamp = marker_swaths.header.stamp;
      // marker_swaths_pt.action  = visualization_msgs::Marker::ADD;
      // marker_swaths_pt.pose.orientation.w = 1.0;
      // marker_swaths_pt.type    = visualization_msgs::Marker::POINTS;
      // marker_swaths_pt.scale.x = 0.05;
      // marker_swaths_pt.scale.y = 0.05;
      // marker_swaths_pt.scale.z = 0.05;
      // marker_swaths_pt.color.r = 1.0;
      // marker_swaths_pt.color.a = 1.0;
      //========================================================
      bool init_point = false;
      visualization_msgs::Marker marker_path_start;
      marker_path_start.header.frame_id = "map";
      marker_path_start.header.stamp = marker_swaths.header.stamp;
      marker_path_start.action  = visualization_msgs::Marker::ADD;
      marker_path_start.pose.orientation.w = 1.0;
      marker_path_start.type    = visualization_msgs::Marker::POINTS;
      marker_path_start.scale.x = 0.2;
      marker_path_start.scale.y = 0.2;
      marker_path_start.scale.z = 0.2;
      marker_path_start.color.b = 1.0;
      marker_path_start.color.a = 1.0;
      //========================================================
      geometry_msgs::Point ros_p;
      for (auto&& s : path.states) {
	      conversor::ROS::to(s.point, ros_p);

        if (!init_point) {
          marker_path_start.points.push_back(ros_p);
          init_point = true;
        }

        marker_swaths.points.push_back(ros_p);
        // marker_swaths_pt.points.push_back(ros_p);
      }

      field_swaths_publisher_     .publish(marker_swaths);
      field_swaths_pt_publisher_  .publish(marker_swaths_pt);
      field_swath_start_publisher_.publish(marker_swaths_start);
      field_swath_end_publisher_  .publish(marker_swaths_end);
      path_start_publisher_       .publish(marker_path_start);
    }

    void VisualizerNode::rqt_callback(fields2cover_ros::F2CConfig &config, uint32_t level) {
      robot_.op_width       = config.op_width;
      robot_.setMinRadius(config.turn_radius);
      optim_.best_angle     = config.swath_angle;
      optim_.headland_width = config.headland_width;
      automatic_angle_      = config.automatic_angle;
      sg_objective_         = config.sg_objective;
      opt_turn_type_        = config.turn_type;
      opt_route_type_       = config.route_type;
      publish_topics();
    }
}


