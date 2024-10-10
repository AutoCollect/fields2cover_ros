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

using namespace std;


namespace fields2cover_ros {
    
  void VisualizerNode::init_VisualizerNode() {
    field_polygon_publisher_      = public_node_handle_.advertise<geometry_msgs::PolygonStamped>("/field/border", 1, true);
    field_no_headlands_publisher_ = public_node_handle_.advertise<geometry_msgs::PolygonStamped>("/field/no_headlands", 1, true);
    field_gps_publisher_          = public_node_handle_.advertise<sensor_msgs::NavSatFix>       ("/gps/fix", 1, true);

    std::string field_file;
    private_node_handle_.getParam("field_file", field_file);

    f2c::Parser::importGml(field_file, fields_);
    f2c::Transform::transform(fields_[0], "EPSG:27200");

    robot_.cruise_speed = 2.0;
    robot_.setMinRadius(2.0);
    double headland_width = 3.0 * robot_.op_width;
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
    f2c::hg::ConstHL hl_gen_;
    F2CCell no_headlands = hl_gen_.generateHeadlands(f, optim_.headland_width).getGeometry(0);

    geometry_msgs::PolygonStamped polygon_st;
    polygon_st.header.stamp = ros::Time::now();
    polygon_st.header.frame_id = "map";
    conversor::ROS::to(f.getCellBorder(0), polygon_st.polygon);
    field_polygon_publisher_.publish(polygon_st);
    polygon_st.polygon.points.clear();
    
    geometry_msgs::PolygonStamped polygon_st2;
    polygon_st2.header.stamp = ros::Time::now();
    polygon_st2.header.frame_id = "map";
    conversor::ROS::to(no_headlands.getGeometry(0), polygon_st2.polygon);
    field_no_headlands_publisher_.publish(polygon_st2);
    polygon_st2.polygon.points.clear();

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


