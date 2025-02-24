//=============================================================================
//    Copyright (C) 2021-2022 Wageningen University - All Rights Reserved
//                     Author: Gonzalo Mier
//                        BSD-3 License
//=============================================================================

#include <signal.h>
#include <memory>

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include <fields2cover_ros/F2CConfig.h>
#include "Fields2CoverVisualizerNode.h"

// Use a global (or static) pointer so the SIGINT handler can access the node
std::unique_ptr<fields2cover_ros::VisualizerNode> g_visualizer_node;

// Custom SIGINT handler
void sigintHandler(int)
{
  ROS_INFO("CTRL-C pressed. Calling saveMap()...");
  if (g_visualizer_node) {
    // g_visualizer_node->saveMap();
  }
  // Now shutdown ROS
  ros::shutdown();
}

int main(int argc, char** argv)
{
  // Ensure ROS doesn't install its own SIGINT handler
  ros::init(argc, argv, "fields2cover_visualizer_node",
            ros::init_options::NoSigintHandler);

  // Install our custom SIGINT handler
  signal(SIGINT, sigintHandler);

  // Node handle
  ros::NodeHandle nh;

  // Create our node instance
  g_visualizer_node = std::make_unique<fields2cover_ros::VisualizerNode>();
  g_visualizer_node->init_VisualizerNode();

  // Setup dynamic reconfigure server
  dynamic_reconfigure::Server<fields2cover_ros::F2CConfig> server;
  auto callback = boost::bind(
      &fields2cover_ros::VisualizerNode::rqt_callback,
      boost::ref(*g_visualizer_node), _1, _2);
  server.setCallback(callback);

  // Let ROS spin until we receive Ctrl+C (SIGINT)
  ros::spin();

  // If ros::spin() exits normally (e.g., ros::shutdown was called), we return here
  return 0;
}