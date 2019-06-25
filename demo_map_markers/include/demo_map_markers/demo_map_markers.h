#ifndef DEMO_MAP_MARKERS_H
#define DEMO_MAP_MARKERS_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_srvs/Empty.h>
#include "demo_state_machine/MCPICUpdate.h"


class DemoMapMarkerStreamer {

public:
  DemoMapMarkerStreamer();
  bool init();
  bool deleteDemoMarkers(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

private:
  std::vector<std::pair<double,double>> coords_;
  ros::Subscriber mcpic_upload_sub_;
  ros::Publisher marker_pub_;
  void updateCallback(const demo_state_machine::MCPICUpdate::ConstPtr& msg);
  void publishHomeMarkers(const std::pair<double,double>& home_coords) const;
  void genXMarker(visualization_msgs::Marker& x_marker) const;
  void genCheckMarker(visualization_msgs::Marker& check_marker) const;
  const std::vector<geometry_msgs::Point> drawX(const std::pair<double,double>& coord_pair) const;
  const std::vector<geometry_msgs::Point> drawCheck(const std::pair<double,double>& coord_pair) const;

  ros::ServiceServer delete_markers_ss_;

  int id_{0};
  double marker_scale_{1.0}, marker_thickness_{1.0};

};


#endif
