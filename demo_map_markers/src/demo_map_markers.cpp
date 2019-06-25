#include "demo_map_markers/demo_map_markers.h"
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <math.h>

DemoMapMarkerStreamer::DemoMapMarkerStreamer()
{
  ros::NodeHandle nh;
  marker_pub_ =
      nh.advertise<visualization_msgs::Marker>("demo_map_markers", 1000);
  mcpic_upload_sub_ = nh.subscribe(
      "/mcpic_update", 10, &DemoMapMarkerStreamer::updateCallback, this);

  delete_markers_ss_ = nh.advertiseService(
      "delete_demo_markers", &DemoMapMarkerStreamer::deleteDemoMarkers, this);
}

bool DemoMapMarkerStreamer::deleteDemoMarkers(std_srvs::Empty::Request& req,
                                              std_srvs::Empty::Response& res)
{
  visualization_msgs::Marker DELETE_MARKER;
  DELETE_MARKER.ns = "demo_markers";
  DELETE_MARKER.action = visualization_msgs::Marker::DELETE;
  for (int i = 0; i < id_ + 1; i++)
  {
    DELETE_MARKER.id = i;
    marker_pub_.publish(DELETE_MARKER);
  }
  id_ = 0;
  return true;
}

bool DemoMapMarkerStreamer::init()
{
  ros::NodeHandle nh, pnh("~");
  pnh.param<double>("marker_scale", marker_scale_, 1.0);
  pnh.param<double>("marker_thickness", marker_thickness_, 1.0);
  std::vector<double> x_coords, y_coords;
  if (!pnh.getParam("x_coordinates", x_coords) ||
      !pnh.getParam("y_coordinates", y_coords))
  {
    ROS_ERROR("Could not find either x_coordinates or y_coordinates parameter "
              "on parameter server. Both must be present.");
    return false;
  }
  if (x_coords.size() != y_coords.size())
  {
    ROS_ERROR_STREAM(
        "Incompatible number of x and y coordinates. Num of x coords: "
        << x_coords.size() << " || Num of y coords: " << y_coords.size());
    return false;
  }

  for (size_t i = 0; i < x_coords.size(); i++)
  {
    coords_.push_back(std::pair<double, double>{x_coords[i], y_coords[i]});
  }

  ros::Duration sleep_dur{1.0};
  while (marker_pub_.getNumSubscribers() == 0 && ros::ok())
  {
    ROS_INFO_STREAM("Waiting for valid subscriber...");
    sleep_dur.sleep();
  }

  std::vector<double> tmp;
  std::pair<double, double> home_coords;
  if (!pnh.getParam("home_coordinates", tmp))
  {
    ROS_WARN_STREAM("No home_coordinates parameter found on parameter server. "
                    "Home marker will not be published.");
  }
  else if (tmp.size() != 2)
  {
    ROS_WARN_STREAM("Invalid size for home_coordinates parameter. Should just "
                    "be a single x,y coord pair (2 values). Home marker will "
                    "not be published.");
  }
  else
  {
    home_coords.first = tmp[0];
    home_coords.second = tmp[1];
    publishHomeMarkers(home_coords);
  }
  return true;
}

void DemoMapMarkerStreamer::publishHomeMarkers(
    const std::pair<double, double>& home_coords) const
{
  visualization_msgs::Marker home_text_marker, home_shape_marker;

  home_text_marker.action = visualization_msgs::Marker::ADD;
  home_text_marker.header.frame_id = "map";
  home_text_marker.header.stamp = ros::Time::now();
  home_text_marker.ns = "home_markers";
  home_text_marker.id = 0;
  home_text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  home_text_marker.text = "We are here.";
  home_text_marker.pose.position.x = home_coords.first;
  home_text_marker.pose.position.y = home_coords.second + 2.0;
  //  home_text_marker.pose.position.z = 5.0;
  home_text_marker.pose.orientation.w = 1.0;
  home_text_marker.scale.z = 1.0;
  home_text_marker.color.b = 1.0;
  home_text_marker.color.a = 1.0;

  home_shape_marker.action = visualization_msgs::Marker::ADD;
  home_shape_marker.header.frame_id = "map";
  home_shape_marker.header.stamp = ros::Time::now();
  home_shape_marker.ns = "home_markers";
  home_shape_marker.id = 1;
  home_shape_marker.type = visualization_msgs::Marker::CYLINDER;
  home_shape_marker.pose.position.x = home_coords.first;
  home_shape_marker.pose.position.y = home_coords.second;
  home_shape_marker.pose.orientation.w = 1.0;
  home_shape_marker.scale.x = 1.0;
  home_shape_marker.scale.y = 1.0;
  home_shape_marker.scale.z = 0.1;
  home_shape_marker.color.b = 1.0;
  home_shape_marker.color.a = 1.0;

  marker_pub_.publish(home_text_marker);
  marker_pub_.publish(home_shape_marker);

  ROS_WARN_STREAM("Published HOME.");
}

void DemoMapMarkerStreamer::updateCallback(
    const demo_state_machine::MCPICUpdate::ConstPtr& msg)
{
  visualization_msgs::Marker marker;
  if (msg->rfid_bad)
  {
    genXMarker(marker);
    ROS_WARN_STREAM("Publishing X.");
  }
  else
  {
    genCheckMarker(marker);
    ROS_WARN_STREAM("Publishing CHECK.");
  }
  id_++;
  marker_pub_.publish(marker);
}

void DemoMapMarkerStreamer::genXMarker(
    visualization_msgs::Marker& x_marker) const
{
  x_marker.action = visualization_msgs::Marker::ADD;
  x_marker.header.frame_id = "map";
  x_marker.header.stamp = ros::Time::now();
  x_marker.ns = "demo_markers";
  x_marker.id = id_;
  x_marker.type = visualization_msgs::Marker::LINE_LIST;
  x_marker.pose.orientation.w = 1.0;
  x_marker.scale.x = marker_thickness_;
  x_marker.color.r = 1.0;
  x_marker.color.a = 1.0;
  auto x_points = drawX(coords_[id_]);
  x_marker.points = x_points;
}

void DemoMapMarkerStreamer::genCheckMarker(
    visualization_msgs::Marker& check_marker) const
{
  check_marker.action = visualization_msgs::Marker::ADD;
  check_marker.header.frame_id = "map";
  check_marker.header.stamp = ros::Time::now();
  check_marker.ns = "demo_markers";
  check_marker.id = id_;
  check_marker.type = visualization_msgs::Marker::LINE_LIST;
  check_marker.pose.orientation.w = 1.0;
  check_marker.scale.x = marker_thickness_;
  check_marker.color.g = 1.0;
  check_marker.color.a = 1.0;
  auto check_points = drawCheck(coords_[id_]);
  check_marker.points = check_points;
}

const std::vector<geometry_msgs::Point> DemoMapMarkerStreamer::drawCheck(
    const std::pair<double, double>& coord_pair) const
{
  double x, y;
  x = coord_pair.first;
  y = coord_pair.second;
  geometry_msgs::Point p1, p2, p3, p4;
  p1.z = p2.z = p3.z = p4.z = 0.0;
  p1.x = x;
  p1.y = y;
  p2.x = x - 1.0;
  p2.y = y + 1.0;
  p3.x = x - 0.5;
  p3.y = y - 0.5;
  p4.x = x + 1.5;
  p4.y = y + 1.5;
  std::vector<geometry_msgs::Point> x_points{p1, p2, p3, p4};
  return x_points;
}

const std::vector<geometry_msgs::Point>
DemoMapMarkerStreamer::drawX(const std::pair<double, double>& coord_pair) const
{
  double x, y;
  x = coord_pair.first;
  y = coord_pair.second;
  geometry_msgs::Point p1, p2, p3, p4;
  p1.z = p2.z = p3.z = p4.z = 0.0;
  p1.x = x - 1.0 * marker_scale_;
  p1.y = y + 1.0 * marker_scale_;
  p2.x = x + 1.0 * marker_scale_;
  p2.y = y - 1.0 * marker_scale_;
  p3.x = x - 1.0 * marker_scale_;
  p3.y = y - 1.0 * marker_scale_;
  p4.x = x + 1.0 * marker_scale_;
  p4.y = y + 1.0 * marker_scale_;
  std::vector<geometry_msgs::Point> check_points{p1, p2, p3, p4};
  return check_points;
}
