#ifndef WAYPOINT_WRITER_H
#define WAYPOINT_WRITER_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Joy.h>
#include <std_srvs/Empty.h>
#include <tf2_ros/transform_listener.h>
#include <fstream>
#include <visualization_msgs/Marker.h>

class WaypointWriter
{
private:
  void loadAPrioriWaypoints();
  bool writeToFile(std::ofstream& waypoint_output_file_stream,
                   const geometry_msgs::Pose& waypoint);
  ros::Publisher waypoint_marker_pub_;
  int waypoint_marker_count_{0}, a_priori_waypoint_marker_count{0};

  ros::Subscriber joy_subscriber_;
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg);

  ros::ServiceServer add_waypoint_server_;
  ros::ServiceServer delete_last_waypoint_server_;
  ros::ServiceServer delete_all_waypoints_server_;
  ros::ServiceServer delete_all_new_waypoints_server_;

  bool addWaypoint();
  void deleteWaypoint(geometry_msgs::Pose& waypoint);
  bool deleteLastWaypoint();
  bool deleteAllWaypoints();
  bool deleteAllNewWaypoints();

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_{tf_buffer_};
  std::vector<geometry_msgs::Pose> waypoints_;
  std::string waypoint_file_path_;

  std::stringstream& printPose(std::stringstream& stream,
                               const geometry_msgs::Pose& pose);

  void publishCounterMarker(geometry_msgs::Pose& waypoint_pose);

public:
  void init();
  bool addWaypointSrv(std_srvs::Empty::Request& req,
                      std_srvs::Empty::Response& res);
  bool deleteLastWaypointSrv(std_srvs::Empty::Request& req,
                             std_srvs::Empty::Response& res);
  bool deleteAllWaypointsSrv(std_srvs::Empty::Request& req,
                             std_srvs::Empty::Response& res);
  bool deleteAllNewWaypointsSrv(std_srvs::Empty::Request& req,
                                std_srvs::Empty::Response& res);

  ~WaypointWriter();
};

struct WayPointerWriterMarkers
{

  static visualization_msgs::Marker default_marker()
  {
    visualization_msgs::Marker DEFAULT;
    DEFAULT.header.frame_id = "map";
    DEFAULT.header.stamp = ros::Time::now();
    DEFAULT.ns = "waypoints";
    DEFAULT.type = visualization_msgs::Marker::ARROW;
    DEFAULT.scale.x = 0.4;
    DEFAULT.scale.y = 0.1;
    DEFAULT.scale.z = 0.1;
    DEFAULT.color.a = 1.0;
    return DEFAULT;
  }

  static visualization_msgs::Marker red()
  {
    visualization_msgs::Marker RED_MARKER = default_marker();
    RED_MARKER.action = visualization_msgs::Marker::ADD;
    RED_MARKER.color.r = 1.0;
    return RED_MARKER;
  }
  static visualization_msgs::Marker green()
  {
    visualization_msgs::Marker GREEN_MARKER = default_marker();
    GREEN_MARKER.action = visualization_msgs::Marker::ADD;
    GREEN_MARKER.color.g = 1.0;
    return GREEN_MARKER;
  }
  static visualization_msgs::Marker delete_one()
  {
    visualization_msgs::Marker DELETE_ONE_MARKER;
    DELETE_ONE_MARKER.ns = "waypoints";
    DELETE_ONE_MARKER.action = visualization_msgs::Marker::DELETE;
    return DELETE_ONE_MARKER;
  }

  static visualization_msgs::Marker delete_all()
  {
    visualization_msgs::Marker DELETE_ALL_MARKER;
    DELETE_ALL_MARKER.action = visualization_msgs::Marker::DELETEALL;
    return DELETE_ALL_MARKER;
  }

  static visualization_msgs::Marker counter_marker()
  {
    visualization_msgs::Marker COUNTER;
    COUNTER.header.frame_id = "map";
    COUNTER.header.stamp = ros::Time::now();
    COUNTER.ns = "counters";
    COUNTER.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    COUNTER.scale.z = 0.25;
    COUNTER.color.r = 1.0;
    COUNTER.color.g = 1.0;
    COUNTER.color.b = 1.0;
    COUNTER.color.a = 1.0;
    return COUNTER;
  }

  static visualization_msgs::Marker delete_counter_marker() {
    visualization_msgs::Marker DELETE_COUNTER;
    DELETE_COUNTER.ns = "counters";
    DELETE_COUNTER.action = visualization_msgs::Marker::DELETE;
    return DELETE_COUNTER;
  }
};

#endif
