#include "waypoint_writer/waypoint_writer.h"

#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <ros/package.h>
#include <iostream>

enum xbox_button
{
  X = 2,
  Y = 3,
  LEFT_BUMPER = 4,
  RIGHT_BUMPER = 5
};

void WaypointWriter::init()
{
  ros::NodeHandle private_nh{"~"};
  bool load_a_priori_waypoints;

  waypoint_marker_pub_ =
      private_nh.advertise<visualization_msgs::Marker>("waypoint_marker", 1000);

  waypoint_file_path_ =
      ros::package::getPath("waypoint_writer") + "/config/waypoints.txt";

  loadAPrioriWaypoints();
  joy_subscriber_ =
      private_nh.subscribe("/joy", 1, &WaypointWriter::joyCallback, this);
  add_waypoint_server_ = private_nh.advertiseService(
      "/add_waypoint", &WaypointWriter::addWaypointSrv, this);
  delete_last_waypoint_server_ = private_nh.advertiseService(
      "/delete_last_waypoint", &WaypointWriter::deleteLastWaypointSrv, this);
  delete_all_waypoints_server_ = private_nh.advertiseService(
      "/delete_all_waypoints", &WaypointWriter::deleteAllWaypointsSrv, this);
  delete_all_new_waypoints_server_ = private_nh.advertiseService(
      "/delete_all_new_waypoints", &WaypointWriter::deleteAllNewWaypointsSrv,
      this);
}

void WaypointWriter::loadAPrioriWaypoints()
{

  ROS_WARN_STREAM("Trying to open/read file at: " << waypoint_file_path_);
  std::ifstream input_waypoint_file_stream;
  input_waypoint_file_stream.exceptions(std::ifstream::badbit);
  try
  {
    input_waypoint_file_stream.open(waypoint_file_path_, std::ifstream::in);
  }
  catch (std::ifstream::failure& e)
  {
    ROS_WARN_STREAM("No waypoints.txt exists to read from.");
    return;
  }

  if (input_waypoint_file_stream.peek() == std::ifstream::traits_type::eof())
  {
    ROS_WARN_STREAM("Detected blank waypoints.txt.");
    return;
  }

  ROS_WARN_STREAM("Found eligible waypoints.txt.");
  while (waypoint_marker_pub_.getNumSubscribers() == 0 && ros::ok())
  {
    ROS_INFO_STREAM("Waiting for valid subscriber...");
    usleep(1e6);
  }

  std::string line;
  ros::Duration d(0.1);

  while (std::getline(input_waypoint_file_stream, line))
  {
    // not super solid. Skips line if it has a # anywhere in line
    if (line.find("#") != std::string::npos)
    {
      continue;
    }
    std::istringstream iss(line);
    std::string token;
    std::vector<std::string> tokens;
    while (std::getline(iss, token, ','))
    {
      tokens.push_back(token);
    }
    geometry_msgs::Pose waypoint_pose;
    waypoint_pose.position.x = std::stof(tokens[0]);
    waypoint_pose.position.y = std::stof(tokens[1]);
    waypoint_pose.position.z = std::stof(tokens[2]);
    waypoint_pose.orientation.x = std::stof(tokens[3]);
    waypoint_pose.orientation.y = std::stof(tokens[4]);
    waypoint_pose.orientation.z = std::stof(tokens[5]);
    waypoint_pose.orientation.w = std::stof(tokens[6]);
    waypoints_.push_back(waypoint_pose);

    ros::spinOnce();
    visualization_msgs::Marker red_marker = WayPointerWriterMarkers::red();
    red_marker.pose = waypoint_pose;
    red_marker.id = waypoint_marker_count_;

    std::stringstream add_stream;
    add_stream << "Publishing new waypoint marker -->";
    ROS_WARN_STREAM(printPose(add_stream, waypoint_pose).str());

    waypoint_marker_pub_.publish(red_marker);
    publishCounterMarker(waypoint_pose);

    d.sleep();
  }
  // Can possibly deal with the incrementing/decrementing cleaner
  a_priori_waypoint_marker_count = waypoint_marker_count_ - 1;
  input_waypoint_file_stream.close();
}

void WaypointWriter::publishCounterMarker(geometry_msgs::Pose& waypoint_pose) {
  visualization_msgs::Marker counter_marker =
      WayPointerWriterMarkers::counter_marker();
  counter_marker.pose = waypoint_pose;
  counter_marker.pose.position.z += 0.1;
  counter_marker.id = waypoint_marker_count_++;
  counter_marker.text = std::to_string(counter_marker.id);
  waypoint_marker_pub_.publish(counter_marker);

}

void WaypointWriter::joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
  if (joy_msg->buttons[X] && joy_msg->buttons[Y] &&
      joy_msg->buttons[LEFT_BUMPER] && joy_msg->buttons[RIGHT_BUMPER])
  {
    deleteAllWaypoints();
  }
  else if (joy_msg->buttons[X] && joy_msg->buttons[Y] &&
           joy_msg->buttons[LEFT_BUMPER])
  {
    deleteAllNewWaypoints();
  }
  else if (joy_msg->buttons[X] && joy_msg->buttons[LEFT_BUMPER])
  {
    deleteLastWaypoint();
  }
  else if (joy_msg->buttons[RIGHT_BUMPER])
  {
    addWaypoint();
  }
}

bool WaypointWriter::addWaypointSrv(std_srvs::Empty::Request& req,
                                    std_srvs::Empty::Response& res)
{
  if (!addWaypoint())
  {
    ROS_ERROR_STREAM("Service call failed to add waypoint.");
  }
  return true;
}

bool WaypointWriter::addWaypoint()
{
  geometry_msgs::TransformStamped waypoint_transform;
  try
  {
    waypoint_transform =
        tf_buffer_.lookupTransform("map", "base_link", ros::Time(0));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_ERROR("%s", ex.what());
    usleep(5e5);
    return false;
  }
  geometry_msgs::Pose waypoint_pose;
  // if we don't set w = 1, there is undefined behavior after transform
  waypoint_pose.orientation.w = 1.0;
  tf2::doTransform(waypoint_pose, waypoint_pose, waypoint_transform);

  visualization_msgs::Marker green_marker = WayPointerWriterMarkers::green();
  green_marker.pose = waypoint_pose;
  green_marker.id = waypoint_marker_count_;

  std::stringstream add_stream;
  add_stream << "Publishing new waypoint marker -->";
  ROS_WARN_STREAM(printPose(add_stream, waypoint_pose).str());

  waypoint_marker_pub_.publish(green_marker);
  publishCounterMarker(waypoint_pose);
  waypoints_.push_back(waypoint_pose);

  usleep(5e5);
  return true;
}

bool WaypointWriter::deleteLastWaypointSrv(std_srvs::Empty::Request& req,
                                           std_srvs::Empty::Response& res)
{
  if (!deleteLastWaypoint())
  {
    ROS_ERROR_STREAM("Service call failed to delete last waypoint.");
  }
  return true;
}

bool WaypointWriter::deleteLastWaypoint()
{
  if (waypoints_.size() == 0)
  {
    ROS_ERROR_STREAM("No waypoints available for deletion.");
    return true;
  }
  else
  {
    geometry_msgs::Pose last_waypoint = waypoints_.back();
    deleteWaypoint(last_waypoint);
  }
  usleep(5e5);
  return true;
}

void WaypointWriter::deleteWaypoint(geometry_msgs::Pose& waypoint)
{
  std::stringstream delete_stream;
  delete_stream << "Deleting waypoint marker-->";
  ROS_WARN_STREAM(printPose(delete_stream, waypoint).str());
  waypoints_.pop_back();
  visualization_msgs::Marker delete_one_marker =
      WayPointerWriterMarkers::delete_one();
  delete_one_marker.id = --waypoint_marker_count_;
  waypoint_marker_pub_.publish(delete_one_marker);
  visualization_msgs::Marker delete_one_counter_marker = WayPointerWriterMarkers::delete_counter_marker();
  delete_one_counter_marker.id = waypoint_marker_count_;
  waypoint_marker_pub_.publish(delete_one_counter_marker);
}

bool WaypointWriter::deleteAllWaypointsSrv(std_srvs::Empty::Request& req,
                                           std_srvs::Empty::Response& res)
{
  if (!deleteAllWaypoints())
  {
    ROS_ERROR_STREAM("Service call failed to delete last waypoint.");
  }
  return true;
}

bool WaypointWriter::deleteAllWaypoints()
{
  ROS_WARN_STREAM("Executing request to delete all current waypoints.");
  waypoints_.clear();
  visualization_msgs::Marker delete_all_marker =
      WayPointerWriterMarkers::delete_all();
  waypoint_marker_pub_.publish(delete_all_marker);
  waypoint_marker_count_ = 0;
  usleep(5e5);
  return true;
}

bool WaypointWriter::deleteAllNewWaypointsSrv(std_srvs::Empty::Request& req,
                                              std_srvs::Empty::Response& res)
{
  if (!deleteAllNewWaypoints())
  {
    ROS_ERROR_STREAM("Service call failed to delete all new waypoints.");
  }
  return true;
}

bool WaypointWriter::deleteAllNewWaypoints()
{
  ROS_WARN_STREAM("Received request to delete all new markers.");
  if (waypoints_.size() == 0)
  {
    ROS_ERROR_STREAM("No waypoints available for deletion.");
    return true;
  }
  else
  {
    // incremeting/decrementing again kind of confusing
    for (int i = waypoint_marker_count_ - 1; i > a_priori_waypoint_marker_count;
         i--)
    {
      deleteWaypoint(waypoints_[i]);
    }
  }
  usleep(5e5);
  return true;
}

WaypointWriter::~WaypointWriter()
{
  std::ofstream waypoint_output_file_stream;
  try
  {
    waypoint_output_file_stream.open(waypoint_file_path_, std::ofstream::trunc);
    for (const auto& waypoint : waypoints_)
    {
      if (!writeToFile(waypoint_output_file_stream, waypoint))
      {
        std::cerr << "Output file stream closed. Could not write requested "
                     "waypoint." << std::endl;
      }
      else
      {
        std::stringstream write_stream;
        write_stream << "Successfully wrote new waypoint to waypoints.txt -->";
        std::cout << printPose(write_stream, waypoint).str() << std::endl;
      }
    }
  }
  catch (std::ofstream::failure& e)
  {
    ROS_ERROR_STREAM("Failure writing selected waypoints to waypoints.txt.");
    return;
  }
}

bool WaypointWriter::writeToFile(std::ofstream& waypoint_output_file_stream,
                                 const geometry_msgs::Pose& waypoint)
{
  if (waypoint_output_file_stream.is_open())
  {
    waypoint_output_file_stream
        << waypoint.position.x << "," << waypoint.position.y << ","
        << waypoint.position.z << "," << waypoint.orientation.x << ","
        << waypoint.orientation.y << "," << waypoint.orientation.z << ","
        << waypoint.orientation.w << ",map\n";
    waypoint_output_file_stream.flush();
  }
  else
  {
    return false;
  }
  return true;
}

std::stringstream& WaypointWriter::printPose(std::stringstream& stream,
                                             const geometry_msgs::Pose& pose)
{
  stream << " X: " << pose.position.x << " Y: " << pose.position.y
         << " Z: " << pose.position.z << " Qx: " << pose.orientation.x
         << " Qy: " << pose.orientation.y << " Qz: " << pose.orientation.z
         << " Qw: " << pose.orientation.w << " framd_id: map";
  return stream;
}
