#include "waypoint_writer/waypoint_writer.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "waypoint_writer");
  WaypointWriter ww;
  try
  {
    ww.init();
  }
  catch (std::exception& e)
  {
    std::cout << e.what() << std::endl;
    return -1;
  }
  ros::spin();
  return 0;
}
