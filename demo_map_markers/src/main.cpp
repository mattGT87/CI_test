#include "demo_map_markers/demo_map_markers.h"

int main(int argc, char** argv)
{

  ros::init(argc, argv, "demo_map_markers");

  DemoMapMarkerStreamer dmms;

  if (!dmms.init())
  {
    ros::shutdown();
  }
  else
  {
    ros::spin();
  }
}
