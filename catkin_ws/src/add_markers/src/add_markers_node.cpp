#include <ros/ros.h>
#include "MarkerAdder.h"

int main( int argc, char** argv ) {
  ros::init(argc, argv, "add_markers");

  double pickupZone[3] = {8.0, 4.0, -0.707};
  double dropoffZone[3] = {-0.5, 4.0, 0.707};
  MarkerAdder markerAdder(pickupZone, dropoffZone);
  ros::spin();

  return 0;
}
