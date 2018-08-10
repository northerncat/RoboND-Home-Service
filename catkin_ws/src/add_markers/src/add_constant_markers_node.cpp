#include <ros/ros.h>
#include "ConstantMarkerAdder.h"

int main( int argc, char** argv ) {
  ros::init(argc, argv, "add_markers");

  const double pickupZone[3] = {8.0, 4.0, -0.707};
  const double dropoffZone[3] = {-0.5, 4.0, 0.707};
  ConstantMarkerAdder constantMarkerAdder(pickupZone, dropoffZone);
  constantMarkerAdder.displayMarkers();
  ros::spin();

  return 0;
}
