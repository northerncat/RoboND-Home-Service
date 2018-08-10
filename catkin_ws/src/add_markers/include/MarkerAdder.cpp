#include "MarkerAdder.h"

#include <cmath>
#include <sstream>

// Constructor
MarkerAdder::MarkerAdder(const double* pickupZone, const double* dropoffZone) : pickedUp(false), ConstantMarkerAdder(pickupZone, dropoffZone) {
  // assign constants
  odom_sub = n.subscribe("/odom", 1, &MarkerAdder::odomCallback, this);

  setMarker(pickupZone[0], pickupZone[1], pickupZone[2]);
  showMarker(true);
}

// The odometry callback checks whether the robot is reaching the pickup zone or the dropoff zone, and
// show/hide the marker accordingly.
void MarkerAdder::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  double robotX = msg->pose.pose.position.x;
  double robotY = msg->pose.pose.position.y;
  double pickupDistance = sqrt(pow(robotX - pickupZone[0], 2.0) + pow(robotY - pickupZone[1], 2.0));
  if (!pickedUp) {
    std::ostringstream pickupSs;
    pickupSs << "pickup distance " << pickupDistance;
    ROS_INFO(pickupSs.str().c_str());
    if (pickupDistance < THRESHOLD) {
      showMarker(false);
      ros::Duration(SLEEP_TIME).sleep();
      pickedUp = true;
    }
  }

  double dropoffDistance = sqrt(pow(robotX - dropoffZone[0], 2.0) + pow(robotY - dropoffZone[1], 2.0));
  if (pickedUp) {
    std::ostringstream dropoffSs;
    dropoffSs << "dropoff distance " << dropoffDistance;
    ROS_INFO(dropoffSs.str().c_str());
    if (dropoffDistance < THRESHOLD) {
      setMarker(dropoffZone[0], dropoffZone[1], dropoffZone[2]);
      showMarker(true);
    }
  }
}