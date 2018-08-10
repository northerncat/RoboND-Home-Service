/**
 * The MarkerAdder class publishes a marker at the designated pickup zone initially, and subscribes to the odometry
 * topic to determine whether the robot has reached the object or not. When the robot has reached the pickup zone,
 * the MarkerAdder would hide the marker, pause for five seconds, and then wait until the robot reaches the dropoff
 * zone to show the marker in the dropoff zone again.
 */
#include "nav_msgs/Odometry.h"
#include "ConstantMarkerAdder.h"

class MarkerAdder : ConstantMarkerAdder {
public:
  MarkerAdder(const double* pickupZone, const double* dropoffZone);

private:
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

  ros::Subscriber odom_sub;
  bool pickedUp;

  constexpr static double THRESHOLD = 0.5;
};