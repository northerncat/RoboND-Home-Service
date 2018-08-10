#include "ConstantMarkerAdder.h"

#include <cmath>

// Constructor
ConstantMarkerAdder::ConstantMarkerAdder(const double* pickupZone, const double* dropoffZone) : pickupZone(pickupZone), dropoffZone(dropoffZone) {
  // assign constants
  MARKER_COLOR[0] = 0.0;
  MARKER_COLOR[1] = 0.0;
  MARKER_COLOR[2] = 1.0;
  MARKER_FRAME_ID = "map";
  MARKER_NS = "add_markers";
  
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  initMarker();

}

void ConstantMarkerAdder::displayMarkers() {
  // show the pickup marker
  setMarker(pickupZone[0], pickupZone[1], pickupZone[2]);
  showMarker(true);

  // hide the pickup marker after five seconds
  ros::Duration(SLEEP_TIME).sleep();
  showMarker(false);

  // show the dropoff marker after another five seconds
  ros::Duration(SLEEP_TIME).sleep();
  setMarker(dropoffZone[0], dropoffZone[1], dropoffZone[2]);
  showMarker(true);
}

// Initialize the marker at the origin as a blue square marker
void ConstantMarkerAdder::initMarker() {
    marker.header.frame_id = MARKER_FRAME_ID;
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = MARKER_NS;
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = MARKER_SHAPE;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = MARKER_SIZE;
    marker.scale.y = MARKER_SIZE;
    marker.scale.z = MARKER_SIZE;

    // Set the color
    marker.color.r = MARKER_COLOR[0];
    marker.color.g = MARKER_COLOR[1];
    marker.color.b = MARKER_COLOR[2];
    marker.color.a = 0.0;

    marker.lifetime = ros::Duration();
}

// Create a marker based on the provided shape, pose and color information. Since the marker is only for
// 2D poses, the pose information only required x and y for position, and z for orientation.
void ConstantMarkerAdder::setMarker(double positionX, double positionY, double rotationZ) {
  marker.pose.position.x = positionX;
  marker.pose.position.y = positionY;
  marker.pose.orientation.z = rotationZ;
  marker.pose.orientation.w = sqrt(1.0 - rotationZ * rotationZ);
}

void ConstantMarkerAdder::showMarker(bool showing) {
  if (showing) {
    marker.color.a = 1.0;
  } else {
    marker.color.a = 0.0;
  }

  // publish the marker to its subscribers, most often rviz
  while (marker_pub.getNumSubscribers() < 1) {
    if (!ros::ok()) {
      return;
    }
    ROS_WARN_ONCE("Please create a subscriber to the marker");
    sleep(1);
  }
  marker_pub.publish(marker);
}