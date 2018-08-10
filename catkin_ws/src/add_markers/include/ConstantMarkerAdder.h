/**
 * The ConstantMarkerAdder class publishes a marker at the designated pickup zone initially, pause for five seconds,
 * hide the marker, pause for another five seconds, and lastly show the marker in the dropoff zone.
 */
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

class ConstantMarkerAdder {
public:
  ConstantMarkerAdder(const double* pickupZone, const double* dropoffZone);
  void displayMarkers();

protected:
  void initMarker();
  void setMarker(double positionX, double positionY, double rotationZ);
  void showMarker(bool hide);

  ros::NodeHandle n;
  ros::Publisher marker_pub;
  visualization_msgs::Marker marker;

  std::string MARKER_NS;
  std::string MARKER_FRAME_ID;
  uint32_t MARKER_SHAPE = visualization_msgs::Marker::CUBE;
  double MARKER_SIZE = 0.5;
  double MARKER_COLOR[3];

  const double* pickupZone;
  const double* dropoffZone;

  constexpr static double SLEEP_TIME = 5.0;
};