#ifndef PX4_CONTROL_VISUALIZATION_FUNCTIONS_H_
#define PX4_CONTROL_VISUALIZATION_FUNCTIONS_H_

#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Dense>
#include <Eigen/Geometry> 
#include "helper.h"


namespace visualization_functions {

// Some colors for visualization markers
class Color : public std_msgs::ColorRGBA {
 public:
  Color() : std_msgs::ColorRGBA() {}
  Color(double red, double green, double blue) : Color(red, green, blue, 1.0) {}
  Color(double red, double green, double blue, double alpha) : Color() {
    r = red;
    g = green;
    b = blue;
    a = alpha;
  }

  static const Color White() { return Color(1.0, 1.0, 1.0); }
  static const Color Black() { return Color(0.0, 0.0, 0.0); }
  static const Color Gray() { return Color(0.5, 0.5, 0.5); }
  static const Color Red() { return Color(1.0, 0.0, 0.0); }
  static const Color Green() { return Color(0.0, 1.0, 0.0); }
  static const Color DarkGreen() { return Color(0.0, 0.3, 0.0); }
  static const Color Blue() { return Color(0.0, 0.0, 1.0); }
  static const Color Yellow() { return Color(1.0, 1.0, 0.0); }
  static const Color Orange() { return Color(1.0, 0.5, 0.0); }
  static const Color Purple() { return Color(0.5, 0.0, 1.0); }
  static const Color Chartreuse() { return Color(0.5, 1.0, 0.0); }
  static const Color Teal() { return Color(0.0, 1.0, 1.0); }
  static const Color Pink() { return Color(1.0, 0.0, 0.5); }
};

void SelectColor(const uint &i, std_msgs::ColorRGBA *color);

void SelectColor(const std::string &des_color, std_msgs::ColorRGBA *color);

// Quadcopter mesh
void MeshMarker(const Eigen::Vector3d &point,
                const Eigen::Quaterniond &quat,
                const std::string &frame_id,
                const std::string &ns,  // namespace
                const std::string &mesh_file,
                const double &size,
                const std_msgs::ColorRGBA &color,
                const double &transparency,  // 0 -> transparent, 1 -> opaque
                const int &seqNumber,
                visualization_msgs::MarkerArray *markerArray);

// xyz frame with origin at quadcopter position
void FrameMarker(const Eigen::Vector3d &point,
                 const Eigen::Matrix3d &Rot,
                 const std::string &frame_id,
                 const std::string &ns,  // namespace
                 const std_msgs::ColorRGBA &color,
                 const int &seqNumber,
                 const double length,
                 visualization_msgs::MarkerArray *markerArray);

}


#endif  // PX4_CONTROL_VISUALIZATION_FUNCTIONS_H_