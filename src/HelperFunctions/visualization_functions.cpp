#include "visualization_functions.h"

namespace visualization_functions {


void SelectColor(const uint &i, std_msgs::ColorRGBA *color) {
	const uint n_colors = 9;
	const uint index = i % n_colors;  // Returns a number between 0 and 8
	switch(index) {
        case 0: *color = Color::Red(); break;
        case 1: *color = Color::Blue(); break;
        case 2: *color = Color::Green(); break;
        case 3: *color = Color::Yellow(); break;
        case 4: *color = Color::Orange(); break;
        case 5: *color = Color::Purple(); break;
        case 6: *color = Color::Chartreuse(); break;
        case 7: *color = Color::Teal(); break;
        case 8: *color = Color::Pink(); break;
    }
}

void SelectColor(const std::string &des_color, std_msgs::ColorRGBA *color) {
	if(des_color.compare("red") == 0) {
		*color = Color::Red();
	} else if(des_color.compare("blue") == 0) {
		*color = Color::Blue();
	} else if(des_color.compare("green") == 0) {
		*color = Color::Green();
	} else if(des_color.compare("yellow") == 0) {
		*color = Color::Yellow();
	} else if(des_color.compare("orange") == 0) {
		*color = Color::Orange();
	} else if(des_color.compare("purple") == 0) {
		*color = Color::Purple();
	} else if(des_color.compare("chartreuse") == 0) {
		*color = Color::Chartreuse();
	} else if(des_color.compare("teal") == 0) {
		*color = Color::Teal();
	} else if(des_color.compare("pink") == 0) {
		*color = Color::Pink();
	} else {
		*color = Color::White();
	} 
}

void MeshMarker(const Eigen::Vector3d &point,
	            const Eigen::Quaterniond &quat,
	            const std::string &frame_id,
	            const std::string &ns,  // namespace
                const std::string &mesh_file,
	            const double &size,
                const std_msgs::ColorRGBA &color,
                const double &transparency,  // 0 -> transparent, 1 -> opaque
	            const int &seqNumber,
	            visualization_msgs::MarkerArray *markerArray) {
	visualization_msgs::Marker marker;
	marker.type = visualization_msgs::Marker::MESH_RESOURCE;
	marker.mesh_resource = "package://px4_control/meshes/" + mesh_file;
	marker.action = visualization_msgs::Marker::ADD;
	marker.color = color;
	marker.color.a = transparency;
	marker.scale.x = size;
	marker.scale.y = size;
	marker.scale.z = size;
	marker.ns = ns;
	marker.header.frame_id = frame_id;
	marker.header.stamp = ros::Time::now();
	marker.pose.orientation.w = quat.w();
	marker.pose.orientation.x = quat.x();
	marker.pose.orientation.y = quat.y();
	marker.pose.orientation.z = quat.z();

	geometry_msgs::Point position;
	position.x = point(0);
	position.y = point(1);
	position.z = point(2);
	marker.pose.position = position;
	marker.id = seqNumber;
	marker.lifetime = ros::Duration(1.0);
	markerArray->markers.push_back(marker);
}

void FrameMarker(const Eigen::Vector3d &point,
                 const Eigen::Matrix3d &Rot,
                 const std::string &frame_id,
                 const std::string &ns,  // namespace
                 const std_msgs::ColorRGBA &color,
                 const int &seqNumber,
                 const double length,
                 visualization_msgs::MarkerArray *markerArray) {
	// const double diameter = 0.01;
	visualization_msgs::Marker marker;
	marker.type = visualization_msgs::Marker::ARROW;
	marker.action = visualization_msgs::Marker::ADD;
	marker.color = color;
	marker.color.a = 1.0;
	marker.scale.x = length*0.1;
	marker.scale.y = length*2*0.1;
	marker.scale.z = length*0.2;
	marker.ns = ns;
	marker.header.frame_id = frame_id;
	marker.header.stamp = ros::Time::now();
	marker.lifetime = ros::Duration(1.0);

	Eigen::Vector3d x_dir, y_dir, z_dir, xf, yf, zf;
	x_dir << Rot(0,0), Rot(1,0), Rot(2,0);
	y_dir << Rot(0,1), Rot(1,1), Rot(2,1);
	z_dir << Rot(0,2), Rot(1,2), Rot(2,2);
	xf = point + length*x_dir;
	yf = point + length*y_dir;
	zf = point + length*z_dir;

	// Add points
	// geometry_msgs::Point p0 = SetPoint(point(0), point(1), point(2));
	marker.points.resize(2);
	marker.points[0] = SetPoint(point(0), point(1), point(2));
	marker.points[1] = SetPoint(xf(0), xf(1), xf(2));
	marker.id = seqNumber;
	markerArray->markers.push_back(marker);
	marker.points[1] = SetPoint(yf(0), yf(1), yf(2));
	marker.id = seqNumber + 1;
	markerArray->markers.push_back(marker);
	marker.points[1] = SetPoint(zf(0), zf(1), zf(2));
	marker.id = seqNumber + 2;

	markerArray->markers.push_back(marker);
}

}  // namespace visualization_functions