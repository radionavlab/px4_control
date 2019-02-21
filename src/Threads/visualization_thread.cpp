
#include "visualization_thread.h"

void VisualizationThread(const double &rate, const std::string &ns) {
    ROS_DEBUG("[px4_control] Visualization Thread started!");

    // Visualization marker parameters
    const std::string frame_id = "map";
    const std::string quad_mesh = "quadrotor_base.dae";
    const double quad_size = 0.375;
    const double reference_transparency = 0.5;
    const double pos_transparency = 1.0;
    const std_msgs::ColorRGBA frame_color = visualization_functions::Color::White();
    const std_msgs::ColorRGBA mesh_color = visualization_functions::Color::Red();
    const std_msgs::ColorRGBA ref_mesh_color = visualization_functions::Color::Green();
    const double reference_size = 0.1;
    const double max_force_marker_length = 10.0;
    const Eigen::Quaterniond rot_quad(cos(M_PI/8.0), 0.0, 0.0, sin(M_PI/8.0)); // Quad mesh is rotated by 45deg
    visualization_msgs::MarkerArray quadArray;
    geometry_msgs::PoseStamped RvizPoseRef;
    nav_msgs::Odometry localOdom;

    // Create publisher
    ros::NodeHandle n; 
    ros::Publisher pub_vis = n.advertise<visualization_msgs::MarkerArray>("px4_control_node/local_setpoint_mesh",100);

    ros::Rate loop_rate(rate);

    while (ros::ok()) {
        //Check if thread should be terminated
        if(WaitForEvent(syncEvents.Terminate,0) == 0){
            break;
        }

        // Empty visualization markers
        quadArray.markers.clear();

        // Reference info
        pthread_mutex_lock(&mutexes.rviz_pose_ref);
            RvizPoseRef = RvizPoseRef_;
        pthread_mutex_unlock(&mutexes.rviz_pose_ref);

        //Get odometry info
        pthread_mutex_lock(&mutexes.odom);
            localOdom = odom;
        pthread_mutex_unlock(&mutexes.odom);

        // Get some parameters for visualization
        const Eigen::Quaterniond orientationMesh = ros2eigenquat(RvizPoseRef.pose.orientation)*rot_quad;
        const Eigen::Vector3d ml_ref_pos = rosPoint2eigenVector(RvizPoseRef.pose.position);

        // Get reference quad mesh
        visualization_functions::MeshMarker(ml_ref_pos, orientationMesh,
                    frame_id, ns, quad_mesh, quad_size, ref_mesh_color, 
                    reference_transparency, 1, &quadArray);

        // Publish quad postion
        const Eigen::Quaterniond q = ros2eigenquat(localOdom.pose.pose.orientation);
        const Eigen::Matrix3d orientation_frame = q.normalized().toRotationMatrix();
        const Eigen::Vector3d position = rosPoint2eigenVector(localOdom.pose.pose.position);

        // Get quad mesh
        visualization_functions::MeshMarker(position, q*rot_quad,
                    frame_id, ns, quad_mesh, quad_size, mesh_color, 
                    pos_transparency, 2, &quadArray);

        // Get triad frame arrows
        const double arrowLength = 0.2; 
        visualization_functions::FrameMarker(position, orientation_frame,
                    frame_id, ns, frame_color, 3, arrowLength, &quadArray);

        pub_vis.publish(quadArray);

        loop_rate.sleep();
    }

    //Shutdown here
    pthread_mutex_lock(&mutexes.threadCount);
        threadCount -= 1;
    pthread_mutex_unlock(&mutexes.threadCount);


    ROS_DEBUG("Exiting Visualization Thread...");
}