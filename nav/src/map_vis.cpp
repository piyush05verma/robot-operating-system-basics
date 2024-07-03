#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>

// Define ROS subscriber and publisher
ros::Subscriber path_sub;
ros::Publisher marker_pub;

// Callback function for path subscriber
void pathCallback(const nav_msgs::Path::ConstPtr& msg) {
    // Create visualization marker
    visualization_msgs::Marker marker;
    marker.header = msg->header;
    marker.ns = "path";
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1; // Width of the line
    marker.color.r = 1.0;
    marker.color.a = 1.0;

    // Fill marker points from path
    for (const auto& pose : msg->poses) {
        marker.points.push_back(pose.pose.position);
    }

    // Publish marker
    marker_pub.publish(marker);
}

int main(int argc, char** argv) {
    // Initialize ROS node
    ros::init(argc, argv, "path_visualizer");
    ros::NodeHandle nh;

    // Subscribe to the path topic
    path_sub = nh.subscribe("/path", 1, pathCallback);

    // Advertise the visualization marker topic
    marker_pub = nh.advertise<visualization_msgs::Marker>("/path_marker", 1);

    // Spin
    ros::spin();

    return 0;
}
