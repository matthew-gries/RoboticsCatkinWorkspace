#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

void bendingCallback(const sensor_msgs::PointCloud2::ConstPtr& cardboard) {
    ROS_INFO("Found: [%s]", cardboard->header.frame_id.c_str());
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "bending");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/multisense/image_points2_color", 1000, bendingCallback);

    ros::spin();

    return 0;
}