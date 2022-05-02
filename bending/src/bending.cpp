#include "bending/BendingDamageAssessment.h"

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

// Create an instance to use in callback
static BendingDamageAssessment bend;

void bendingCallback(const sensor_msgs::PointCloud2::ConstPtr& cardboard) {
    ROS_INFO("Found rosbag entry with sequence id [%d]!", cardboard->header.seq);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cardboardPointCloud = bend.ros2PCL(cardboard);

    ROS_INFO("Adding point cloud [%d]...", cardboard->header.seq);
    bend.addPointCloud(cardboardPointCloud);
    ROS_INFO("Point cloud [%d] added!", cardboard->header.seq);

    ROS_INFO("Point cloud sequence ID [%d]", cardboardPointCloud->header.seq);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "bending");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/multisense/image_points2_color", 1000, bendingCallback);

    ros::Time startTime = ros::Time::now();
    ros::Duration loopDuration(60.0); // 1 min
    while ((ros::Time::now() < startTime + loopDuration) && ros::ok()) {
        ros::spinOnce();
    }

    BendingDamageErrorData maxNormalMSE = bend.getSequenceWithMaxMSE();
    ROS_INFO("Max MSE: %f", maxNormalMSE.error);

    return 0;
}