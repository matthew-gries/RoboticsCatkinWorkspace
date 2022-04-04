#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_types.h"
#include "pcl/PCLPointCloud2.h"
#include "pcl/conversions.h"
#include "pcl_ros/transforms.h"

/**
 * @brief Class to do bending damage assessment
 * 
 */
struct BendingDamageAssessment {

    // Convert a ROS PointCloud2 to a PCL point cloud
    static pcl::PointCloud<pcl::PointXYZRGB>::Ptr ros2PCL(const sensor_msgs::PointCloud2::ConstPtr& rosCloud);
};

pcl::PointCloud<pcl::PointXYZRGB>::Ptr BendingDamageAssessment::ros2PCL(const sensor_msgs::PointCloud2::ConstPtr& rosCloud) {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*rosCloud, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclPtr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(pcl_pc2, *pclPtr);
    return pclPtr;
}

// Create an instance to use in callback
static BendingDamageAssessment bend;

void bendingCallback(const sensor_msgs::PointCloud2::ConstPtr& cardboard) {
    ROS_INFO("Found rosbag entry with sequence id [%d]!", cardboard->header.seq);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cardboardPointCloud = bend.ros2PCL(cardboard);

    ROS_INFO("Point cloud sequence ID [%d]", cardboardPointCloud->header.seq);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "bending");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/multisense/image_points2_color", 1000, bendingCallback);

    ros::spin();

    return 0;
}