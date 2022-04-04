#ifndef BENDING_DAMAGE_ASSESSMENT_H
#define BENDING_DAMAGE_ASSESSMENT_H

#include "bending/BendingDamageInfo.h"

#include <vector>
#include "pcl/point_types.h"
#include "sensor_msgs/PointCloud2.h"

/**
 * @brief Class to do bending damage assessment
 * 
 */
struct BendingDamageAssessment {

    /** Non-member functions **/

    // Convert a ROS PointCloud2 to a PCL point cloud
    static pcl::PointCloud<pcl::PointXYZRGB>::Ptr ros2PCL(const sensor_msgs::PointCloud2::ConstPtr& rosCloud);

    /** Member functions **/

    // Check if the reference point cloud has been set
    bool isRefPointCloudSet();

    // Add a point cloud. If this is the first point cloud added, it is set as the reference
    void addPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pointCloud);

    /** Fields **/

    // Reference point cloud representing data for no damage
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr reference;

    // Vector representing the damage information. The first element corresponds to the information extracted from the reference cloud
    std::vector<BendingDamageInfo> damageInfo;
};



#endif