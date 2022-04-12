#ifndef BENDING_DAMAGE_ASSESSMENT_H
#define BENDING_DAMAGE_ASSESSMENT_H

#include "bending/BendingDamageInfo.h"
#include "bending/BendingDamageNormalCorrespondance.h"
#include "bending/BendingDamageErrorData.h"

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

    // Get the bending damage info of the point cloud identified by the sequence number. Assumes the sequence number is valid, returns an empty damage info field
    // if not
    BendingDamageInfo& getDamageInfoBySequenceNumber(std::uint32_t seq);

    // Find all pairs of closest normals between normals in sequence `a` and sequence `b`
    std::vector<BendingDamageNormalCorrespondence> findCorrespondences(std::uint32_t a, std::uint32_t b);

    // Get the sequence with the maximum MSE
    BendingDamageErrorData getSequenceWithMaxMSE();

    /** Fields **/

    // Reference point cloud representing data for no damage
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr reference;

    // Vector representing the damage information. The first element corresponds to the information extracted from the reference cloud
    std::vector<BendingDamageInfo> damageInfo;

private:

    // Calculate the mse between two sequences
    float mse(std::uint32_t a, std::uint32_t b);

    // Dummy info
    BendingDamageInfo mDummyInfo;
};



#endif