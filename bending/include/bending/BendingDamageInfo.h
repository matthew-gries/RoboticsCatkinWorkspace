#ifndef BENDING_DAMAGE_INFO_H
#define BENDING_DAMAGE_INFO_H

#include <cstdint>

#include "pcl/point_types.h"
#include "pcl/common/distances.h"
#include "pcl/ModelCoefficients.h"
#include "pcl/PointIndices.h"

/**
 * @brief Structure representing information received when doing damage assessment on
 * a point cloud
 */
struct BendingDamageInfo {
    std::uint32_t seq;
    pcl::ModelCoefficients::Ptr modelCoefficients;
    pcl::PointIndices::Ptr inliers;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster;
    pcl::PointCloud<pcl::Normal>::Ptr normals;
};




#endif