#ifndef BENDING_DAMAGE_NORMAL_CORRESPONDENCE_H
#define BENDING_DAMAGE_NORMAL_CORRESPONDENCE_H

#include "pcl/point_types.h"

#include <cstdint>

struct BendingDamageNormalCorrespondence {
    std::size_t normalIndexA;
    std::size_t normalIndexB;
    pcl::PointCloud<pcl::Normal>::Ptr pointCloudsA;
    pcl::PointCloud<pcl::Normal>::Ptr pointCloudsB;
};

#endif