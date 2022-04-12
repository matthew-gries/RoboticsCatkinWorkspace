#ifndef BENDING_DAMAGE_ERROR_DATA_H
#define BENDING_DAMAGE_ERROR_DATA_H

#include <cstdint>

#include "pcl/point_types.h"
#include "pcl/common/distances.h"
#include "pcl/ModelCoefficients.h"
#include "pcl/PointIndices.h"

enum BendDamageErrorDataType {
    MSE_BETWEEN_NEARSET_NORMALS
};

/**
 * @brief Structure representing information about the error between a sequence
 */
struct BendingDamageErrorData {
    std::uint32_t seq;
    std::uint32_t refSeq;
    float error;
    BendDamageErrorDataType type;
};

#endif