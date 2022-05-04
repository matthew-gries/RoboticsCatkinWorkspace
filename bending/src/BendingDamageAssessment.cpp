#include "bending/BendingDamageAssessment.h"

#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/PCLPointCloud2.h"
#include "pcl/conversions.h"
#include "pcl_ros/transforms.h"
#include "pcl/features/normal_3d.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/search/kdtree.h"
#include "ros/ros.h"

#include <algorithm>
#include <limits>

pcl::PointCloud<pcl::PointXYZRGB>::Ptr BendingDamageAssessment::ros2PCL(const sensor_msgs::PointCloud2::ConstPtr& rosCloud) {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*rosCloud, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclPtr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(pcl_pc2, *pclPtr);
    return pclPtr;
}

bool BendingDamageAssessment::isRefPointCloudSet() {
    return static_cast<bool>(this->reference);
}

void BendingDamageAssessment::addPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pointCloud) {

    if (!this->isRefPointCloudSet()) {
        this->reference = pointCloud;
    }

    /**
     * @brief Segment the point cloud into planes, should be able to find the cardboard
     * 
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*pointCloud, *cloud_filtered);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_temp(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*pointCloud, *cloud_filtered_temp);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.05);
    seg.setInputCloud(pointCloud);
    seg.segment(*inliers, *coefficients);

    pcl::ExtractIndices<pcl::PointXYZRGB> extract;

    int i = 0, nr_points = (int)pointCloud->size();

    while (cloud_filtered->size() > 0.3 * nr_points) {

        seg.setInputCloud(cloud_filtered);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.size() == 0) {
            PCL_ERROR("Could not estimate a planar model for the given dataset.\n");
            return;
        }

        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers);
        extract.filter(*cloud_filtered_temp);

        cloud_filtered.swap(cloud_filtered_temp);
    }

    for (int i = 0; i < cloud_filtered->points.size(); i++) {
        cloud_filtered->points[i].r = 0;
        cloud_filtered->points[i].g = 255;
        cloud_filtered->points[i].b = 0;
    }

    /**
     * @brief Perform cluster extraction to get the cluster representing the cardboard
     * 
     */
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(cloud_filtered);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(0.02);
    ec.setMinClusterSize(10);
    ec.setMaxClusterSize(250000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_filtered);
    ec.extract(cluster_indices);

    // Find the largest cluster
    int size = -1;
    size_t largest_idx = 0;
    for (size_t i = 0; i < cluster_indices.size(); i++) {
        if ((int)cluster_indices[i].indices.size() > size) {
            size = cluster_indices[i].indices.size();
            largest_idx = i;
        }
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (const auto& idx : cluster_indices[largest_idx].indices) {
        cluster_cloud->push_back((*cloud_filtered)[idx]);
    }

    /**
     * @brief Find the normals of the cluster representing the cardboard
     * 
     */
    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setInputCloud(cluster_cloud);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr ne_tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
    ne.setSearchMethod(ne_tree);

    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch(0.03);

    // Compute the features
    ne.compute(*cloud_normals);

    BendingDamageInfo info {
        pointCloud->header.seq,
        coefficients,
        inliers,
        cluster_cloud,
        cloud_normals
    };

    this->damageInfo.push_back(info);
}

BendingDamageInfo& BendingDamageAssessment::getDamageInfoBySequenceNumber(std::uint32_t seq) {
    std::vector<BendingDamageInfo>::iterator seqIter = std::find_if(
        this->damageInfo.begin(),
        this->damageInfo.end(),
        [seq](BendingDamageInfo& info) {return info.seq == seq;}
    );

    if (seqIter == this->damageInfo.end()) {
        return this->mDummyInfo;
    }

    return *seqIter;
}

std::vector<BendingDamageNormalCorrespondence> BendingDamageAssessment::findCorrespondences(std::uint32_t a, std::uint32_t b) {

    // Get the point clouds specified by the sequence number
    std::vector<BendingDamageNormalCorrespondence> corresp;

    BendingDamageInfo& aInfo = this->getDamageInfoBySequenceNumber(a);
    BendingDamageInfo& bInfo = this->getDamageInfoBySequenceNumber(b);

    if (!aInfo.normals || !bInfo.normals) {
        return corresp;
    }

    // Order the point clouds by which is larger and which is smaller
    bool aLargerThanB = aInfo.normals->size() > bInfo.normals->size();

    pcl::PointCloud<pcl::Normal>::Ptr largerPointCloud = (aLargerThanB) ? aInfo.normals : bInfo.normals;
    pcl::PointCloud<pcl::Normal>::Ptr smallerPointCloud = (aLargerThanB) ? bInfo.normals : aInfo.normals;

    for (size_t i = 0; i < smallerPointCloud->size(); i++) {

        float minDist = std::numeric_limits<float>::max();
        size_t minJ = 0;

        const pcl::Normal& norm_i = smallerPointCloud->at(i);

        for (size_t j = 0; j < largerPointCloud->size(); j++) {

            const pcl::Normal& norm_j = largerPointCloud->at(j);

            float dist = sqrt(
                pow(norm_i.normal_x - norm_j.normal_x, 2) +
                pow(norm_i.normal_y - norm_j.normal_y, 2) +
                pow(norm_i.normal_z - norm_j.normal_z, 2) * 1.0
            );

            if (dist < minDist) {
                minDist = dist;
                minJ = j;
            }
        }

        if (aLargerThanB) { corresp.push_back(BendingDamageNormalCorrespondence{minJ, i, largerPointCloud, smallerPointCloud}); }
        else { corresp.push_back(BendingDamageNormalCorrespondence{i, minJ, smallerPointCloud, largerPointCloud}); }
    }

    return corresp;
}

BendingDamageErrorData BendingDamageAssessment::getSequenceWithMaxMSE() {

    std::uint32_t refSeq = this->reference->header.seq;
    float maxError = std::numeric_limits<float>::min();
    uint32_t maxErrorSeq = refSeq;

    for (const BendingDamageInfo& info : this->damageInfo) {
        if (info.seq == this->reference->header.seq) {
            continue;
        }
        ROS_INFO("Geting MSE...");
        float error = this->mse(refSeq, info.seq);

        if (error > maxError) {
            maxError = error;
            maxErrorSeq = info.seq;
        }

    }

    return BendingDamageErrorData {
        maxErrorSeq,
        refSeq,
        maxError,
        BendDamageErrorDataType::MSE_BETWEEN_NEARSET_NORMALS
    };
}

float BendingDamageAssessment::mse(std::uint32_t a, std::uint32_t b) {
    ROS_INFO("Finding correspondences...");
    std::vector<BendingDamageNormalCorrespondence> corresp = this->findCorrespondences(a, b);
    ROS_INFO("Correspondences found!");

    float totalError = 0.0;

    for (const BendingDamageNormalCorrespondence& c : corresp) {
        // ROS_INFO("%ld %ld", c.pointCloudsA->size(), c.pointCloudsB->size());
        pcl::Normal& aNorm = c.pointCloudsA->at(c.normalIndexA);
        pcl::Normal& bNorm = c.pointCloudsB->at(c.normalIndexB);

        totalError += sqrt(
                pow(aNorm.normal_x - bNorm.normal_x, 2) +
                pow(aNorm.normal_y - bNorm.normal_y, 2) +
                pow(aNorm.normal_z - bNorm.normal_z, 2) * 1.0
            );
    }

    return totalError / corresp.size();
}