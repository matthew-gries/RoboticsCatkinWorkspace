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