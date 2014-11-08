#ifndef FEATURE_EXTRACTOR_H
#define FEATURE_EXTRACTOR_H

#include <vector>
#include <iostream>
#include <ras_utils/ras_utils.h>
// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/pfhrgb.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>

#define CLOUD_RES  0.005 // [m]
#define KEYPOINT_RES 0.01 // [m]
#define PCL_MAX_Z    0.45 // [m]
#define PCL_MAX_X    0.3 // [m]
/**
 * @brief Takes as an input a point cloud image containing one object, and
 * performs the following operations:
 * 1.- Segmention of the object
 * 2.- Detection of interest points
 * 3.- Feature extraction
 */
template<template<typename, typename, typename> class DescriptorExtractor, typename DescriptorType>
class Feature_Extractor
{
public:
    /**
     * @brief Default constructor
     */
    Feature_Extractor();

    /**
     * @brief Given a raw input point cloud extracts the object's feature descriptors
     * @param cloud_in the raw input point cloud
     */
    void get_descriptors(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud_in,
                      typename pcl::PointCloud<DescriptorType>::Ptr &descriptors);

private:
    /**
     * @brief Preprocesses the input cloud: pass-through filter, sampling, region of interest
     * @param cloud_in  input cloud
     * @param cloud_out output cloud
     */
    void preprocessing(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud_in,
                             pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out);

    /**
     * @brief Segments the object from the scene
     * @param cloud_in  input cloud, containing the whole scene
     * @param cloud_out output cloud, containing only the 3D object
     */
    void segmentation(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud_in,
                            pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out);
    /**
     * @brief Extracts keypoints from the object
     * @param cloud_in input cloud of the object
     * @param keypoints cloud of keypoints
     */
    void keypoint_extraction(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud_in,
                                   pcl::PointCloud<pcl::PointXYZRGB>::Ptr &keypoints);

    /**
     * @brief Extracts feature descriptors for each keypoint
     * @param keypoints the input keypoints
     */
    void descriptor_extraction(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &keypoints,
                               const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud_in,
                            typename pcl::PointCloud<DescriptorType>::Ptr &descriptors);
};

// ==========================================================================================
// ==========================================================================================
template<template<typename, typename, typename> class DescriptorExtractor, typename DescriptorType>
Feature_Extractor<DescriptorExtractor, DescriptorType>::Feature_Extractor()
{
}

template<template<typename, typename, typename> class DescriptorExtractor, typename DescriptorType>
void Feature_Extractor<DescriptorExtractor, DescriptorType>::get_descriptors(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud_in,
                                                                          typename pcl::PointCloud<DescriptorType>::Ptr &descriptors)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr object (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints (new pcl::PointCloud<pcl::PointXYZRGB>);

    // ** Preprocess the input cloud
    preprocessing(cloud_in, cloud_in_filtered);

    // ** Segment object
    segmentation(cloud_in_filtered, object);

    // ** Extract keypoints
    keypoint_extraction(object, keypoints);

    // ** Get descriptor
    descriptor_extraction(keypoints, cloud_in_filtered, descriptors);
}

template<template<typename, typename, typename> class DescriptorExtractor, typename DescriptorType>
void Feature_Extractor<DescriptorExtractor, DescriptorType>::preprocessing(
                                      const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud_in,
                                            pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out)
{
    std::clock_t begin = clock();

    // ** Voxel Grid filter (downsampling)
    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    vg.setInputCloud (cloud_in);
    vg.setLeafSize (CLOUD_RES, CLOUD_RES, CLOUD_RES);
    vg.filter (*cloud_out);

    // ** Pass-through filter
    pcl::PassThrough<pcl::PointXYZRGB> pt_filter;
    pt_filter.setInputCloud (cloud_out);
    pt_filter.setFilterFieldName ("z");
    pt_filter.setFilterLimits (0.0, PCL_MAX_Z);
    pt_filter.filter (*cloud_out);
    pt_filter.setInputCloud (cloud_out);
    pt_filter.setFilterFieldName ("x");
    pt_filter.setFilterLimits (-PCL_MAX_X/2.0, PCL_MAX_X/2.0);
    pt_filter.filter (*cloud_out);

//    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_out);
//    viewer->addPointCloud<pcl::PointXYZRGB> (cloud_out, rgb,"sample cloud");
//    viewer->addCoordinateSystem();
//    viewer->spin();
    std::clock_t end = clock();
    ROS_INFO("Preprocessing [%.3f ms]. Before: %lu points. After: %lu points",
             RAS_Utils::time_diff_ms(begin, end), cloud_in->size(), cloud_out->size());
}

template<template<typename, typename, typename> class DescriptorExtractor, typename DescriptorType>
void Feature_Extractor<DescriptorExtractor, DescriptorType>::segmentation(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud_in,
                                           pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out)
{
    std::clock_t begin = clock();

    // ** Remove planes

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (0.01);

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;

    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_in);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    // Extract the object
    extract.setInputCloud (cloud_in);
    extract.setIndices (inliers);
    extract.setNegative (true); //Extract the object, not the plane!
    extract.filter (*cloud_out);
    std::cout << "PointCloud representing the planar component: " << cloud_out->size() << " data points." << std::endl;
    std::clock_t end = clock();

    // ** Get largest cluster
    /// @todo

    ROS_INFO("Segmentation [%.3f ms]", RAS_Utils::time_diff_ms(begin, end));
}

template<template<typename, typename, typename> class DescriptorExtractor, typename DescriptorType>
void Feature_Extractor<DescriptorExtractor, DescriptorType>::keypoint_extraction(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud_in,
                                                  pcl::PointCloud<pcl::PointXYZRGB>::Ptr &keypoints)
{
    // ** Subsample cloud since there is not really texture in cloud
    std::clock_t begin = clock();

    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    vg.setInputCloud (cloud_in);
    vg.setLeafSize (KEYPOINT_RES, KEYPOINT_RES, KEYPOINT_RES);
    vg.filter (*keypoints);

    std::clock_t end = clock();
    ROS_INFO("Keypoints [%.3f ms]. Before: %lu points. After: %lu points",
             RAS_Utils::time_diff_ms(begin, end), cloud_in->size(), keypoints->size());
}

template<template<typename, typename, typename> class DescriptorExtractor, typename DescriptorType>
void Feature_Extractor<DescriptorExtractor, DescriptorType>::descriptor_extraction(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &keypoints,
                                              const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud_in,
                                           typename pcl::PointCloud<DescriptorType>::Ptr &descriptors)
{
    std::clock_t begin = clock();

    // ** First, compute normals
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>());
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimation;
    normal_estimation.setSearchMethod
        (pcl::search::KdTree<pcl::PointXYZRGB>::Ptr (new pcl::search::KdTree<pcl::PointXYZRGB>));
    normal_estimation.setRadiusSearch (0.015); // Tune this
    normal_estimation.setInputCloud (cloud_in);
    normal_estimation.compute (*cloud_normals);

    double t_normals = RAS_Utils::time_diff_ms(begin,clock());
    begin = clock();
    // ** Second, compute descriptors
    DescriptorExtractor<pcl::PointXYZRGB, pcl::Normal, DescriptorType> pfhrgb;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    pfhrgb.setSearchMethod (tree);

    // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
    pfhrgb.setRadiusSearch (0.05); // Tune this, has to be larger than the previous one

    descriptors.reset(new pcl::PointCloud<DescriptorType>);

    pfhrgb.setInputCloud (keypoints);
    pfhrgb.setSearchSurface(cloud_in);
    pfhrgb.setInputNormals (cloud_normals);
    pfhrgb.compute (*descriptors);

    double t_descriptors = RAS_Utils::time_diff_ms(begin, clock());

    ROS_INFO("Descritors [Normal: %.3f, Descriptor: %.3f, TOTAL: %.3f ms]", t_normals, t_descriptors, t_normals + t_descriptors);
}


#endif // FEATURE_EXTRACTOR_H
