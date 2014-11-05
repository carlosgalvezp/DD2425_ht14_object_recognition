#include <object_recognition/feature_extractor.h>

Feature_Extractor::Feature_Extractor()
{
}

void Feature_Extractor::get_descriptors(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud_in,
                                              pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr &descriptors)
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


void Feature_Extractor::preprocessing(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud_in,
                                            pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out)
{
    std::clock_t begin = clock();

    // Voxel Grid filter (downsampling)
    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    vg.setInputCloud (cloud_in);
    vg.setLeafSize (CLOUD_RES, CLOUD_RES, CLOUD_RES);
    vg.filter (*cloud_out);

    std::clock_t end = clock();
    ROS_INFO("Preprocessing [%.3f ms]. Before: %u points. After: %u points",
             RAS_Utils::time_diff_ms(begin, end), cloud_in->size(), cloud_out->size());
}

void Feature_Extractor::segmentation(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud_in,
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

void Feature_Extractor::keypoint_extraction(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud_in,
                                                  pcl::PointCloud<pcl::PointXYZRGB>::Ptr &keypoints)
{
    // ** Subsample cloud since there is not really texture in cloud
    std::clock_t begin = clock();

    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    vg.setInputCloud (cloud_in);
    vg.setLeafSize (KEYPOINT_RES, KEYPOINT_RES, KEYPOINT_RES);
    vg.filter (*keypoints);

    std::clock_t end = clock();
    ROS_INFO("Keypoints [%.3f ms]. Before: %u points. After: %u points",
             RAS_Utils::time_diff_ms(begin, end), cloud_in->size(), keypoints->size());
}

void Feature_Extractor::descriptor_extraction(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &keypoints,
                                              const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud_in,
                                                    pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr &descriptors)
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
    pcl::PFHRGBEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PFHRGBSignature250> pfhrgb;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    pfhrgb.setSearchMethod (tree);

    // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
    pfhrgb.setRadiusSearch (0.05); // Tune this, has to be larger than the previous one

    descriptors.reset(new pcl::PointCloud<pcl::PFHRGBSignature250>);

    pfhrgb.setInputCloud (keypoints);
    pfhrgb.setSearchSurface(cloud_in);
    pfhrgb.setInputNormals (cloud_normals);
    pfhrgb.compute (*descriptors);

    double t_descriptors = RAS_Utils::time_diff_ms(begin, clock());

    ROS_INFO("Descritors [Normal: %.3f, Descriptor: %.3f, TOTAL: %.3f ms]", t_normals, t_descriptors, t_normals + t_descriptors);
}
