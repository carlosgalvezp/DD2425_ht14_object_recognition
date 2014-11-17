#include <object_recognition/object_extractor_3d.h>

Object_Extractor_3D::Object_Extractor_3D()
{
}

void Object_Extractor_3D::extract_object(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud_in,
                                            pcl::PointCloud<pcl::PointXYZRGB>::Ptr      &cloud_out)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_preproc(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_no_planes(new pcl::PointCloud<pcl::PointXYZRGB>);

    // ** Downsample
    preprocess(cloud_in, cloud_preproc);

    // ** Remove dominant planes
    removePlanes(cloud_preproc, cloud_no_planes);

    // ** Get biggest resulting cluster
    extractCluster(cloud_no_planes, cloud_out);
}

void Object_Extractor_3D::preprocess(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud_in,
                                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr      &cloud_out)
{
    ROS_INFO("Preprocessing...");

    // ** Voxel Grid filter (downsampling)
    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    vg.setInputCloud (cloud_in);
    vg.setLeafSize (CLOUD_RES, CLOUD_RES, CLOUD_RES);
    vg.filter (*cloud_out);


    // ** Pass-through filter
//    pcl::PassThrough<pcl::PointXYZ> pt_filter;
//    pt_filter.setInputCloud (cloud_out);
//    pt_filter.setFilterFieldName ("z");
//    pt_filter.setFilterLimits (0.0, PCL_MAX_Z);
//    pt_filter.filter (*cloud_out);
//    pt_filter.setInputCloud (cloud_out);
//    pt_filter.setFilterFieldName ("x");
//    pt_filter.setFilterLimits (-PCL_MAX_X/2.0, PCL_MAX_X/2.0);
//    pt_filter.filter (*cloud_out);


}

void Object_Extractor_3D::removePlanes(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud_in,
                                          pcl::PointCloud<pcl::PointXYZRGB>::Ptr      &cloud_out)
{
    ROS_INFO("Removing planes...");

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>),
            cloud_p (new pcl::PointCloud<pcl::PointXYZRGB>), cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::copyPointCloud(*cloud_in, *cloud_filtered);
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

    int i = 0, nr_points = (int) cloud_filtered->points.size ();
    // While 30% of the original cloud is still there
    while (cloud_filtered->points.size () > 0.3 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloud_filtered);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the inliers
        extract.setInputCloud (cloud_filtered);
        extract.setIndices (inliers);
        extract.setNegative (false);
        extract.filter (*cloud_p);
        std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

        // Create the filtering object
        extract.setNegative (true);
        extract.filter (*cloud_f);
        cloud_filtered.swap (cloud_f);
        i++;
    }
    pcl::copyPointCloud(*cloud_filtered, *cloud_out);
}

void Object_Extractor_3D::extractCluster(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud_in,
                                            pcl::PointCloud<pcl::PointXYZRGB>::Ptr      &cloud_out)
{
    ROS_INFO("Extracting cluster...");
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud (cloud_in);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_in);
    ec.extract (cluster_indices);

    double max_size = 0;
    double max_idx = -1;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters;
    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
        {
            cloud_cluster->points.push_back (cloud_in->points[*pit]); //*
        }

        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        clusters.push_back(cloud_cluster);

        if(cloud_cluster->points.size() > max_size)
        {
            max_size = cloud_cluster->points.size();
            max_idx = j;
        }
        ++j;
    }

    pcl::copyPointCloud(*(clusters[max_idx]), *cloud_out);
}
