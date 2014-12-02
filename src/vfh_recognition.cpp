#include <object_recognition/vfh_recognition.h>

VFH_Recognition::VFH_Recognition()
{
}

void VFH_Recognition::computeDescriptor(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud_in,
                                         pcl::PointCloud<pcl::VFHSignature308>::Ptr &descriptor)
{

    // ** Downsample
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in_downsampled(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    vg.setInputCloud (cloud_in);
    vg.setLeafSize (CLOUD_RES, CLOUD_RES, CLOUD_RES);
    vg.filter (*cloud_in_downsampled);

    // ** Compute normals
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimation;
    normal_estimation.setSearchMethod
        (pcl::search::KdTree<pcl::PointXYZRGB>::Ptr (new pcl::search::KdTree<pcl::PointXYZRGB>));
    normal_estimation.setRadiusSearch (20 * CLOUD_RES); // Tune this
    normal_estimation.setInputCloud (cloud_in_downsampled);
    normal_estimation.compute (*normals);


    // ** VFH
    pcl::VFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::VFHSignature308> vfh;
    vfh.setInputCloud (cloud_in_downsampled);
    vfh.setInputNormals (normals);

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    vfh.setSearchMethod (tree);

    vfh.compute (*descriptor);
}

double VFH_Recognition::matchDescriptor(const pcl::PointCloud<pcl::VFHSignature308>::ConstPtr &d1,
                                        const pcl::PointCloud<pcl::VFHSignature308>::ConstPtr &d2)
{
    pcl::VFHSignature308 desc1 = d1->points[0];
    pcl::VFHSignature308 desc2 = d2->points[0];

    cv::Mat m1 = cv::Mat(DESCRIPTOR_SIZE,1, CV_32FC1, &desc1.histogram);
    cv::Mat m2 = cv::Mat(DESCRIPTOR_SIZE,1, CV_32FC1, &desc2.histogram);
    return cv::compareHist(m1, m2, CV_COMP_CORREL);
}
