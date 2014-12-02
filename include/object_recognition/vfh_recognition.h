#ifndef VFH_RECOGNITION_H
#define VFH_RECOGNITION_H

#include <ras_utils/ras_utils.h>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/vfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/voxel_grid.h>

// OpenCV
#include <opencv2/imgproc/imgproc.hpp>

#define CLOUD_RES 0.001
#define DESCRIPTOR_SIZE 308

class VFH_Recognition
{
public:
    VFH_Recognition();

    /**
     * @brief Computes the VFH descriptor for the given he input cloud
     * @param cloud_in
     * @param descriptor
     */
    void computeDescriptor(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud_in,
                            pcl::PointCloud<pcl::VFHSignature308>::Ptr &descriptor);

    /**
     * @brief Matches the given descriptor to a database
     * @param descriptor
     * @return number between 0.0 and 1.0. 1.0 means perfect match
     */
    double matchDescriptor(const pcl::PointCloud<pcl::VFHSignature308>::ConstPtr &d1,
                           const pcl::PointCloud<pcl::VFHSignature308>::ConstPtr &d2);

};

#endif // VFH_RECOGNITION_H
