#ifndef OBJECT_RECOGNITION_H
#define OBJECT_RECOGNITION_H

#include <object_recognition/color_bayes_classifier.h>
#include <object_recognition/object_recognition_3d.h>
#include <ras_utils/pcl_utils.h>

// OpenCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// PCL
#include <pcl/common/transforms.h>

// Eigen
#include <Eigen/Core>
/**
 * @brief Combines 2D and 3D object recognition to achieve the final classifier
 */

class Object_Recognition
{
public:
    Object_Recognition();

    std::string classify(const cv::Mat &rgb_img,
                         const cv::Mat &depth_img,
                         const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud,
                         const cv::Mat &mask_img,
                         const Eigen::Matrix4f &t_cam_to_robot, const pcl::PointXYZ &mass_center);
private:
    Color_Bayes_Classifier color_classifier_;
    Object_Recognition_3D classifier3D_;

    bool is_concave(const cv::Mat &depth_img, const cv::Mat &mask_img);
};

#endif // OBJECT_RECOGNITION_H
