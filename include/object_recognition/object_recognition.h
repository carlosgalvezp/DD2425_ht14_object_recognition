#ifndef OBJECT_RECOGNITION_H
#define OBJECT_RECOGNITION_H

#include <ros/ros.h>
#include <object_recognition/color_bayes_classifier.h>
#include <object_recognition/object_recognition_3d.h>
#include <ras_utils/pcl_utils.h>
#include <ras_utils/ras_utils.h>
#include <object_recognition/shape_detector_2d.h>
#include <object_recognition/ryan_vision.h>

// OpenCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// PCL
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

// Eigen
#include <Eigen/Core>

#define N_CLASSIFICATIONS 10 // Number of classifications
#define SCALE_FACTOR      0.25
/**
 * @brief Combines 2D and 3D object recognition to achieve the final classifier
 */

class Object_Recognition
{
public:
    Object_Recognition();
    Object_Recognition(const ros::Publisher &pcl_pub);

    bool classify(const cv::Mat &rgb_img, const cv::Mat &depth_img, const cv::Mat &color_mask,
                  const Eigen::Matrix4f &t_cam_to_robot, std::string &result);
private:
    Color_Bayes_Classifier color_classifier_;
    Object_Recognition_3D classifier3D_;
//    Shape_Detector_2D shape_detector_;
    std::vector<std::string> classifications;

    ros::Publisher pcl_pub_;

    bool is_concave(const cv::Mat &depth_img, const cv::Mat &mask_img);

    bool classifyCarlos(const cv::Mat &rgb_img, const cv::Mat &depth_img, const cv::Mat &color_mask,
                        const Eigen::Matrix4f &t_cam_to_robot, std::string &result);

    std::vector<std::string> object_names;
};

#endif // OBJECT_RECOGNITION_H
