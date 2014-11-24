#ifndef OBJECT_RECOGNITION_H
#define OBJECT_RECOGNITION_H

#include <object_recognition/color_bayes_classifier.h>
#include <object_recognition/object_recognition_3d.h>
#include <ras_utils/pcl_utils.h>
#include <ras_utils/ras_utils.h>
#include <object_recognition/shape_detector_2d.h>
// OpenCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// PCL
#include <pcl/common/transforms.h>

// Eigen
#include <Eigen/Core>

#define N_CLASSIFICATIONS 10 // Number of classifications

/**
 * @brief Combines 2D and 3D object recognition to achieve the final classifier
 */

class Object_Recognition
{
public:
    Object_Recognition();

    bool classify(const cv::Mat &rgb_img, const cv::Mat &rgb_cropped, bool is_concave, const cv::Mat &color_mask, std::string &result);
private:
    Color_Bayes_Classifier color_classifier_;
    Object_Recognition_3D classifier3D_;
    Shape_Detector_2D shape_detector_;

    bool is_concave(const cv::Mat &depth_img, const cv::Mat &mask_img);

    std::vector<std::string> classifications;
};

#endif // OBJECT_RECOGNITION_H
