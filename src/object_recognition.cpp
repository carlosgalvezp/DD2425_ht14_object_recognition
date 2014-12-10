#include <object_recognition/object_recognition.h>

Object_Recognition::Object_Recognition()
{
}

Object_Recognition::Object_Recognition(const ros::Publisher &pcl_pub)
    : pcl_pub_(pcl_pub)
{
    object_names = {OBJECT_NAME_RED_CUBE,
                    OBJECT_NAME_BLUE_CUBE,
                    OBJECT_NAME_GREEN_CUBE,
                    OBJECT_NAME_YELLOW_CUBE,
                    OBJECT_NAME_YELLOW_BALL,
                    OBJECT_NAME_RED_BALL,
                    OBJECT_NAME_GREEN_CYLINDER,
                    OBJECT_NAME_BLUE_TRIANGLE,
                    OBJECT_NAME_PURPLE_CROSS,
                    OBJECT_NAME_PATRIC,
                    OBJECT_NAME_UNKNOWN};
}


bool Object_Recognition::classify(const cv::Mat &bgr_img, const cv::Mat &depth_img, const cv::Mat &color_mask,
                                  const Eigen::Matrix4f &t_cam_to_robot, std::string &result)
{
    return classifyCarlos(bgr_img, depth_img, color_mask, t_cam_to_robot, result);
//    return visionRyan(bgr_img);
}


bool Object_Recognition::classifyCarlos(const cv::Mat &bgr_img, const cv::Mat &depth_img, const cv::Mat &color_mask,
                                        const Eigen::Matrix4f &t_cam_to_robot, std::string &result)
{
    // ** Mask rgb and depth img
    cv::Mat rgb_masked, depth_masked;
    bgr_img.copyTo(rgb_masked, color_mask);
    depth_img.copyTo(depth_masked, color_mask);

    // ** Build object point cloud and transform into robot frame
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    PCL_Utils::buildPointCloud(rgb_masked, depth_masked, object_cloud, SCALE_FACTOR);
    pcl::transformPointCloud(*object_cloud, *object_cloud, t_cam_to_robot);
    object_cloud->header.frame_id = COORD_FRAME_ROBOT;
    pcl_pub_.publish(object_cloud);

    // ** Call 3D recognition
    std::vector<double> shape_probabilities(RAS_Names::MODELS_3D_NAMES.size()+1);
    classifier3D_.recognize_vfh(object_cloud, shape_probabilities);

    // Compute p(others) as 1.0 - n_3D_points / N_color_points (small number when concave)
    double p_others = 1.0 - (double) object_cloud->size() / (double)(cv::countNonZero(color_mask)*(SCALE_FACTOR*SCALE_FACTOR));
    std::cout << "P OTHERS " << p_others<<std::endl;
    std::cout << "P CUBE " << shape_probabilities[0] <<std::endl;
    std::cout << "P BALL " << shape_probabilities[1]<<std::endl;
    shape_probabilities[shape_probabilities.size()-1] = p_others;

    // Normalize
    RAS_Utils::normalize_probabilities(shape_probabilities);

    // ** Call Color Bayes Classifier
    std::vector<int> color_classes{COLOR_RED, COLOR_GREEN, COLOR_BLUE, COLOR_YELLOW, COLOR_PURPLE, COLOR_LIGHT_GREEN, COLOR_ORANGE};
    std::vector<double> color_probabilities(color_classes.size());

    cv::Mat hsv_img;
    cv::cvtColor(bgr_img, hsv_img, CV_BGR2HSV);

    color_classifier_.classify(hsv_img, color_mask, color_classes, color_probabilities);

//    // ** Compute probabilities for every object
    std::vector<double> object_probabilities(10);
    object_probabilities[OBJECT_IDX_RED_CUBE]       = shape_probabilities[SHAPE_3D_CUBE]  * color_probabilities[COLOR_RED];
    object_probabilities[OBJECT_IDX_BLUE_CUBE]      = shape_probabilities[SHAPE_3D_CUBE]  * color_probabilities[COLOR_BLUE];
    object_probabilities[OBJECT_IDX_GREEN_CUBE]     = shape_probabilities[SHAPE_3D_CUBE]  * color_probabilities[COLOR_GREEN];
    object_probabilities[OBJECT_IDX_YELLOW_CUBE]    = shape_probabilities[SHAPE_3D_CUBE]  * color_probabilities[COLOR_YELLOW];

    object_probabilities[OBJECT_IDX_YELLOW_BALL]    = shape_probabilities[SHAPE_3D_BALL]  * color_probabilities[COLOR_YELLOW];
    object_probabilities[OBJECT_IDX_RED_BALL]       = shape_probabilities[SHAPE_3D_BALL]  * color_probabilities[COLOR_RED];

    object_probabilities[OBJECT_IDX_GREEN_CYLINDER] = shape_probabilities[SHAPE_3D_OTHER] * color_probabilities[COLOR_LIGHT_GREEN];
    object_probabilities[OBJECT_IDX_BLUE_TRIANGLE]  = shape_probabilities[SHAPE_3D_OTHER] * color_probabilities[COLOR_BLUE];
    object_probabilities[OBJECT_IDX_PURPLE_CROSS]   = shape_probabilities[SHAPE_3D_OTHER] * color_probabilities[COLOR_PURPLE];
    object_probabilities[OBJECT_IDX_PATRIC]         = shape_probabilities[SHAPE_3D_OTHER] * color_probabilities[COLOR_ORANGE];

    // ** Pick the most likely
    double max_p = 0.0;
    for(std::size_t i = 0; i < object_probabilities.size(); ++i)
    {
        double p = object_probabilities[i];
        if(p > max_p)
        {
            max_p = p;
            result = this->object_names[i];
        }
    }

    std::cout << "CLASSIFICATION: " << result<<std::endl;
    return true;
}
