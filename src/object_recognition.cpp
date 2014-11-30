#include <object_recognition/object_recognition.h>

Object_Recognition::Object_Recognition()
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

Object_Recognition::Object_Recognition(const ros::Publisher &pcl_pub, const Eigen::Matrix4f &t_cam_to_robot)
    : pcl_pub_(pcl_pub), t_cam_to_robot_(t_cam_to_robot){}


bool Object_Recognition::classify(const cv::Mat &bgr_img, const cv::Mat &depth_img, const cv::Mat &color_mask, std::string &result)
{
    classifyCarlos(bgr_img, depth_img, color_mask, result);
    return false;
}


bool Object_Recognition::classifyCarlos(const cv::Mat &bgr_img, const cv::Mat &depth_img, const cv::Mat &color_mask, std::string &result)
{
    // ** Mask rgb and depth img
    cv::Mat rgb_masked, depth_masked;
    bgr_img.copyTo(rgb_masked, color_mask);
    depth_img.copyTo(depth_masked, color_mask);

    // ** Build object point cloud and transform into robot frame
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    PCL_Utils::buildPointCloud(rgb_masked, depth_masked, object_cloud, 0.25);
    pcl::transformPointCloud(*object_cloud, *object_cloud, t_cam_to_robot_);
    pcl_pub_.publish(object_cloud);
//    pcl::io::savePCDFile("/home/carlos/3d_data/test/my_test.pcd", *object_cloud);
//    std::cout << "SAVED FILE"<<std::endl;
//    cv::namedWindow("");
//    cv::waitKey();

    // ** Call 3D recognition
    std::vector<double> shape_probabilities;
    classifier3D_.recognize(object_cloud, shape_probabilities);

    // ** Call Color Bayes Classifier
    std::vector<double> color_probabilities;
    std::vector<int> color_classes{COLOR_RED, COLOR_GREEN, COLOR_BLUE, COLOR_YELLOW, COLOR_PURPLE};

    cv::Mat hsv_img;
    cv::cvtColor(bgr_img, hsv_img, CV_BGR2HSV);

    color_classifier_.classify(hsv_img, color_mask, color_classes, color_probabilities);

//    // ** Compute probabilities for every object
//    std::vector<double> object_probabilities;
//    object_probabilities[OBJECT_IDX_RED_CUBE]       = shape_probabilities[SHAPE_3D_CUBE] * color_probabilities[COLOR_RED];
//    object_probabilities[OBJECT_IDX_BLUE_CUBE]      = shape_probabilities[SHAPE_3D_CUBE] * color_probabilities[COLOR_BLUE];
//    object_probabilities[OBJECT_IDX_GREEN_CUBE]     = shape_probabilities[SHAPE_3D_CUBE] * color_probabilities[COLOR_GREEN];
//    object_probabilities[OBJECT_IDX_YELLOW_CUBE]    = shape_probabilities[SHAPE_3D_CUBE] * color_probabilities[COLOR_YELLOW];

//    object_probabilities[OBJECT_IDX_YELLOW_BALL]    = shape_probabilities[SHAPE_3D_BALL] * color_probabilities[COLOR_YELLOW];
//    object_probabilities[OBJECT_IDX_RED_BALL]       = shape_probabilities[SHAPE_3D_BALL] * color_probabilities[COLOR_RED];

//    object_probabilities[OBJECT_IDX_GREEN_CYLINDER] = shape_probabilities[SHAPE_3D_OTHER] * color_probabilities[COLOR_GREEN];
//    object_probabilities[OBJECT_IDX_BLUE_TRIANGLE]  = shape_probabilities[SHAPE_3D_OTHER] * color_probabilities[COLOR_BLUE];
//    object_probabilities[OBJECT_IDX_PURPLE_CROSS]   = shape_probabilities[SHAPE_3D_OTHER] * color_probabilities[COLOR_PURPLE];
//    object_probabilities[OBJECT_IDX_PATRIC]         = shape_probabilities[SHAPE_3D_OTHER] * color_probabilities[COLOR_RED];

//    // ** Pick the most likely
//    double max_p = 0.0;
//    for(std::size_t i = 0; i < object_probabilities.size(); ++i)
//    {
//        double p = object_probabilities[i];
//        if(p > max_p)
//        {
//            max_p = p;
//            result = this->object_names[i];
//        }
//    }
}
