#include <object_recognition/object_recognition.h>

Object_Recognition::Object_Recognition()
{
}

std::string Object_Recognition::classify(const cv::Mat &rgb_img, const cv::Mat &depth_img, const cv::Mat &mask_img)
{
    // ** Build Point Cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    PCL_Utils::buildPointCloud(rgb_img, depth_img, cloud);

    // ** Create HSV image
    cv::Mat hsv_img;
    cv::cvtColor(rgb_img, hsv_img, CV_BGR2HSV);

    // ** Run 3D classifier
    int main_class = classifier3D_.recognize(cloud);
    std::string result;
    std::vector<int> colors;
    int c;
    switch(main_class)
    {
        //** Cube
        case SHAPE_3D_CUBE:
            colors = {COLOR_RED,COLOR_GREEN,COLOR_BLUE,COLOR_YELLOW};
            c = color_classifier_.classify(hsv_img, mask_img, colors);

            switch(c)
            {
                case COLOR_RED:
                    result = OBJECT_RED_CUBE;
                    break;
                case COLOR_GREEN:
                    result = OBJECT_GREEN_CUBE;
                    break;
                case COLOR_BLUE:
                    result = OBJECT_BLUE_CUBE;
                    break;
                case COLOR_YELLOW:
                    result = OBJECT_YELLOW_CUBE;
                    break;
            }

            break;

        // ** Ball
        case SHAPE_3D_BALL:
            colors = {COLOR_RED,COLOR_YELLOW};
            c = color_classifier_.classify(hsv_img, mask_img, colors);

            switch(c)
            {
            case COLOR_RED:
                result = OBJECT_RED_BALL;
                break;
            case COLOR_YELLOW:
                result = OBJECT_YELLOW_BALL;
                break;
            }
            break;
        // ** Other
        default:
            colors = {COLOR_RED,COLOR_GREEN,COLOR_BLUE,COLOR_PURPLE};
            c = color_classifier_.classify(hsv_img, mask_img, colors);

            switch(c)
            {
                case COLOR_RED:
                    result = OBJECT_PATRIC;
                    break;
                case COLOR_GREEN:
                    result = OBJECT_GREEN_CYLINDER;
                    break;
                case COLOR_BLUE:
                    result = OBJECT_BLUE_TRIANGLE;
                    break;
                case COLOR_PURPLE:
                    result = OBJECT_PURPLE_CROSS;
                    break;
            }
            break;
    }
    return result;
}
