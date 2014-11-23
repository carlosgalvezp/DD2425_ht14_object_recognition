#include <object_recognition/object_recognition.h>

Object_Recognition::Object_Recognition()
{
}

//Decision tree combining 3D and 2D information:

//    1) Is the object concave?
//       Yes -> run Bayes color classifier for green, blue, purple or red (orange)
//       No ->
//          2) Run circle detector. Is there a circle?
//            Yes -> It's a sphere, run color classifier for red and yellow
//            No  -> It's a cube, run color classifier for red, green, blue and yellow

std::string Object_Recognition::classify(const cv::Mat &rgb_img, const cv::Mat &depth_img, const cv::Mat &mask_img, bool is_concave)
{
    ROS_INFO("Classifying");
    std::vector<int> colors;
    int c;
    std::string result;
    cv::Mat hsv_img;
    cv::cvtColor(rgb_img, hsv_img, CV_BGR2HSV);

    if(is_concave)
    {
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
        default:
            result = OBJECT_UNKNOWN;
            break;
        }
    }
    else
    {
        if(shape_detector_.circle_detection(rgb_img, mask_img, false))
        {
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
            default:
                result = OBJECT_UNKNOWN;
                break;
            }
        }
        else
        {
            colors = {COLOR_RED,COLOR_GREEN, COLOR_BLUE, COLOR_YELLOW};
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
            default:
                result = OBJECT_UNKNOWN;
                break;
            }
        }
    }
    return result;
}
