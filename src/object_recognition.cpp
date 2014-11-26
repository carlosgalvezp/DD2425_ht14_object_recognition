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

bool Object_Recognition::classify(const cv::Mat &rgb_img, const cv::Mat &rgb_cropped, bool is_concave, const cv::Mat &color_mask, std::string &result)
{
    std::vector<int> colors;
    int c;
    std::string result_tmp;
    cv::Mat hsv_img;
    cv::cvtColor(rgb_img, hsv_img, CV_BGR2HSV);

    if(is_concave)
    {
        colors = {COLOR_RED,COLOR_GREEN,COLOR_BLUE,COLOR_PURPLE};
        c = color_classifier_.classify(hsv_img, color_mask, colors);

        switch(c)
        {
        case COLOR_RED:
            result_tmp = OBJECT_PATRIC;
            break;
        case COLOR_GREEN:
            result_tmp = OBJECT_GREEN_CYLINDER;
            break;
        case COLOR_BLUE:
            result_tmp = OBJECT_BLUE_TRIANGLE;
            break;
        case COLOR_PURPLE:
            result_tmp = OBJECT_PURPLE_CROSS;
            break;
        default:
            result_tmp = OBJECT_UNKNOWN;
            break;
        }
    }
    else
    {
        colors = {COLOR_RED,COLOR_GREEN,COLOR_BLUE,COLOR_YELLOW};
        c = color_classifier_.classify(hsv_img, color_mask, colors);

        if(c == COLOR_RED || c == COLOR_YELLOW)
        {
            if(shape_detector_.circle_detection(rgb_cropped, false)) // ** Ball
            {
                switch(c)
                {
                    case COLOR_RED:
                        result_tmp = OBJECT_RED_BALL;
                        break;
                    case COLOR_YELLOW:
                        result_tmp = OBJECT_YELLOW_BALL;
                        break;
                    default:
                        result_tmp = OBJECT_UNKNOWN;
                        break;
                }
            }
            else
            {
                switch(c)
                {
                    case COLOR_RED:
                        result_tmp = OBJECT_RED_CUBE;
                        break;
                    case COLOR_YELLOW:
                        result_tmp = OBJECT_YELLOW_CUBE;
                        break;
                    default:
                        result_tmp = OBJECT_UNKNOWN;
                        break;
                }
            }
        }
        else
        {
            switch(c)
            {
                case COLOR_GREEN:
                    result_tmp = OBJECT_GREEN_CUBE;
                    break;
                case COLOR_BLUE:
                    result_tmp = OBJECT_BLUE_CUBE;
                    break;
                default:
                    result_tmp = OBJECT_UNKNOWN;
                    break;
            }
        }
    }
    if(classifications.size() < N_CLASSIFICATIONS)
    {
        classifications.push_back(result_tmp);
        return false;
    }
    else
    {
        result = RAS_Utils::get_most_repeated<std::string>(classifications);
        classifications.clear();
        return true;
    }
}
