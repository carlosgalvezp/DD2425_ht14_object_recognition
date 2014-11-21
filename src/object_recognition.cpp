#include <object_recognition/object_recognition.h>

Object_Recognition::Object_Recognition()
{
}

std::string Object_Recognition::classify(const cv::Mat &hsv_img, const cv::Mat &depth_img, const cv::Mat &mask_img)
{

//        int main_class = classifier3D_.recognize(cloud, mass_center);
//        ROS_INFO("3D Main class %d",main_class);
//        std::string result;
        std::vector<int> colors;
        int c;
        std::string result;
        if(is_concave(depth_img, mask_img))
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
            }
        }
        else
        {
            result = "Convex object";
        }
        std::cout << "RESULT "<<result<<std::endl;
        return result;
}

bool Object_Recognition::is_concave(const cv::Mat &depth_img, const cv::Mat &mask_img)
{
    cv::Mat mask_img2;
    int size = 5; // This depends on scaling factor
    cv::Mat element = getStructuringElement(cv::MORPH_RECT,cv::Size(2*size+1, 2*size+1),cv::Point( size, size) );
    cv::erode(mask_img, mask_img2, element);

    double nanPoints=0, normalPoints=0, totalPoints=0;
    for(unsigned int i = 0; i < depth_img.rows; i+=STEP)
    {
        for(unsigned int j = 0; j < depth_img.cols; j+=STEP)
        {
            if(mask_img2.at<uint8_t>(i,j) != 0)
            {
                float v = depth_img.at<float>(i,j);
                if(!isnan(v) && v!=0)
                    ++normalPoints;
                else
                    ++nanPoints;

                ++totalPoints;
            }
        }
    }
    double ratio = nanPoints/totalPoints;
    std::cout << "RATIO: "<<ratio <<std::endl;
    return ratio > 0.7;
}
