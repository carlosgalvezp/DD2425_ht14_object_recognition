#include <object_recognition/color_bayes_classifier.h>
#include "ros/ros.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
void test1();
void test2(const char* path);

int main(int argc, char* argv[])
{
//    test1();
    test2(argv[1]);
    return 0;
}

void test1()
{
    Color_Bayes_Classifier classifier;
    ros::WallTime t1 = ros::WallTime::now();
    int color = classifier.classify(116,190);
    std::cout << "Time: "<<RAS_Utils::time_diff_ns(t1, ros::WallTime::now())<<" ns"<<std::endl;
    ROS_INFO("The color is: %d",color);
}

void test2(const char *path)
{
    std::cout << "Test 2"<<std::endl;
    Color_Bayes_Classifier classifier;

    // ** Read image and convert to HSV
    cv::Mat img = cv::imread(path);
    cv::imshow("Input image", img);
    cv::cvtColor(img, img, CV_RGB2HSV);
    std::vector<cv::Mat> img_channels;
    cv::split(img, img_channels);

    std::vector<cv::Mat> masks;
    for(unsigned int i = 0; i < 7; ++i)
        masks.push_back(cv::Mat::zeros(img.rows, img.cols, CV_8UC1));

    // ** Classify
    for(unsigned int i=0; i < img.rows; ++i)
    {
        for(unsigned int j=0; j< img.cols;++j)
        {
            const cv::Mat &h_mat = img_channels[0];
            const cv::Mat &s_mat = img_channels[1];

            double h = h_mat.at<uint8_t>(i,j);
            double s = s_mat.at<uint8_t>(i,j);

            int c = classifier.classify(h,s);
            if(c != -1)
                masks[c].at<uint8_t>(i,j) = 255;
        }
    }

    // ** Display result
    for(std::size_t i = 0; i < masks.size(); ++i)
    {
        std::stringstream ss;
        ss << "Mask "<<i;
        imshow(ss.str(), masks[i]);
    }
    cv::waitKey();
}
