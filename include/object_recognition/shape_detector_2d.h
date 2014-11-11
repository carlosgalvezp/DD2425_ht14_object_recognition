#ifndef SHAPE_DETECTOR_2D_H
#define SHAPE_DETECTOR_2D_H

// STL
#include <iostream>
#include <ctime>
#include <vector>

// OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/core/core.hpp"

#include <ras_utils/ras_utils.h>

class Shape_Detector_2D
{
public:
    Shape_Detector_2D();

    bool circle_detection(const cv::Mat &rgb_img);
    bool square_detection(const cv::Mat &rgb_img);

private:
    double angle( cv::Point pt1, cv::Point pt2, cv::Point pt0 );
    void findSquares( const cv::Mat& image, std::vector<std::vector<cv::Point> >& squares );
    void drawSquares( cv::Mat& image, const std::vector<std::vector<cv::Point> >& squares );

};

#endif // SHAPE_DETECTOR_2D_H
