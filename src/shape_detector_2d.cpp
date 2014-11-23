#include <object_recognition/shape_detector_2d.h>


// ** Shape detector parameters
#define CIRCLE_DETECTOR_CANNY_TH    100
#define CIRCLE_DETECTOR_ACCUM_TH    30
#define CIRCLE_DETECTOR_MIN_R       25
#define CIRCLE_DETECTOR_MAX_R       45

Shape_Detector_2D::Shape_Detector_2D()
{
}

bool Shape_Detector_2D::circle_detection(const cv::Mat &bgr_img, const cv::Mat &color_mask, bool show)
{
    // ** Convert to gray
    cv::Mat src_draw;
    bgr_img.copyTo(src_draw);
    cv::Mat src_gray;
    cv::cvtColor( bgr_img, src_gray, CV_BGR2GRAY );


    // ** Sharpen image
    cv::Mat laplacian, edge_image;
//    cv::GaussianBlur(src_gray, tmp, cv::Size(5,5), 5);
    cv::Laplacian(src_gray, laplacian, CV_16S,3);
    convertScaleAbs( laplacian, edge_image);
    cv::imshow("Laplacian", edge_image);
    cv::imshow("Source gray", src_gray);
    cv::addWeighted(src_gray, 1.5, edge_image, -0.5, 0, src_gray);
    cv::imshow("After gray", src_gray);
    cv::waitKey(1);
    // ** Run Hough transform
    std::vector<cv::Vec3f> circles;
    cv::HoughCircles( src_gray, circles, CV_HOUGH_GRADIENT, 1, src_gray.rows/8, CIRCLE_DETECTOR_CANNY_TH,
                                                                                CIRCLE_DETECTOR_ACCUM_TH,
                                                                                CIRCLE_DETECTOR_MIN_R,
                                                                                CIRCLE_DETECTOR_MAX_R);

    // clone the colour, input image for displaying purposes
    bool result = false;
    for( size_t i = 0; i < circles.size(); i++ )
    {
        if(show)
        {
            ROS_INFO("CIRCLE");
           cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
           int radius = cvRound(circles[i][2]);
           // circle center
           cv::circle( src_draw, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
           // circle outline
           cv::circle( src_draw, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );
           cv::imshow("Circles", src_draw);
           cv::waitKey(1);
        }
        cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));

        if(inROI(center, color_mask)) // The mask is completely filled
            result = true;
    }
    return result;
}

bool Shape_Detector_2D::inROI(const cv::Point &p, const cv::Mat &mask)
{
    return (mask.at<uint8_t>(p.y, p.x) != 0);
}


bool Shape_Detector_2D::square_detection(const cv::Mat &rgb_img)
{
    ///////////////////////////////////////
    ////      HOW THIS CODE WORKS        //
    ///////////////////////////////////////

    // 1)use canny method to find lines
    // 2)approximation of lines to make all lines intersect
    // 3)find intersection points and calculate intersection angle
    // 4)find rectangles


    cv::Mat image = rgb_img;

    std::vector<std::vector<cv::Point> > squares;
    findSquares(image, squares);  //call find square function
//    drawSquares(image, squares);  //call draw square function
    return squares.size() > 0;
}

    ///////////////////////////////////////////////////
    //       Intersection points (used later)        //
    ///////////////////////////////////////////////////

    // from pt0->pt1 and from pt0->pt2
    double Shape_Detector_2D::angle(cv::Point pt1, cv::Point pt2, cv::Point pt0 )
    {
        double dx1 = pt1.x - pt0.x;
        double dy1 = pt1.y - pt0.y;
        double dx2 = pt2.x - pt0.x;
        double dy2 = pt2.y - pt0.y;
        return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
    }

    /////////////////////////////////////////////////////////////////
    //   Use canny and findcontour functions to find lines         //
    /////////////////////////////////////////////////////////////////
    void Shape_Detector_2D::findSquares( const cv::Mat& image, std::vector<std::vector<cv::Point> >& squares )
    {
        //////////////////////////////////////
        ////      PARAMETERS TO TUNE        //
        //////////////////////////////////////

        int thresh = 800, N = 3;  //initial canny threshold to find lines, between 600-800 works best, N is iteration times with small change in threshold value
        double cosinethreshold=0.6; //fight against not exact 90 degrees intersection, 0.1 means almost 90degree intersections.

        cv::Mat pyr, timg, gray0(image.size(), CV_8U), gray;
        // down-scale and upscale the image to filter out the noise
        cv::pyrDown(image, pyr, cv::Size(image.cols/2, image.rows/2));
        cv::pyrUp(pyr, timg, image.size());

        // find squares in every color plane of the image
        for( int c = 0; c < 3; c++ )
        {
            int ch[] = {c, 0};
            cv::mixChannels(&timg, 1, &gray0, 1, ch, 1);
            // try several threshold levels
            for( int l = 0; l < N; l++ )
            {
                if( l == 0 )
                {
                    cv::Canny(gray0, gray, 0, thresh, 5);
                    cv::dilate(gray, gray, cv::Mat(), cv::Point(-1,-1));
                }
                else
                {
                    gray = gray0 >= (l+1)*255/N;
                }
                //namedWindow("Win1", CV_WINDOW_AUTOSIZE);imshow("Win1", gray0);waitKey(0);
                //namedWindow("Win1", CV_WINDOW_AUTOSIZE);imshow("Win1", gray);waitKey(0);
                std::vector<std::vector<cv::Point> > contours;
                cv::findContours(gray, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);


                ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                // Approximate the lines to make them intersect, check contours area and intersection angle (cosine value) to identify rectangles //
                ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

                std::vector<cv::Point> approx;
                // test each contour
                for( size_t i = 0; i < contours.size(); i++ )
                {
                    // approximate contour with accuracy proportional
                    // to the contour perimeter
                    cv::approxPolyDP(cv::Mat(contours[i]), approx, cv::arcLength(cv::Mat(contours[i]), true)*0.02, true);
                    // square contours should have 4 vertices after approximation
                    // relatively large area (to filter out noisy contours)
                    // and be convex.
                    // Note: absolute value of an area is used because
                    // area may be positive or negative - in accordance with the
                    // contour orientation

                    if( approx.size() == 4 && fabs(cv::contourArea(cv::Mat(approx))) > 2500 && fabs(cv::contourArea(cv::Mat(approx))) < 4500 && cv::isContourConvex(cv::Mat(approx)) )
                    {
                        double maxCosine = 0;
                        for( int j = 2; j < 5; j++ )
                        {
                            // find the maximum cosine of the angle between joint edges
                            double cosine = fabs(angle(approx[j%4], approx[j-2], approx[j-1]));
                            maxCosine = MAX(maxCosine, cosine);
                        }
                        // if cosines of all angles are small
                        // (all angles are ~90 degree) then write quandrange
                        // vertices to resultant sequence
                        if( maxCosine < cosinethreshold )
                            squares.push_back(approx);
                    }

                }
            }
        }
    }

    void Shape_Detector_2D::drawSquares( cv::Mat& image, const std::vector<std::vector<cv::Point> >& squares )
    {
        for( size_t i = 0; i < squares.size(); i++ )
        {
            const cv::Point* p = &squares[i][0];
            int n = (int)squares[i].size();
            cv::polylines(image, &p, &n, 1, true, cv::Scalar(0,255,0), 1);
        }

        cv::imshow("win1", image);
    }
