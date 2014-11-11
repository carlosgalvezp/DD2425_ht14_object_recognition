#include <object_recognition/shape_detector_2d.h>
Shape_Detector_2D::Shape_Detector_2D()
{
}

bool Shape_Detector_2D::circle_detection(const cv::Mat &rgb_img)
{

/////////////////////////////////////
//      PARAMETERS                 //
/////////////////////////////////////

  //parameters in sobel function for edge detection
  int scale = 3;
  int delta = 0;

  //parameters in Hough transform for circle detection
  int ratio = 3;          //invers ratio of resolution, the bigger value, the worse resolution, recommend 3
  int edge_param = 300;   //threshhold for converting edged image to binary, recommend 300
  int center_param = 200; //threshhold for detection of a circle, recommmend 200
  int radius_min = 40;    //red ball radius betwwen 40-75
  int radium_max = 75;    // 0,0 means no limit


//////////////////////////////////////////////////////////
//      1) Load image, blur, convert to grayscale       //
//////////////////////////////////////////////////////////

  /// Load an image
  cv::Mat src = rgb_img;

  /// Blur image
  cv::GaussianBlur( src, src, cv::Size(3,3), 3, 3, cv::BORDER_DEFAULT );  //Size Size(9,9), 2, 2,
//  namedWindow("Win2", CV_WINDOW_AUTOSIZE);
//  imshow("Win2", src);
  //waitKey(0);

  /// Convert it to gray
  cv::Mat src_gray;
  cv::cvtColor( src, src_gray, CV_RGB2GRAY );

//  namedWindow("Win3", CV_WINDOW_AUTOSIZE);
//  imshow("Win3", src_gray);
  //waitKey(0);


/////////////////////////////////////////////////////////////
//      2) Find edges in the image using Sobel method      //
/////////////////////////////////////////////////////////////

  /// Generate grad_x and grad_y
  cv::Mat grad_x, grad_y;
  cv::Mat abs_grad_x, abs_grad_y;

  /// Gradient X
  //Scharr( src_gray, grad_x, ddepth, 1, 0, scale, delta, BORDER_DEFAULT );
  cv::Sobel( src_gray, grad_x, CV_16S, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT );
  cv::convertScaleAbs( grad_x, abs_grad_x );

  /// Gradient Y
  //Scharr( src_gray, grad_y, ddepth, 0, 1, scale, delta, BORDER_DEFAULT );
  cv::Sobel( src_gray, grad_y, CV_16S, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT );
  cv::convertScaleAbs( grad_y, abs_grad_y );

  /// Total Gradient (approximate)
  cv::Mat grad;
  cv::addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad );

//  namedWindow("Win4", CV_WINDOW_AUTOSIZE);
//  imshow("Win4", grad);
  //waitKey(0);


//  Mat binary;   int grad_low=150;  int grad_high=255;
//  inRange(grad, grad_low, grad_high, binary);
//  namedWindow("Win5", CV_WINDOW_AUTOSIZE);
//  imshow("Win5", binary);
//  waitKey(0);

///////////////////////////////////////////////////
//      3) Hough transform the egde image        //
///////////////////////////////////////////////////

    std::vector<cv::Vec3f> circles;

     /// Apply the Hough Transform to find the circles

     cv::HoughCircles( grad, circles, CV_HOUGH_GRADIENT, ratio, grad.rows/8, edge_param, center_param, radius_min, radium_max );

//     ///Draw the circles detected
//     for( std::size_t i = 0; i < circles.size(); i++ )
//     {
//         cv::Point center(cv::cvRound(circles[i][0]), cvRound(circles[i][1]));
//         int radius = cvRound(circles[i][2]);

//         // circle center
//         circle( src, center, 3, Scalar(255,255,255), -1, 8, 0 );
//         cout<< center <<endl;
//         // circle outline
//         circle( src, center, radius, Scalar(255,255,255), 3, 8, 0 );
//      }

     /// Show your results
//     namedWindow( "Hough Circle Transform Demo", CV_WINDOW_AUTOSIZE );
//     imshow( "Hough Circle Transform Demo", src );

//     gettimeofday(&t2, NULL);
//     std::cout << "Time: "<<(t2.tv_sec / t1.tv_sec)*1000.0 +
//                  (t2.tv_usec - t1.tv_usec)/1000.0 <<" ms"<< std::endl;

//     waitKey(0);

  return circles.size() > 0;
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

        int thresh = 600, N = 1;  //initial canny threshold to find lines, between 200-600 works best, N is iteration times with small change in threshold value
        double cosinethreshold=0.3; //fight against not exact 90 degrees intersection, 0.1 means almost 90degree intersections.

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

                    if( approx.size() == 4 && fabs(cv::contourArea(cv::Mat(approx))) > 1000 && cv::isContourConvex(cv::Mat(approx)) )
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
