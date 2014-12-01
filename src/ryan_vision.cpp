#include <object_recognition/ryan_vision.h>


//SQUARE DETECTION

    ///////////////////////////////////////
    ////      HOW THIS CODE WORKS        //
    ///////////////////////////////////////

    // 1)use canny method to find lines
    // 2)approximation of lines to make all lines intersect
    // 3)find intersection points and calculate intersection angle
    // 4)find rectangles


    //////////////////////////////////////
    ////      PARAMETERS TO TUNE        //
    //////////////////////////////////////

    int thresh = 600, N = 20;  //initial canny old to find lines, between 600-800 works best, N is iteration times with small change in grayscale value
    double cosinethreshold=0.7; //fight against not exact 90 degrees intersection, 0.1 means almost 90degree intersections.
    //Mat image = imread("/home/ras/shape_detector_working/square_detection_working/build/Test/frame0001.png", CV_LOAD_IMAGE_ANYCOLOR);
    //Mat image = imread("/home/ras/shape_detector_working/square_detection_working/build/all_color_squares/frame0011.jpg", CV_LOAD_IMAGE_ANYCOLOR);
    //Mat image = imread("/home/ras/shape_detector_working/square_detection_working/build/vision_data_img.2/blue_triangle/blue_triangle_RGB_2.png", CV_LOAD_IMAGE_ANYCOLOR);


    int contour_pixel_min = 2500; //2500
    int contour_pixel_max = 3500; //4000

    int foundtimes=0;

///////////////////////////////////////////////////
//       Intersection points (used later)        //
///////////////////////////////////////////////////

 //from pt0->pt1 and from pt0->pt2

static double angle( Point pt1, Point pt2, Point pt0 )
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

static void findSquares( const Mat& image, vector<vector<Point> >& squares )
{

    Mat down, up, gray0(image.size(), CV_8U), canny_output; Mat img_contours(image.size(), CV_8U);

    // down-scale and upscale the image to filter out the noise
    pyrDown(image, down, Size(image.cols/2, image.rows/2));
    pyrUp(down, up, image.size());


// find squares in every color plane of the image
for( int c = 0; c < 3; c++ )
        {
    Mat channel[3];
    split(up, channel);

        if (c==0){gray0=channel[0];}
        if (c==1){gray0=channel[1];}
        if (c==2){gray0=channel[2];}

        for( int l = 0; l < N; l++ )
        {
            if( l == 0 )
                {
                Canny(gray0, canny_output, 0, thresh, 5);  //find edges


            }
            else
                {
                canny_output = gray0 >= (l+1)*255/N;  //change intensity values of edges
                dilate(canny_output, canny_output, Mat(), Point(-1,-1),2);
                erode(canny_output, canny_output, Mat(), Point(-1,-1),2);
                //imshow("gray", gray0);
                //waitKey(0);

                //imshow("canny", canny_output);
                //waitKey(0);
            }

        vector<vector<Point> > contours;
        findContours(canny_output, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);

        //cout<<"there are "<<contours.size()<< " contours in this image"<<endl;


        drawContours(img_contours,contours,-1,255,1);
        //imshow("All contours",img_contours);
        //waitKey(0);


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Approximate the lines to make them intersect, check contours area and intersection angle (cosine value) to identify rectangles //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        vector<Point> approx;

        for( int i = 0; i < contours.size(); i++ )  // test each contour
        {

        approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.02, true);// approximate contour with accuracy proportional
                                                                                             // to the contour perimeter, 0,02 works on squares


        // square contours should have 4 vertices after approximation
        // relatively large area (to filter out noisy contours)

        // Note: absolute value of an area is used because
        // area may be positive or negative - in accordance with the
        // contour orientation

        if( approx.size() == 4 && fabs(contourArea(Mat(approx))) > contour_pixel_min && fabs(contourArea(Mat(approx))) < contour_pixel_max  )// and be convex. && isContourConvex(Mat(approx))
        {
            foundtimes++;

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

///////////////////////////////////////////////////////
//      Draws all the squares in the image           //
///////////////////////////////////////////////////////

static void drawSquares( Mat& image, const vector<vector<Point> >& squares )
{
for( size_t i = 0; i < squares.size(); i++ )
    {
    const Point* p = &squares[i][0];
    int n = (int)squares[i].size();
    polylines(image, &p, &n, 1, true, Scalar(0,255,0), 1);

    }


imshow("Result", image);
}

void detectRyan (const cv::Mat &image)
{

    ////////////////////////////////////////////////
    //          How this code work                //
    ////////////////////////////////////////////////

//1) load original image and convert to HSV image with different lightning condition
//2) for each HSV masked image find a farily large contour
//3) check if we have exatcly one contour left and calculate probability of finding an object


    ////////////////////////////////////////////
    //             Parameters                 //
    ////////////////////////////////////////////

        //for detection
        int hue_low, hue_high;
        int hue_low_blue =10, hue_low_red =110, hue_low_green =60, hue_low_purple =140, hue_low_yellow =90;
        int hue_high_blue=30, hue_high_red=130, hue_high_green=80, hue_high_purple=170, hue_high_yellow=100;
        int sat_low=100, sat_high=255, bright_low=100, bright_high=255;

        int purple=0, red=0, green=0, blue=0, yellow=0; //how many times have the algorithm detected a color
        int max_contour_pixles=2000;
        int color_idx=5; //1-purple, 2-red, 3-green, 4-blue, 5-yellow
        int contour_min = 50, contour_max = 200;   //a perfect square is 75


    //Mat image = imread("/home/ras/Brain2/build/yellowball.png", CV_LOAD_IMAGE_ANYCOLOR);
    //Mat image = imread("/home/ras/Brain2/build/redball.png", CV_LOAD_IMAGE_ANYCOLOR);
        cvtColor(image,image,CV_BGR2RGB);

    if (image.empty())
    {
        cout << "Error : Image cant be loaded..!!" << endl;
    }

    //imshow("Input Raw Image", image);
    //waitKey(0);

    for(color_idx = 1; color_idx < 6; color_idx++){

////////////////////////////////////////////////////////////////////////////////////////////////////
//       1) load original image and convert to HSV image with different lightning condition       //
////////////////////////////////////////////////////////////////////////////////////////////////////

          Mat image_modified_lightning;

          for(double i = 1; i < 15; i++){

                image_modified_lightning = image*(i/10+0.7);cout<<"lightning condition: lightX"<< (i/10+0.7)<<endl; //lightning factor 0.8-1.8

                //imshow("Input Image lightning condition", image_modified_lightning);
                //waitKey(0);

                Mat img_hsv;
                cvtColor(image_modified_lightning,img_hsv,CV_RGB2HSV);

/////////////////////////////////////////////////////////////////////////////////////////////
//       2) for each HSV masked image, remove noise then find farily large contours        //
/////////////////////////////////////////////////////////////////////////////////////////////

                Mat img_binary;
                if(color_idx==1){hue_low = hue_low_purple; hue_high = hue_high_purple;}
                if(color_idx==2){hue_low = hue_low_red; hue_high = hue_high_red;}
                if(color_idx==3){hue_low = hue_low_green; hue_high = hue_high_green;}
                if(color_idx==4){hue_low = hue_low_blue; hue_high = hue_high_blue;}
                if(color_idx==5){hue_low = hue_low_yellow; hue_high = hue_high_yellow;}

                inRange(img_hsv, Scalar(hue_low, sat_low, bright_low), Scalar(hue_high, sat_high, bright_high), img_binary);

                erode( img_binary, img_binary, Mat(), Point(-1,-1),2); //remove all single pixels

                Mat img_contour=Mat::zeros(image.rows, image.cols, CV_8UC1);

                vector<vector<Point> > contours;

                findContours(img_binary,contours,CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

                drawContours(img_contour,contours,-1,255,0);  //-1 means draw all contours, 255 means white and 0 means single pixel thickness

                if(contours.size()>0)
                {

                    int sum_of_pixles;

                    sum_of_pixles = ((sum(img_contour)/255)(0)); //calculate amout of pixels in the contours image

                    //if too many pixels, ignore this image
                    if (sum_of_pixles > max_contour_pixles){contours.clear();cout<<"Too many pixels! A very noisy image is removed!"<<endl;}

                    //remove small contours
                        for (vector<vector<Point> >::iterator it = contours.begin(); it!=contours.end(); )
                        {
                            if (it->size() < contour_min)
                            { it=contours.erase(it);
                                //cout << "Removed a small contour" << endl;
                            }
                            else if (it->size() > contour_max)
                            { it=contours.erase


                                        (it);
                                //cout << "Removed a big contour" << endl;
                            }
                            else
                            {++it;}

                        }

                }//end of contours.size()>0

                    //cout <<"Number of contours left after removing too big or too small contours: "<< contours.size() << endl;

                    //Mat img_contour_left = Mat::zeros(image.rows, image.cols, CV_8UC1);
                    //if (contours.size()>0){drawContours(img_contour_left,contours,-1,Scalar(255,255,255),1);}

                    //imshow("Contours left",img_contour_left);
                    //waitKey(0);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//        3)  Check if we have exatcly one contour left and calculate probability of finding an object        //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

                vector<vector<Point> > contours_purple,contours_red,contours_green,contours_blue,contours_yellow;
                if(color_idx==1){contours_purple=contours;}
                if(color_idx==2){contours_red=contours;}
                if(color_idx==3){contours_green=contours;}
                if(color_idx==4){contours_blue=contours;}
                if(color_idx==5){contours_yellow=contours;}


                if (contours_purple.size()==1)
                {purple=purple+1;cout<<"a purple object is found!"<<endl;}
                if (contours_red.size()==1)
                {red=red+1;cout<<"a red object is found!"<<endl;}
                if (contours_blue.size()==1)
                {blue=blue+1;cout<<"a blue object is found!"<<endl;}
                if (contours_green.size()==1)
                {green=green+1;cout<<"a green object is found!"<<endl;}
                if (contours_yellow.size()==1)
                {yellow=yellow+1;cout<<"a yellow object is found!"<<endl;}

                //cout << "Done"<< endl;
                //waitKey(0);
                destroyAllWindows();

            }//end of one lighting condition

        }//end of one color index

    //find color that appears the most
    int biggestvalue=0;
    if (purple>biggestvalue){biggestvalue=purple;}
    if (red>biggestvalue){biggestvalue=red;}
    if (blue>biggestvalue){biggestvalue=blue;}
    if (green>biggestvalue){biggestvalue=green;}
    if (yellow>biggestvalue){biggestvalue=yellow;}
    cout<<"Detection results:"<<endl;
    cout<<"Purple: "<<purple<<" times"<<endl;
    cout<<"red: "<<red<<" times"<<endl;
    cout<<"blue: "<<blue<<" times"<<endl;
    cout<<"green: "<<green<<" times"<<endl;
    cout<<"yellow: "<<yellow<<" times"<<endl;
    cout<<"Biggestvalue is: "<<biggestvalue<<endl;


    //percentage of belief
    double belief;
    if (biggestvalue==0){belief=0;}
    if (biggestvalue>0){belief=(biggestvalue*100)/(purple+red+blue+green+yellow);}

    if(biggestvalue>0){
        if(purple==biggestvalue){cout << "Found PURPLE Object!"<<endl;}
        if(red==biggestvalue){cout << "Found RED Object!"<<endl;}
        if(blue==biggestvalue){cout << "Found BLUE Object!"<<endl;}
        if(green==biggestvalue){cout << "Found GREEN Object!"<<endl;}
        if(yellow==biggestvalue){cout << "Found YELLOW Object!"<<endl;}
    cout <<"...We are "<<belief<<"% sure of it!"<<endl;
    }

cout << "PROGRAM FINISHED"<<endl;
imshow("Input Raw Image", image);
cv::waitKey();

    imshow("Input Raw Image", image);
    destroyAllWindows();





//SQUARE DETECTION

    //////////////////////////
    //        Main          //
    //////////////////////////




    if( image.empty() )
    {
        cout << "Couldn't load " << endl;
    }

    double j=1;
    Mat image_modified_lightning;

    for (j=1;j<15;j++)
    {image_modified_lightning=image*(j/5+0.6);cout<<"lightning condition: lightX"<< (j/5+0.6)<<endl; //lightning factors.

        vector<vector<Point> > squares;
        foundtimes=0;
        findSquares(image_modified_lightning, squares);  //call find square function
        drawSquares(image_modified_lightning, squares);  //call draw square function
        cout<<"Found sqaures "<<foundtimes<<" times"<<endl;
        waitKey(0);
    }


    waitKey(0);


    }




