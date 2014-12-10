#include <object_recognition/ryan_vision.h>

using namespace cv;
using namespace std;


////////////////////////////////////////////
//             Parameters                 //
////////////////////////////////////////////


    //for color detection

    int hue_low_blue  =5  , hue_low_red  =115, hue_low_green  =50 , hue_low_purple  =140 , hue_low_yellow  =95;
    int hue_high_blue =25 , hue_high_red =125, hue_high_green =85 , hue_high_purple =170 , hue_high_yellow =110;
    int sat_low_blue  =1 ,  sat_low_red  =100, sat_low_green  =1  , sat_low_purple  =80  , sat_low_yellow  =150;
    int sat_high_blue =255, sat_high_red =255, sat_high_green =255, sat_high_purple =255 , sat_high_yellow =255;
    int bri_low_blue  =1  , bri_low_red  =1  , bri_low_green  =1  , bri_low_purple  =1   , bri_low_yellow  =1;
    int bri_high_blue =255, bri_high_red =255, bri_high_green =255, bri_high_purple  =255, bri_high_yellow =255;


    int max_contour_pixles=2000;

    int contour_min = 30, contour_max = 200;   //a perfect square is 75, FOR RED & ORANGE contour_min=50

    //for sqaure recognition
    int N = 10;  //N is iteration times with small change in grayscale value, very important constant, at least 10
    int light_conditions=10;
    double square_approx_accuracy=0.02; //how much we approximate contours to straight lines
    double triangle_approx_accuracy=0.05;
    double initial_light_condition=0.8;   //initial light condition with 0.2 stepN
    double cosinethreshold=0.3; //fight against not exact 90 degrees intersection, 0.1 means almost 90degree intersections.
    int contour_pixel_min = 1500, contour_pixel_max = 3500;

    //for circle recognition
    int radius_min = 30;    //red ball radius betwwen 40-75
    int radium_max = 80;    // 0,0 means no limit

////////////////////////////////////////////
//             Functions                  //
////////////////////////////////////////////
int color_detection(const cv::Mat &image,int color_idx)
{
    int purple=0, red=0, green=0, blue=0, yellow=0; //how many times have the algorithm detected a color
   int hue_low,hue_high,sat_low,sat_high,bri_low,bri_high;
    int color_index=0;
    int color=1;
    for(color = 1; color < 6; color++){
          Mat image_modified_lightning;
          for(double i = 1; i < 15; i++){
                image_modified_lightning = image*(i/10+0.5);
                //cout<<"lightning condition: lightX"<< (i/10+0.7)<<endl; //lightning factor 0.8-1.8
                Mat img_hsv;
                cvtColor(image_modified_lightning,img_hsv,CV_RGB2HSV);

/////////////////////////////////////////////////////////////////////////////////////////////
//       2) for each HSV masked image, remove noise then find farily large contours        //
/////////////////////////////////////////////////////////////////////////////////////////////

                Mat img_binary;
                if(color==1){hue_low = hue_low_purple; hue_high = hue_high_purple; sat_low = sat_low_purple; sat_high = sat_high_purple; bri_low = bri_low_purple; bri_high = bri_high_purple; contour_min=50;}
                if(color==2){hue_low = hue_low_red;    hue_high = hue_high_red;    sat_low = sat_low_red;    sat_high = sat_high_red;    bri_low = bri_low_red;    bri_high = bri_high_red;    contour_min=30;}
                if(color==3){hue_low = hue_low_green;  hue_high = hue_high_green;  sat_low = sat_low_green;  sat_high = sat_high_green;  bri_low = bri_low_green;  bri_high = bri_high_green;  contour_min=50;}
                if(color==4){hue_low = hue_low_blue;   hue_high = hue_high_blue;   sat_low = sat_low_blue;   sat_high = sat_high_blue;   bri_low = bri_low_blue;   bri_high = bri_high_blue;   contour_min=50;}
                if(color==5){hue_low = hue_low_yellow; hue_high = hue_high_yellow; sat_low = sat_low_yellow; sat_high = sat_high_yellow; bri_low = bri_low_yellow; bri_high = bri_high_yellow; contour_min=50;}


                inRange(img_hsv, Scalar(hue_low, sat_low, bri_low), Scalar(hue_high, sat_high, bri_high), img_binary);
                //imshow("red mask",img_binary;)
                //imshow("Input image",img_binary);waitKey(0);
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
                    if (sum_of_pixles > max_contour_pixles){
                        contours.clear();
                        //cout<<"Too many pixels! A very noisy image is removed!"<<endl;
                    }

                    //remove small contours
                        for (vector<vector<Point> >::iterator it = contours.begin(); it!=contours.end(); )
                        {
                            if (it->size() < contour_min)
                            { it=contours.erase(it);
                                //cout << "Removed a small contour" << endl;
                            }
                            else if (it->size() > contour_max)
                            { it=contours.erase(it);
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
                if(color==1){contours_purple=contours;}
                if(color==2){contours_red=contours;}
                if(color==3){contours_green=contours;}
                if(color==4){contours_blue=contours;}
                if(color==5){contours_yellow=contours;}

                if (contours_purple.size()==1)
                {purple=purple+1;
                    //cout<<"a purple object is found!"<<endl;
                }
                if (contours_red.size()==1)
                {red=red+1;
                    //cout<<"a red object is found!"<<endl;
                }
                if (contours_green.size()==1)
                {green=green+1;
                    //cout<<"a green object is found!"<<endl;
                }
                if (contours_blue.size()==1)
                {blue=blue+1;
                    //cout<<"a blue object is found!"<<endl;
                }
                if (contours_yellow.size()==1)
                {yellow=yellow+1;
                    //cout<<"a yellow object is found!"<<endl;
                }

                //cout << "Done"<< endl;
                //waitKey(0);
                //destroyAllWindows();

            }//end of one lighting condition

        }//end of one color index


cout << "COLOR DETECTION FINISHED"<<endl;
//imshow("Input Raw Image", image);

//RESULT
    //find color that appears the most
    int biggestvalue=0;
    if (purple>biggestvalue){biggestvalue=purple;}
    if (red>biggestvalue){biggestvalue=red;}
    if (green>biggestvalue){biggestvalue=green;}
    if (blue>biggestvalue){biggestvalue=blue;}
    if (yellow>biggestvalue){biggestvalue=yellow;}
//    cout<<"Detection results:"<<endl;
//    cout<<"Purple: "<<purple<<" times"<<endl;
//    cout<<"red: "<<red<<" times"<<endl;
//    cout<<"blue: "<<blue<<" times"<<endl;
//    cout<<"green: "<<green<<" times"<<endl;
//    cout<<"yellow: "<<yellow<<" times"<<endl;
//    cout<<"Biggestvalue is: "<<biggestvalue<<endl;

    //percentage of belief
    double belief;
    if (biggestvalue==0){belief=0;cout<<"Couldnt identify color!"<<endl;}
    if (biggestvalue>0){belief=(biggestvalue*100)/(purple+red+green+blue+yellow);}

    if(biggestvalue>0){
        if(purple==biggestvalue){color_index=1; cout << "Found PURPLE Object!"<<endl;}
        if(red==biggestvalue){color_index=2;    cout << "Found RED Object!"<<endl;}
        if(green==biggestvalue){color_index=3;  cout << "Found GREEN Object!"<<endl;}
        if(blue==biggestvalue){color_index=4;   cout << "Found BLUE Object!"<<endl;}
        if(yellow==biggestvalue){color_index=5; cout << "Found YELLOW Object!"<<endl;}
    cout <<"The probability of this is "<<belief<<"%"<<endl;
    }
    color_idx=color_index;
    return color_idx;
}

double circle_recognition(const Mat& image,double P_ball)
{

/////////////////////////////////////
//      PARAMETERS                 //
/////////////////////////////////////
  int conditions = 0;
  int found_circle = 0;

  for(double i = 1; i < 4; i++)
  {
  Mat image_modified_lightning = image*(i/10+1);
  //cout<<"lightning condition: lightX"<<(i/10+1)<<endl; //lightning factor 0.8-1.8


  //namedWindow("Win1", CV_WINDOW_AUTOSIZE);
  //imshow("Win1", image_modified_lightning);
  //waitKey(0);

  /// Blur image
  Mat blur;
  GaussianBlur(image_modified_lightning, blur, Size(3,3), 3, 3, BORDER_DEFAULT );  //Size Size(9,9), 2, 2,
  //namedWindow("Win2", CV_WINDOW_AUTOSIZE);
  //imshow("Win2", blur);
  //waitKey(0);

  /// Convert it to gray
  Mat gray;
  cvtColor( blur, gray, CV_RGB2GRAY );
  //namedWindow("Win3", CV_WINDOW_AUTOSIZE);
  //imshow("Win3", gray);
  //waitKey(0);

  //find edge using canny method

  Mat canny_output;
  //for (
  int thresh=150;// thresh<201; thresh=thresh+50;
      conditions=conditions+1; //used for calculate probability
      Canny(gray, canny_output, 0, thresh, 3);
      //imshow("Win5", canny_output);
      //waitKey(0);

      //int total_pixles = ((sum(canny_output)/255)(0)); //calculate amout of pixels in the canny image
      //cout<<"size of canny: "<<total_pixles<<endl;
///////////////////////////////////////////////////
//      3) Hough transform the egde image        //
///////////////////////////////////////////////////


    vector<Vec3f> circles;
    //cout << "Houghtransforming, please wait"  <<endl;

     /// Apply the Hough Transform to find the circles

     HoughCircles( canny_output, circles, CV_HOUGH_GRADIENT, 3, image.rows, 200, 200, radius_min, radium_max );

     //waitKey(0);

     ///Draw the circles detected
     Mat image_circle=Mat::zeros(image.rows, image.cols, CV_8UC1);
     for( size_t i = 0; i < circles.size(); i++ )
     {
         Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
         int radius = cvRound(circles[i][2]);

         // circle center
         circle( image_circle, center, 3, Scalar(255,0,0), -1, 8, 0 );  //this function draws circles
         //cout<< center <<endl;
         // circle outline
         circle( image_circle, center, radius, Scalar(255,255,255), 3, 8, 0 ); //this function draws circles
      }

     /// Show your results
     cout<<"number of circles in this image: "<<circles.size()<<endl;
     namedWindow( "Hough Circle", CV_WINDOW_AUTOSIZE );
     //imshow( "Hough Circle", image_circle );
     //waitKey(0);
     if (circles.size()==1){
         //cout<<"FOUND A CIRCLE"<<endl;
         found_circle=found_circle+1;}

    //end of a canny threshold
   }//end of a light condition
  cout<<"CIRCLE RECOGNITION FINISHED"<<endl;
  cout<<"There were "<<conditions<<" conditions, "<<"There were circles in "<<found_circle<<" of them!"<<endl;
  double belief=found_circle*100/(conditions);
  cout<<"The probability of finding a circle is "<<belief<<"%"<<endl;
  P_ball=belief;
  return P_ball;
}

double square_recognition( const Mat& image,double P_square)
{
    Mat image_modified_lightning;
    int square_this_grayscale=0;
    for (double j=1;j<(light_conditions+1);j++)//different light conditions
    {image_modified_lightning=image*(j/5+initial_light_condition);
        //cout<<"lightning condition: lightX"<< (j/5+0.6)<<endl; //lightning factors.
    int foundsquare=0;
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
                Canny(gray0, canny_output, 0, 600, 5);  //find edges
                }
            else
                {
                canny_output = gray0 >= (l+1)*255/N;  //change intensity values of edges
                dilate(canny_output, canny_output, Mat(), Point(-1,-1),2);
                erode(canny_output, canny_output, Mat(), Point(-1,-1),2);
            }
        vector<vector<Point> > contours;
        findContours(canny_output, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);
        drawContours(img_contours,contours,-1,255,1);
            //cout<<"number of contours: "<<contours.size()<<endl;
        vector<Point> approx;
        for( int i = 0; i < contours.size(); i++ )  // test each contour
        {
        approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*square_approx_accuracy, true);// approximate contour with accuracy proportional
                                                                                             // to the contour perimeter, 0,02 works on squares
        if( approx.size() == 4 && fabs(contourArea(Mat(approx))) > contour_pixel_min && fabs(contourArea(Mat(approx))) < contour_pixel_max  )// and be convex. && isContourConvex(Mat(approx))
        {
        double maxCosine = 0;
            for( int j = 2; j < 5; j++ )
            {
        // find the maximum cosine of the angle between joint edges
            Point pt1=approx[j%4]; Point pt2=approx[j-2];Point pt0=approx[j-1];
            double dx1 = pt1.x - pt0.x;
            double dy1 = pt1.y - pt0.y;
            double dx2 = pt2.x - pt0.x;
            double dy2 = pt2.y - pt0.y;
            double angle=(dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
            double cosine = fabs(angle);
            maxCosine = MAX(maxCosine, cosine);
            }
        if( maxCosine < cosinethreshold )
            {
            foundsquare=foundsquare+1;
            //cout<<squaretimes<<endl;

            }
        }
        }//end of one contour
}//end of one intensity level
if (foundsquare>0){
    //cout<<"we found a square in this gray scale channel!"<<endl;
square_this_grayscale=square_this_grayscale+1;
}
}//end of one grayscale
}//end of a light condition
    cout << "SQUARE RECOGNITION FINISHED"<<endl<<"There were "<<3*(light_conditions)<<" conditions, there were squares in " <<square_this_grayscale<< " of them!"<<endl;
    double belief=(100*square_this_grayscale)/(3*(light_conditions));
    cout << "The probability of finding a square is "<<belief<<"%"<<endl;
    P_square=belief;
    return P_square;
//end of function
}

double triangle_recognition( const Mat& image,double P_triangle)
{
    Mat image_modified_lightning;
    int triangle_this_grayscale=0;
    for (double j=1;j<(light_conditions+1);j++)//different light conditions
    {image_modified_lightning=image*(j/5+initial_light_condition);
        //cout<<"lightning condition: lightX"<< (j/5+0.6)<<endl; //lightning factors.
    int foundtriangle=0;
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
                Canny(gray0, canny_output, 0, 600, 5);  //find edges
            }
            else
                {
                canny_output = gray0 >= (l+1)*255/N;  //change intensity values of edges
                dilate(canny_output, canny_output, Mat(), Point(-1,-1),2);
                erode(canny_output, canny_output, Mat(), Point(-1,-1),2);
                //imshow("Canny output",canny_output);waitKey();
            }
        vector<vector<Point> > contours;
        findContours(canny_output, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);
        drawContours(img_contours,contours,-1,255,1);
            //cout<<"number of contours: "<<contours.size()<<endl;
        vector<Point> approx;
        for( int i = 0; i < contours.size(); i++ )  // test each contour
        {
        approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*triangle_approx_accuracy, true);// approximate contour with accuracy proportional
                                                                                             // to the contour perimeter, 0,02 works on squares
        if( approx.size() == 3 && fabs(contourArea(Mat(approx))) > contour_pixel_min && fabs(contourArea(Mat(approx))) < contour_pixel_max  )// and be convex. && isContourConvex(Mat(approx))
        {

            foundtriangle=foundtriangle+1;
            //cout<<squaretimes<<endl;


        }
        }//end of one contour
}//end of one intensity level
if (foundtriangle>0){
    //cout<<"we found a triangle in this gray scale channel!"<<endl;
triangle_this_grayscale=triangle_this_grayscale+1;
}
}//end of one grayscale
}//end of a light condition
    cout << "TRIANGLE RECOGNITION FINISHED"<<endl<<"There were "<<3*(light_conditions)<<" conditions, there were triangles in " <<triangle_this_grayscale<< " of them!"<<endl;
    double belief=(100*triangle_this_grayscale)/(3*(light_conditions));
    cout << "The probability of finding a triangle is "<<belief<<"%"<<endl;
    P_triangle=belief;
    return P_triangle;
//end of function
}

//Main
int visionRyan (const cv::Mat &image)
{

    //for (int a=1;a<10;a++){
    cvtColor(image,image,CV_BGR2RGB);

    //cvtColor(image,image,CV_BGR2RGB);
    imshow("Input image Ryan",image);

    //Probabilities and color index
    double P_ball=0;
    double P_square=0;
    double P_triangle=0;
    int color_idx=0;



    ///////////////////////////
    //         LOGIC         //
    ///////////////////////////

    //COLOR DETECTION
    color_idx=color_detection(image,color_idx);

    //1-purple, 2-red, 3-green, 4-blue, 5-yellow
    if (color_idx==1){
        cout<<"FOUND PURPLE CROSS!"<<endl;
    }
    if (color_idx==2){
        P_square=square_recognition(image,P_square);
        if(P_square>49){
            cout<<"FOUND RED CUBE!"<<endl;
        }
        else{
            P_ball=circle_recognition(image,P_ball);
            if(P_ball>24){
                cout<<"FOUND RED BALL!"<<endl;
            }
            else {
                cout<<"FOUND ORANGE STAR!"<<endl;
            }
        }
    }
    if (color_idx==3){
        P_square=square_recognition(image,P_square);
        if(P_square>49){
            cout<<"FOUND GREEN CUBE!"<<endl;
        }
        else{
            cout<<"FOUND GREEN CYLINDER!"<<endl;
        }
    }
    if (color_idx==4){
        P_square=square_recognition(image,P_square);
        P_triangle=triangle_recognition(image,P_triangle);
        if (P_square<10){P_square=0;}
        if (P_triangle<10){P_triangle=0;}
        if (P_square<P_triangle){cout<<"FOUND BLUE TRIANGLE!"<<endl;}
        if (P_square>P_triangle){cout<<"FOUND BLUE CUBE!"<<endl;}
        if (P_square==P_triangle){cout<<"FOUND BLUE TRIANGLE!"<<endl;}
    }
    if (color_idx==5){
        P_square=square_recognition(image,P_square);
        if(P_square>49){
            cout<<"FOUND YELLOW CUBE!"<<endl;
        }
        else{
            cout<<"FOUND YELLOW BALL!"<<endl;
        }
    }
    if (color_idx==0){
        cout<<"COULD NOT RECOGNIZE OBJECT!"<<endl;
    }

    //CIRCLE RECOGNITION
    //P_ball=findSquares(image,P_ball);

    //SQUARE RECOGNITION
    //P_square=findSquares(image,P_square);

    //TRIANGLE RECOGNITION
    //P_triangle=findTriangle(image,P_triangle);
    std::cout << "====================== END ====================" << std::endl;
    waitKey();


    destroyAllWindows();
}









