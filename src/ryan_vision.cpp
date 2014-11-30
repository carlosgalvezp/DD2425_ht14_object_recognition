#include <object_recognition/ryan_vision.h>


void detectRyan (const cv::Mat &image)
{

////////////////////////////////////////////////
//          How this code work                //
////////////////////////////////////////////////

//1) load original image and convert to HSV image with different lightning condition
//2) for each HSV masked image find a farily large contour in the relatative center of the image
//3) Check the HSV value at the center of the contour
//4) check if we can decide which object it is using object list
//4) decide which shape detector to run
//5) decide which object it is
//6) update the object list


////////////////////////////////////////////
//             Parameters                 //
////////////////////////////////////////////

//for detection
int hue_low_blue =10; int hue_low_red =110; int hue_low_green =50; int hue_low_purple =140; int hue_low_yellow =90;
int hue_high_blue=30; int hue_high_red=130; int hue_high_green=70; int hue_high_purple=170; int hue_high_yellow=100;
int sat_low=100; int sat_high=255; int bright_low=150; int bright_high=255;
bool blue=false; bool red=false; bool green=false; bool purple=false; bool yellow=false;
int max_contour_pixles=2000;
int color_idx=5; //1-purple, 2-red, 3-green, 4-blue, 5-yellow
int contour_min = 20;   //a perfect square is 75
int contour_max = 200;



//imshow("Input Raw Image", image);
//waitKey(0);

for(color_idx = 1; color_idx < 6; color_idx++){

///////////////////////////////////////////////////////////////////////////////////
//       1) Convert to different lightning conditions and convert into hsv       //
///////////////////////////////////////////////////////////////////////////////////

Mat image_modified_lightning;
for(int i = 0; i < 10; i++){
    if(i==0){image_modified_lightning = image*0.4;cout<<"lightning condition: lightX0.4"<<endl;}
    if(i==1){image_modified_lightning = image*0.5;cout<<"lightning condition: lightX0.5"<<endl;}
    if(i==2){image_modified_lightning = image*0.6;cout<<"lightning condition: lightX0.6"<<endl;}
    if(i==3){image_modified_lightning = image*0.7;cout<<"lightning condition: lightX0.7"<<endl;}
    if(i==4){image_modified_lightning = image*0.8;cout<<"lightning condition: lightX0.8"<<endl;}
    if(i==5){image_modified_lightning = image*0.9;cout<<"lightning condition: lightX0.9"<<endl;}
    if(i==6){image_modified_lightning = image*1.0;cout<<"lightning condition: NORMAL"<<endl;}
    if(i==7){image_modified_lightning = image*1.1;cout<<"lightning condition: lightX1.1"<<endl;}
    if(i==7){image_modified_lightning = image*1.2;cout<<"lightning condition: lightX1.2"<<endl;}
    if(i==8){image_modified_lightning = image*1.3;cout<<"lightning condition: lightX1.3"<<endl;}
    if(i==9){image_modified_lightning = image*1.4;cout<<"lightning condition: lightX1.4"<<endl;}
    if(i==8){image_modified_lightning = image*1.5;cout<<"lightning condition: lightX1.5"<<endl;}
    if(i==9){image_modified_lightning = image*1.6;cout<<"lightning condition: lightX1.6"<<endl;}



//imshow("Input Image lightning condition", image_modified_lightning);
//waitKey(0);


Mat img_hsv;
cvtColor(image_modified_lightning,img_hsv,CV_RGB2HSV);


/////////////////////////////////////////////////////////////////
//       2) find contours in all color masked images           //
/////////////////////////////////////////////////////////////////

Mat img_binary_purple;Mat img_binary_red;Mat img_binary_green;Mat img_binary_blue;Mat img_binary_yellow;

if(color_idx==1){inRange(img_hsv, Scalar(hue_low_purple, sat_low, bright_low), Scalar(hue_high_purple, sat_high, bright_high), img_binary_purple);cout << "Purple mask on"<< endl;
//imshow( "purple mask", img_binary_purple );
//waitKey(0);
}
if(color_idx==2){inRange(img_hsv, Scalar(hue_low_red, sat_low, bright_low), Scalar(hue_high_red, sat_high, bright_high), img_binary_red);cout << "Red mask on"<< endl;
//imshow( "red mask", img_binary_red );
//waitKey(0);
}
if(color_idx==3){inRange(img_hsv, Scalar(hue_low_green, sat_low, bright_low), Scalar(hue_high_green, sat_high, bright_high), img_binary_green);cout << "Green mask on"<< endl;
//imshow( "green mask", img_binary_green );
//waitKey(0);
}
if(color_idx==4){inRange(img_hsv, Scalar(hue_low_blue, sat_low, bright_low), Scalar(hue_high_blue, sat_high, bright_high), img_binary_blue);cout << "Blue mask on"<< endl;
//imshow( "blue mask", img_binary_blue );
//waitKey(0);
}
if(color_idx==5){inRange(img_hsv, Scalar(hue_low_yellow, sat_low, bright_low), Scalar(hue_high_yellow, sat_high, bright_high), img_binary_yellow);cout << "Yellow mask on"<< endl;
//imshow( "yellow mask", img_binary_yellow );
//waitKey(0);
}


int r=image.rows; int c=image.cols;

Mat img_contour_purple = Mat::zeros(r, c, CV_8UC1);
Mat img_contour_red = Mat::zeros(r, c, CV_8UC1);
Mat img_contour_green = Mat::zeros(r, c, CV_8UC1);
Mat img_contour_blue = Mat::zeros(r, c, CV_8UC1);
Mat img_contour_yellow = Mat::zeros(r, c, CV_8UC1);

vector<vector<Point> > contours_purple;
vector<vector<Point> > contours_red;
vector<vector<Point> > contours_green;
vector<vector<Point> > contours_blue;
vector<vector<Point> > contours_yellow;


if(color_idx==1)
{findContours(img_binary_purple,contours_purple,CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);cout<<"number of contours using purple mask: "<<contours_purple.size()<<endl;
 drawContours(img_contour_purple,contours_purple,-1,255,0);}  //-1 means draw all contours, 255 means white and 0 means single pixel thickness
if(color_idx==2)
{findContours(img_binary_red,contours_red,CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);cout<<"number of contours using red mask: "<<contours_red.size()<<endl;
 drawContours(img_contour_red,contours_red,-1,255,0);}
if(color_idx==3)
{findContours(img_binary_green,contours_green,CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);cout<<"number of contours using green mask: "<<contours_green.size()<<endl;
 drawContours(img_contour_green,contours_green,-1,255,0);}
if(color_idx==4)
{findContours(img_binary_blue,contours_blue,CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);cout<<"number of contours using blue mask: "<<contours_blue.size()<<endl;
 drawContours(img_contour_blue,contours_blue,-1,255,0);}
if(color_idx==5)
{findContours(img_binary_yellow,contours_yellow,CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);cout<<"number of contours using yellow mask: "<<contours_yellow.size()<<endl;
 drawContours(img_contour_yellow,contours_yellow,-1,255,0);}

//if(color_idx==1)
//{imshow( "contours purple", img_contour_purple );
//waitKey(0);}
//if(color_idx==2)
//{imshow( "contours red", img_contour_red );
//waitKey(0);}
//if(color_idx==3)
//{imshow( "contours green", img_contour_green );
//waitKey(0);}
//if(color_idx==4)
//{imshow( "contours blue", img_contour_blue );
//waitKey(0);}
//if(color_idx==5)
//{imshow( "contours yellow", img_contour_yellow );
//waitKey(0);}


////////////////////////////////////////////////////////////////////////////////////////////////
//        3)  check the size of the contours and draw the ones that are between min and max   //
////////////////////////////////////////////////////////////////////////////////////////////////

int sum_of_purple_pixles; int sum_of_red_pixles; int sum_of_green_pixles; int sum_of_blue_pixles; int sum_of_yellow_pixles;
//calculate amout of pixels in the contours image
if(color_idx==1){sum_of_purple_pixles = ((sum(img_contour_purple)/255)(0));
cout<< "Amount of purple pixles: " << sum_of_purple_pixles << endl;}
if(color_idx==2){sum_of_red_pixles = ((sum(img_contour_red)/255)(0));
cout<< "Amount of red pixles: " << sum_of_red_pixles << endl;}
if(color_idx==3){sum_of_green_pixles = ((sum(img_contour_green)/255)(0));
cout<< "Amount of green pixles: " << sum_of_green_pixles << endl;}
if(color_idx==4){sum_of_blue_pixles = ((sum(img_contour_blue)/255)(0));
cout<< "Amount of blue pixles: " << sum_of_blue_pixles << endl;}
if(color_idx==5){sum_of_yellow_pixles = ((sum(img_contour_yellow)/255)(0));
cout<< "Amount of yellow pixles : " << sum_of_yellow_pixles << endl;}

//if too many pixels, ignore this image
if(color_idx==1){if (sum_of_purple_pixles > max_contour_pixles){contours_yellow.clear();cout<<"Too many pixels! A very noisy image is removed!"<<endl;}}
if(color_idx==2){if (sum_of_red_pixles > max_contour_pixles){contours_yellow.clear();cout<<"Too many pixels! A very noisy image is removed!"<<endl;}}
if(color_idx==3){if (sum_of_green_pixles > max_contour_pixles){contours_yellow.clear();cout<<"Too many pixels! A very noisy image is removed!"<<endl;}}
if(color_idx==4){if (sum_of_blue_pixles > max_contour_pixles){contours_yellow.clear();cout<<"Too many pixels! A very noisy image is removed!"<<endl;}}
if(color_idx==5){if (sum_of_yellow_pixles > max_contour_pixles){contours_yellow.clear();cout<<"Too many pixels! A very noisy image is removed!"<<endl;}}

//remove small contours
if(color_idx==1){
Mat img_contour_purple_left = Mat::zeros(image.rows, image.cols, CV_8UC1);
for (vector<vector<Point> >::iterator it = contours_purple.begin(); it!=contours_purple.end(); )
{
    if (it->size() < contour_min)
       { it=contours_purple.erase(it);
        //cout << "Removed a small contour" << endl;
        }
    else if (it->size() > contour_max)
       { it=contours_purple.erase(it);
        //cout << "Removed a big contour" << endl;
    }
    else
        {++it;}
}
cout <<"Number of contours left after removing too big or too small contours: "<< contours_purple.size() << endl;
drawContours(img_contour_purple_left,contours_purple,-1,Scalar(255,255,255),1);

//imshow("Contours purple left",img_contour_purple_left);
//waitKey(0);
}
if(color_idx==2){
Mat img_contour_red_left = Mat::zeros(image.rows, image.cols, CV_8UC1);
for (vector<vector<Point> >::iterator it = contours_red.begin(); it!=contours_red.end(); )
{
    if (it->size() < contour_min)
       { it=contours_red.erase(it);
        //cout << "Removed a small contour" << endl;
        }
    else if (it->size() > contour_max)
       { it=contours_red.erase(it);
        //cout << "Removed a big contour" << endl;
    }
    else
        {++it;}
}
cout <<"Number of contours left after removing too big or too small contours: "<< contours_red.size() << endl;
drawContours(img_contour_red_left,contours_red,-1,Scalar(255,255,255),1);

//imshow("Contours red left",img_contour_red_left);
//waitKey(0);
}
if(color_idx==3){
Mat img_contour_green_left = Mat::zeros(image.rows, image.cols, CV_8UC1);
for (vector<vector<Point> >::iterator it = contours_green.begin(); it!=contours_green.end(); )
{
    if (it->size() < contour_min)
       { it=contours_green.erase(it);
        //cout << "Removed a small contour" << endl;
        }
    else if (it->size() > contour_max)
       { it=contours_green.erase(it);
        //cout << "Removed a big contour" << endl;
    }
    else
        {++it;}
}
cout <<"Number of contours left after removing too big or too small contours: "<< contours_green.size() << endl;
drawContours(img_contour_green_left,contours_green,-1,Scalar(255,255,255),1);

//imshow("Contours green left",img_contour_green_left);
//waitKey(0);
}
if(color_idx==4){
Mat img_contour_blue_left = Mat::zeros(image.rows, image.cols, CV_8UC1);
for (vector<vector<Point> >::iterator it = contours_blue.begin(); it!=contours_blue.end(); )
{
    if (it->size() < contour_min)
       { it=contours_blue.erase(it);
        //cout << "Removed a small contour" << endl;
        }
    else if (it->size() > contour_max)
       { it=contours_blue.erase(it);
        //cout << "Removed a big contour" << endl;
    }
    else
        {++it;}
}
cout <<"Number of contours left after removing too big or too small contours: "<< contours_blue.size() << endl;
drawContours(img_contour_blue_left,contours_blue,-1,Scalar(255,255,255),1);

//imshow("Contours blue left",img_contour_blue_left);
//waitKey(0);
}
if(color_idx==5){
Mat img_contour_yellow_left = Mat::zeros(image.rows, image.cols, CV_8UC1);
for (vector<vector<Point> >::iterator it = contours_yellow.begin(); it!=contours_yellow.end(); )
{
    if (it->size() < contour_min)
       { it=contours_yellow.erase(it);
        //cout << "Removed a small contour" << endl;
        }
    else if (it->size() > contour_max)
       { it=contours_yellow.erase(it);
        //cout << "Removed a big contour" << endl;
    }
    else
        {++it;}
}
cout <<"Number of contours left after removing too big or too small contours: "<< contours_yellow.size() << endl;
drawContours(img_contour_yellow_left,contours_yellow,-1,Scalar(255,255,255),1);

//imshow("Contours yellow left",img_contour_yellow_left);
//waitKey(0);
}





/////////////////////////////////////////////////////////
//        4)  Check if we have found an object         //
/////////////////////////////////////////////////////////


if (contours_purple.size()==1)
{purple=true;cout<<"a purple object is found!"<<endl;}
if (contours_red.size()==1)
{red=true;cout<<"a red object is found!"<<endl;}
if (contours_blue.size()==1)
{blue=true;cout<<"a blue object is found!"<<endl;}
if (contours_green.size()==1)
{green=true;cout<<"a green object is found!"<<endl;}
if (contours_yellow.size()==1)
{yellow=true;cout<<"a yellow object is found!"<<endl;}


//cout << "Done"<< endl;
//waitKey(0);
destroyAllWindows();

}//end of one lighting condition

}//end of color index

cout << "PROGRAM FINISHED"<<endl;
if(purple==true){cout << "Found PURPLE Object!"<<endl;}
if(red==true){cout << "Found RED Object!"<<endl;}
if(blue==true){cout << "Found BLUE Object!"<<endl;}
if(green==true){cout << "Found GREEN Object!"<<endl;}
if(yellow==true){cout << "Found YELLOW Object!"<<endl;}


//imshow("Input Raw Image", image);
//waitKey(0);


}//end of function

