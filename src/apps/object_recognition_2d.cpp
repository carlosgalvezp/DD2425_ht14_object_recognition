#include <iostream>
#include <ctime>
#include <ras_utils/ras_utils.h>
#include <object_recognition/shape_detector_2d.h>
// ROS
#include "ros/ros.h"
#include <std_msgs/String.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>

// Services
#include <ras_srv_msgs/Recognition.h>

// OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/core/core.hpp"

#define QUEUE_SIZE 10

void nullDeleter(void*);

class Object_Recognition_2D_Node{

public:
    Object_Recognition_2D_Node(const ros::NodeHandle& n);

private:
    ros::NodeHandle n_;
    Shape_Detector_2D shape_detector_;

    ros::Publisher speaker_pub_;
    ros::ServiceServer service_;

    bool Recognition_Callback(ras_srv_msgs::Recognition::Request  &req,
                              ras_srv_msgs::Recognition::Response &res);
    std::string recognize(const cv::Mat &rgb_img, const cv::Mat &mask, int color);
};
// =============================================================================
// =============================================================================

int main(int argc, char* argv[])
{
    // ** Init node
    ros::init(argc, argv, "object_recognition");
    ros::NodeHandle n;

    // ** Create object recognition object
    Object_Recognition_2D_Node o(n);

    ros::spin();
    return 0;
}

Object_Recognition_2D_Node::Object_Recognition_2D_Node(const ros::NodeHandle& n)
    : n_(n)
{
    // ** Publishers
    speaker_pub_ = n_.advertise<std_msgs::String>("/espeak/string", 1000);

    // ** Services
    service_ = n_.advertiseService("/object_recognition/recognition",
                                   &Object_Recognition_2D_Node::Recognition_Callback,this);
}

void nullDeleter(void*) {}

bool Object_Recognition_2D_Node::Recognition_Callback(ras_srv_msgs::Recognition::Request  &req,
                                                      ras_srv_msgs::Recognition::Response &res)
{
    // ** Convert ROS messages to OpenCV images and scale
    boost::shared_ptr<sensor_msgs::Image> req_rgb_ptr(&req.rgb_img, nullDeleter);
    boost::shared_ptr<sensor_msgs::Image> req_mask_ptr(&req.mask, nullDeleter);

    cv_bridge::CvImageConstPtr rgb_ptr     = cv_bridge::toCvShare(req_rgb_ptr);
    cv_bridge::CvImageConstPtr mask_ptr    = cv_bridge::toCvShare(req_mask_ptr);

    const cv::Mat& rgb_img     = rgb_ptr->image;
    const cv::Mat& mask        = mask_ptr->image;
    int color                  = req.color;
    // ** 2D object recognition
    std::string object = recognize(rgb_img, mask,color);

    // ** Publish to speaker node
    std::string str = ("I see " + object);
    ROS_ERROR("%s",str.c_str());
    std_msgs::String msg;
    msg.data = str;
//    speaker_pub_.publish(msg);
    res.result = 1;

    return true;
}

std::string Object_Recognition_2D_Node::recognize(const cv::Mat &rgb_img, const cv::Mat &mask, int color)
{
    std::string result;
    // ** Implement a decision tree
    // First branch: color. Next branches: shape (circle, square, triangle...)
    switch (color)
    {
        case 0: // BLUE
            ///@todo need triangle detector
            break;
        case 1: // RED/ORANGE
            if(shape_detector_.circle_detection(rgb_img))
            {
                result = "a red ball";
            }
            else
            {
//                if(shape_detector_.square_detection(rgb_img))
//                {
                    result = "a red cube";
//                }
//                else
//                {
//                    result = "Patric";
//                }
            }

            break;
        case 2: // GREEN
             if(shape_detector_.circle_detection(rgb_img))
             {
                result = "a green cylinder";
             }
             else
             {
                result = "a green cube";
             }
             break;
        case 3: // PURPLE
            result = "a purple cross";
            break;
        case 4: // YELLOW
            if(shape_detector_.circle_detection(rgb_img))
            {
                result = "a yellow ball";
            }
            else
            {
                result = "a yellow cube";
            }
            break;
    }

    return result;
}

