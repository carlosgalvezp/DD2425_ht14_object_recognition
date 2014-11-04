#include <iostream>
#include <string>
#include <sstream>
#include <dirent.h>
#include <object_recognition/feature_extractor.h>

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

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/features/pfhrgb.h>
#include <pcl/io/pcd_io.h>

// OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#define QUEUE_SIZE 1000
#define PUBLISH_RATE 1.0/5.0  //Save data every 5 seconds
class Vision_Training{

    typedef image_transport::ImageTransport ImageTransport;
    typedef image_transport::Publisher ImagePublisher;
    typedef image_transport::SubscriberFilter ImageSubFilter;

    typedef message_filters::sync_policies::
        ApproximateTime<sensor_msgs::Image,
                        sensor_msgs::Image> RGBD_Sync_Policy;

    typedef message_filters::Synchronizer<RGBD_Sync_Policy> RGBD_Sync;

public:
    Vision_Training(const ros::NodeHandle& n);
    /**
     * @brief Save data at a given frequency
     */
    void record_data(std::string path, std::string model_name);

    /**
     * @brief Read the PCD data, computes the feature model, and saves it into .txt file
     */
    void compute_model(std::string path);

private:
    ros::NodeHandle n_;

    image_transport::ImageTransport rgb_transport_;
    image_transport::ImageTransport depth_transport_;

    image_transport::SubscriberFilter rgb_sub_;
    image_transport::SubscriberFilter depth_sub_;

    boost::shared_ptr<RGBD_Sync> rgbd_sync_;

    ros::Subscriber pcl_sub_;

    cv::Mat rgb_img_, depth_img_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;

    /**
     * @brief Callback to process RGB and Depth image
     * @param rgb_msg
     * @param depth_msg
     */
    void RGBD_Callback(const sensor_msgs::ImageConstPtr& rgb_msg,
                       const sensor_msgs::ImageConstPtr& depth_msg);

    /**
     * @brief Callback to process registered point cloud
     * @param pcl_msg
     */
    void PCL_Callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &pcl_msg);

    void process_model_folder(std::string path);
    void process_model(std::string path);

};

// =============================================================================
// =============================================================================

int main(int argc, char* argv[])
{
    if (argc < 2)
    {
        ROS_ERROR("Usage: rosrun vision_training <option>");
        ROS_ERROR("Options: ");
        ROS_ERROR("1 - Record PCD point clouds of the object");
        ROS_ERROR("2 - Given the PCD files, extract features and descriptors and store them");
        return -1;
    }

    // ** Init node
    ros::init(argc, argv, "vision_training");
    ros::NodeHandle n;

    // ** Create object recognition object
    Vision_Training o(n);

    int option = std::atoi(argv[1]);
    switch (option)
    {
        case 1:
            if (argc < 4)
            {
                ROS_ERROR("Usage: 'rosrun vision_training 1 <path-to-saved-files> <model_name>");
                return -1;
            }
            o.record_data(argv[2], argv[3]);
        break;

        case 2:
            if (argc < 3)
            {
                ROS_ERROR("Usage: rosrun vision_training 2 <path-to-models>");
                return -1;
            }
            o.compute_model(argv[2]);
        break;
    }

    return 0;
}


Vision_Training::Vision_Training(const ros::NodeHandle& n)
    : n_(n), rgb_transport_(n), depth_transport_(n)
{
    // ** Subscribers
    // Point Cloud
    pcl_sub_ = n_.subscribe<pcl::PointCloud<pcl::PointXYZRGB> >
            ("/camera/depth_registered/points", QUEUE_SIZE, &Vision_Training::PCL_Callback,this);

    //RGB and Depth images
    rgb_sub_.subscribe(rgb_transport_,
                       "/camera/rgb/image_raw", QUEUE_SIZE);
    depth_sub_.subscribe(depth_transport_,
                       "/camera/depth/image", QUEUE_SIZE);

    rgbd_sync_.reset(new RGBD_Sync(RGBD_Sync_Policy(QUEUE_SIZE), rgb_sub_, depth_sub_));
    rgbd_sync_->registerCallback(boost::bind(&Vision_Training::RGBD_Callback, this, _1, _2));

}

void Vision_Training::record_data(std::string path, std::string model_name)
{
    ROS_INFO("======= RECORDING DATA =======");
    // ** Publish data
    ros::Rate rate(PUBLISH_RATE);
    int n_item = 1;
    while(ros::ok())
    {
        ROS_INFO("Reading data...");
        ros::spinOnce(); //Get data

        // ** Save pcd and images
        ROS_INFO("Saving data %u...", n_item);
        // PCD
        if (cloud_ != 0)
        {
            std::stringstream ss, ss2, ss3;
            ss << path<<model_name<<"/raw/"<<model_name<<"_"<<n_item<<".pcd";
            pcl::io::savePCDFile(ss.str(), *cloud_);

            // Images
            ss2 << path<<model_name<<"/raw/" <<model_name<<"_RGB_"<<n_item<<".png";
            ss3 << path<<model_name<<"/raw/" << model_name<<"_D_"<<n_item<<".png";
            cv::imwrite(ss2.str(), rgb_img_);
            cv::imwrite(ss3.str(), depth_img_);

            ++n_item;
        }

        // ** Sleep
        rate.sleep();
    }
}

void Vision_Training::compute_model(std::string path)
{
    ROS_INFO("======= COMPUTING MODEL =======");
    DIR *dir;
    struct dirent *ent;
    const char* path_c = path.c_str();
    // ** Open base directory, which contains a folder for every model
    if((dir = opendir(path_c)) != NULL)
    {
        std::cout << "Entering directory " << path << std::endl;
        // ** Read entities (files or folders) and process folders only
        while((ent = readdir(dir)) != NULL)
        {
            if(ent->d_type == DT_DIR && ent->d_name[0] != '.')
            {
                std::string model_name = ent->d_name;
                std::string model_path = path + model_name;
                process_model_folder(model_path);
            }
        }

    }
    else
    {
        std::cout <<"Can't read directory "<<path<<std::endl;
    }
}

void Vision_Training::process_model_folder(std::string path)
{
    DIR *dir;
    struct dirent *ent;
    const char* path_c = (path+"/raw/").c_str();
    // ** Open directory, which contains .pcd files of objects
    if((dir = opendir(path_c)) != NULL)
    {
        std::cout << "Entering directory " << path << std::endl;
        // ** Read entities (files or folders) and process folders only
        while((ent = readdir(dir)) != NULL)
        {
            std::string f_name(ent->d_name);
            if(ent->d_type == DT_REG && f_name[f_name.length()-1] == 'd') // .pcD, look only for pcd files
            {
                std::string cloud_path = path + "/raw/" + f_name;
                std::string f_name_no_ext = f_name.substr(0, f_name.length()-4);
                std::string desc_name = path + "/descriptors/"+f_name_no_ext + "_desc.pcd";

                std::cout << "\t -"<< cloud_path<<std::endl;
                std::cout << "\t Saving into "<<desc_name << std::endl;

                // ** Process file
                Feature_Extractor fo;
                pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr descriptors (new pcl::PointCloud<pcl::PFHRGBSignature250>);
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZRGB>);

                pcl::io::loadPCDFile(cloud_path, *cloud_in);
                fo.get_descriptors(cloud_in, descriptors);
                pcl::io::savePCDFile(desc_name, *descriptors);
            }
        }

    }
    else
    {
        std::cout <<"Can't read directory "<<path<<std::endl;
    }
}

void Vision_Training::RGBD_Callback(const sensor_msgs::ImageConstPtr &rgb_msg,
                                    const sensor_msgs::ImageConstPtr &depth_msg)
{
    // ** Convert ROS messages to OpenCV images
    cv_bridge::CvImageConstPtr rgb_ptr   = cv_bridge::toCvShare(rgb_msg);
    cv_bridge::CvImageConstPtr depth_ptr = cv_bridge::toCvShare(depth_msg);

    const cv::Mat& rgb_img   = rgb_ptr->image;
    const cv::Mat& depth_img = depth_ptr->image;

    // ** Store image
    rgb_img_ = rgb_img;
    depth_img_ = depth_img;
}

void Vision_Training::PCL_Callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &pcl_msg)
{
    cloud_.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::copyPointCloud(*pcl_msg, *cloud_);
}
