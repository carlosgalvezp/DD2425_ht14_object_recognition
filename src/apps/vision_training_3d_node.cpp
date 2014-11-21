#include <iostream>
#include <string>
#include <sstream>
#include <dirent.h>
#include <object_recognition/feature_extractor_3d.hpp>
#include <object_recognition/object_extractor_3d.h>
#include <ras_utils/types.h>
#include <boost/filesystem.hpp>
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
#include <pcl/point_types_conversion.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

// OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Eigen
#include <Eigen/Core>
#include <Eigen/LU>

#define QUEUE_SIZE 1000
#define PUBLISH_RATE 1.0/5.0  //Save data every 5 seconds
class Vision_Training{


public:
    Vision_Training(const ros::NodeHandle& n);
    /**
     * @brief Save data at a given frequency
     */
    void record_data(std::string model_name);

    /**
     * @brief Read the PCD data, computes the feature model, and saves it into .txt file
     */
    void compute_models();

private:
    ros::NodeHandle n_;
    ros::Subscriber pcl_sub_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;

    Eigen::Matrix4f t_cam_to_robot_;

    Object_Extractor_3D object_extractor_;
    Feature_Extractor_3D<RAS_Utils::DescriptorExtractor, RAS_Utils::DescriptorType> feature_extractor_;

    /**
     * @brief Callback to process registered point cloud
     * @param pcl_msg
     */
    void PCL_Callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &pcl_msg);

    void process_model_folder(std::string path);
    void process_model(std::string path);

    void load_calibration(const std::string &path);
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
            if (argc < 3)
            {
                ROS_ERROR("Usage: 'rosrun vision_training 1 <model_name>");
                return -1;
            }
            o.record_data(argv[2]);
        break;

        case 2:
            if (argc < 2)
            {
                ROS_ERROR("Usage: rosrun vision_training 2");
                return -1;
            }
            o.compute_models();
        break;
    }

    return 0;
}


Vision_Training::Vision_Training(const ros::NodeHandle& n)
    : n_(n)
{
    // ** Subscribers
    // Point Cloud
    pcl_sub_ = n_.subscribe<pcl::PointCloud<pcl::PointXYZRGB> >
            ("/camera/depth_registered/points", QUEUE_SIZE, &Vision_Training::PCL_Callback,this);

    load_calibration(RAS_Names::CALIBRATION_PATH);
}

void Vision_Training::record_data(std::string model_name)
{
    ROS_INFO("======= RECORDING DATA =======");
    // ** Publish data
    ros::Rate rate(PUBLISH_RATE);
    int n_item = 1;
    std::string path = RAS_Names::models_3D_path + model_name+"/";
    // Create folder if it does not exist
    boost::filesystem::create_directory(path);

    std::string raw_folder = path + "raw/";
    // Empty the data folder
    boost::filesystem::remove_all(raw_folder); // This also removes the /raw/ folder
    boost::filesystem::create_directory(raw_folder); // Re-create it

    while(ros::ok())
    {
        ROS_INFO("Reading data...");
        ros::spinOnce(); //Get data

        // ** Save pcd and images
        ROS_INFO("Saving data %u...", n_item);
        // PCD
        if (cloud_ != 0)
        {
            std::stringstream ss;
            // Create directory if it does not exist
            ss << raw_folder <<model_name<<"_"<<n_item<<".pcd";
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_t (new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::transformPointCloud(*cloud_, *cloud_t, t_cam_to_robot_);
            pcl::io::savePCDFile(ss.str(), *cloud_t);

            ++n_item;
        }

        // ** Sleep
        rate.sleep();
    }
}

void Vision_Training::compute_models()
{
    ROS_INFO("======= COMPUTING MODEL =======");
    DIR *dir;
    struct dirent *ent;
    std::string path = RAS_Names::models_3D_path;
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
    // Empty descriptors folder
    std::string descriptors_folder = path + "/descriptors/";
    std::string objects_folder = path + "/object/";

    boost::filesystem::remove_all(descriptors_folder); // This also removes the /raw/ folder
    boost::filesystem::create_directory(descriptors_folder); // Re-create it
    boost::filesystem::remove_all(objects_folder); // This also removes the /raw/ folder
    boost::filesystem::create_directory(objects_folder); // Re-create it

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
                std::string desc_name = descriptors_folder+f_name_no_ext + "_desc.pcd";
                std::string object_name = objects_folder+f_name_no_ext + "_obj.pcd";

                std::cout << "\t Saving into "<<desc_name << std::endl;

                // ** Process file
                pcl::PointCloud<RAS_Utils::DescriptorType>::Ptr descriptors (new pcl::PointCloud<RAS_Utils::DescriptorType>);
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZRGB>);
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr object(new pcl::PointCloud<pcl::PointXYZRGB>);

                // Load .pcd file
                pcl::io::loadPCDFile(cloud_path, *cloud_in);

                // Extract object
                pcl::PointXYZ p(0,0,0);
                object_extractor_.extract_object(cloud_in, p, object);

                // Extract descriptors
                feature_extractor_.get_descriptors(object, descriptors);

                // Save descriptors
                pcl::io::savePCDFile(desc_name, *descriptors);
                pcl::io::savePCDFile(object_name, *object);
            }
        }
    }
    else
    {
        std::cout <<"Can't read directory "<<path_c<<std::endl;
    }
}

void Vision_Training::PCL_Callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &pcl_msg)
{
    cloud_.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::copyPointCloud(*pcl_msg, *cloud_);
}

void Vision_Training::load_calibration(const std::string &path)
{
    std::ifstream file;
    file.open(path.c_str());
    t_cam_to_robot_ = Eigen::Matrix4f::Identity();
    for(unsigned int i = 0; i < 4; ++i)
    {
        for(unsigned int j = 0; j < 4; ++j)
        {
            file >> t_cam_to_robot_(i,j);
        }
    }
    std::cout << "Read calibration: "<<std::endl << t_cam_to_robot_<<std::endl;
    file.close();
}
