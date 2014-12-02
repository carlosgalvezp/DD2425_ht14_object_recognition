#include <iostream>
#include <string>
#include <sstream>
#include <dirent.h>
#include <object_recognition/feature_extractor_3d.hpp>
#include <object_recognition/object_extractor_3d.h>
#include <ras_utils/types.h>
#include <boost/filesystem.hpp>

#include <object_recognition/vfh_recognition.h>

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
#include <message_filters/subscriber.h>
// PCL
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/segmentation/extract_clusters.h>

// OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Eigen
#include <Eigen/Core>
#include <Eigen/LU>

#define QUEUE_SIZE 2
#define SCALE_FACTOR 0.25

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
    typedef message_filters::sync_policies::
        ApproximateTime<sensor_msgs::Image,
                        sensor_msgs::Image> RGBD_Sync_Policy;
    typedef message_filters::Synchronizer<RGBD_Sync_Policy> RGBD_Sync;

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub_;
    message_filters::Subscriber<sensor_msgs::Image> depth_sub_;
    boost::shared_ptr<RGBD_Sync> rgbd_sync_;

    ros::NodeHandle n_;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;

    Eigen::Matrix4f t_cam_to_robot_;

    Object_Extractor_3D object_extractor_;
    Feature_Extractor_3D<RAS_Utils::DescriptorExtractor, RAS_Utils::DescriptorType> feature_extractor_;

    VFH_Recognition feature_extractor_vfh_;

    void RGBD_Callback(const sensor_msgs::ImageConstPtr &rgb_msg,
                       const sensor_msgs::ImageConstPtr &depth_msg);

    void process_model_folder(std::string path);
    void process_model(std::string path);

    void load_calibration(const std::string &path);
    void getObject(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud_in,
                                            pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out);
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
    rgb_sub_.subscribe(n_, TOPIC_CAMERA_RGB, QUEUE_SIZE);
    depth_sub_.subscribe(n_, TOPIC_CAMERA_DEPTH, QUEUE_SIZE);
    rgbd_sync_.reset(new RGBD_Sync(RGBD_Sync_Policy(QUEUE_SIZE), rgb_sub_, depth_sub_));
    rgbd_sync_->registerCallback(boost::bind(&Vision_Training::RGBD_Callback, this, _1, _2));


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
        // ** Sleep
        ROS_INFO("Reading data...");
        ros::spinOnce();
        rate.sleep();

        // PCD
        if (cloud_ != 0 && cloud_->size() != 0)
        {
            // ** Save pcd and images
            ROS_INFO("Saving data %u...", n_item);

            std::stringstream ss;
            // Create directory if it does not exist
            ss << raw_folder <<model_name<<"_"<<n_item<<".pcd";
            pcl::io::savePCDFile(ss.str(), *cloud_);

            ++n_item;
        }
    }
}

void Vision_Training::compute_models()
{
    ROS_INFO("======= COMPUTING MODEL BLABLABLA =======");
    DIR *dir;
    struct dirent *ent;
    std::string path = RAS_Names::models_3D_path;
    std::cout << "COMPUTE MODELS PATH: "<<path<<std::endl;
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
    std::string keypoints_folder =   path + "/keypoints/";

    boost::filesystem::remove_all(descriptors_folder); // This also removes the /raw/ folder
    boost::filesystem::create_directory(descriptors_folder); // Re-create it

    boost::filesystem::remove_all(keypoints_folder); // This also removes the /raw/ folder
    boost::filesystem::create_directory(keypoints_folder); // Re-create it

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
                std::string keypoint_name = keypoints_folder+f_name_no_ext + "_kpts.pcd";

                std::cout << "\t Saving into "<<desc_name << std::endl;

                // ** Process file
//                pcl::PointCloud<RAS_Utils::DescriptorType>::Ptr descriptors (new pcl::PointCloud<RAS_Utils::DescriptorType>);
                pcl::PointCloud<pcl::VFHSignature308>::Ptr descriptors (new pcl::PointCloud<pcl::VFHSignature308>);

                pcl::PointCloud<pcl::PointXYZRGB>::Ptr object(new pcl::PointCloud<pcl::PointXYZRGB>);
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZRGB>);

                // Load .pcd file
                pcl::io::loadPCDFile(cloud_path, *object);

                // Extract keypoints
                feature_extractor_.get_keypoints(object, keypoints);
//                feature_extractor_.get_descriptors(object, descriptors);
                feature_extractor_vfh_.computeDescriptor(object, descriptors);

                // Save descriptors
                pcl::io::savePCDFile(desc_name, *descriptors);
                pcl::io::savePCDFile(keypoint_name, *keypoints);
            }
        }
    }
    else
    {
        std::cout <<"Can't read directory "<<path_c<<std::endl;
    }
}

void Vision_Training::RGBD_Callback(const sensor_msgs::ImageConstPtr &rgb_msg, const sensor_msgs::ImageConstPtr &depth_msg)
{
    // ** Convert ROS messages to OpenCV images and scale
    cv_bridge::CvImageConstPtr rgb_ptr   = cv_bridge::toCvShare(rgb_msg);
    cv_bridge::CvImageConstPtr depth_ptr   = cv_bridge::toCvShare(depth_msg);
    const cv::Mat& rgb_img     = rgb_ptr->image;
    const cv::Mat& depth_img   = depth_ptr->image;

    // ** Build point cloud and transform to robot frame
    cloud_.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
    PCL_Utils::buildPointCloud(rgb_img, depth_img, cloud_, SCALE_FACTOR);
    pcl::transformPointCloud(*cloud_, *cloud_, t_cam_to_robot_);

    // ** Filter the object
    this->getObject(cloud_, cloud_);
}

void Vision_Training::getObject(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud_in,
                                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out)
{
    // ** Plane removal
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);

    seg.setInputCloud (cloud_in);
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
    {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
        return;
    }

    // Extract the non-plane
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud (cloud_in);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*cloud_out);

    // ** Pass through
    pcl::PassThrough<pcl::PointXYZRGB> pass;

    pass.setInputCloud (cloud_out);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.015,0.05);
    pass.filter (*cloud_out);

    // ** Clustering
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud (cloud_out);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_out);
    ec.extract (cluster_indices);

    std::size_t max_size=0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
            cloud_cluster->points.push_back (cloud_out->points[*pit]);

        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        if(cloud_cluster->size() > max_size)
        {
            max_size = cloud_cluster->size();
            *cloud_out = *cloud_cluster;
        }
    }
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
