#include <iostream>
#include <object_recognition/object_recognition_3d.h>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>

int main(int argc, char* argv[])
{
    if (argc < 2)
    {
        std::cout << "Usage: ./test_object_recognition <path-to-pcd-test-file>"<<std::endl;
        return -1;
    }

    // ** Create object recognition object
    Object_Recognition_3D obj_recognition;

    // ** Load test point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile(argv[1], *cloud);

    // ** Recognize
    std::vector<double> class_probabilities(3);
    ros::WallTime t1(ros::WallTime::now());
    obj_recognition.recognize_vfh(cloud, class_probabilities);
    std::cout << "Time recognition VFH: "<<RAS_Utils::time_diff_ms(t1, ros::WallTime::now()) <<" ms"<<std::endl;
    std::cout << "==== Probabilities: "<< class_probabilities[0] << ","<<class_probabilities[1]<<std::endl;
    return 0;
}
