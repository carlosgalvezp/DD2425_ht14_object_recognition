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
    std::string obj_name = obj_recognition.recognize(cloud);
    std::cout << "==== The object is: "<< obj_name << " ===="<<std::endl;

    return 0;
}
