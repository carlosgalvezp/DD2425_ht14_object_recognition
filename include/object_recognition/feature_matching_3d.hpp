#ifndef FEATURE_MATCHING_3D_H
#define FEATURE_MATCHING_3D_H

#include <object_recognition/object_model_3d.hpp>
#include <ras_utils/ras_utils.h>
#include <dirent.h>
// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>


template<typename DescriptorExtractor, typename DescriptorType>
class Feature_Matching_3D
{
public:
    Feature_Matching_3D();
    std::string match(const typename pcl::PointCloud<DescriptorType>::ConstPtr &object_descriptors);

private:
    std::vector<Object_Model<DescriptorType> > objects_model_;

    int getCorrespondences(const typename pcl::PointCloud<DescriptorType>::ConstPtr &test_descriptors,
                           const typename pcl::PointCloud<DescriptorType>::ConstPtr &model_descriptors);
    void one_way_correspondences(const typename pcl::PointCloud<DescriptorType>::ConstPtr &source,
                                 const typename pcl::PointCloud<DescriptorType>::ConstPtr &target,
                                 std::vector<int>& correspondences);
    void load_models();
    void load_object_model(const std::string &path, const std::string &model_name);
};


// =============================================================================
// =============================================================================
template<typename DescriptorExtractor, typename DescriptorType>
Feature_Matching_3D<DescriptorExtractor, DescriptorType>::Feature_Matching_3D()
{
    load_models();
}

template<typename DescriptorExtractor, typename DescriptorType>
std::string Feature_Matching_3D<DescriptorExtractor, DescriptorType>::match(const typename pcl::PointCloud<DescriptorType>::ConstPtr &descriptors)
{
    // ** Matching with model
    int max_correspondences = 0;
    std::string best_model;

    for(unsigned int i = 0; i < objects_model_.size(); ++i)
    {
        int n_correspondences = getCorrespondences(descriptors, objects_model_[i].descriptors_);
        std::cout << "Model "<<objects_model_[i].name_ <<": "<<n_correspondences << " correspondences"<<std::endl;
        if(n_correspondences > max_correspondences)
        {
            max_correspondences = n_correspondences;
            best_model = objects_model_[i].name_;
        }
    }
    std::cout << "Matching: "<< max_correspondences << " correspondences." << std::endl;
    // ** Output result
    return best_model;
}


template<typename DescriptorExtractor, typename DescriptorType>
int Feature_Matching_3D<DescriptorExtractor, DescriptorType>::getCorrespondences(
            const typename pcl::PointCloud<DescriptorType>::ConstPtr &test_descriptors,
            const typename pcl::PointCloud<DescriptorType>::ConstPtr &model_descriptors)
{
//    std::cout << "Finding correspondences..."<<std::endl;
    // ** 1-way correspondences
    std::vector<int> src2tgt, tgt2src;
    one_way_correspondences(test_descriptors, model_descriptors, src2tgt);
    one_way_correspondences(model_descriptors, test_descriptors, tgt2src);

    // ** First filtering
    pcl::CorrespondencesPtr correspondences_ (new pcl::Correspondences);

    std::vector<std::pair<unsigned, unsigned> > correspondences;
    for (unsigned cIdx = 0; cIdx < src2tgt.size (); ++cIdx)
        if (tgt2src[src2tgt[cIdx]] == cIdx)
            correspondences.push_back(std::make_pair(cIdx, src2tgt[cIdx]));

    correspondences_->resize (correspondences.size());
    for (unsigned cIdx = 0; cIdx < correspondences.size(); ++cIdx)
    {
        (*correspondences_)[cIdx].index_query = correspondences[cIdx].first;
        (*correspondences_)[cIdx].index_match = correspondences[cIdx].second;
    }

//    // ** Second filtering with RANSAC
//    std::cout << "Before RANSAC: "<<correspondences_->size() << std::endl;
//    // ** Second filtering: RANSAC
//    pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZI> rejector;
//    rejector.setInlierThreshold(0.05); //Default: 0.05
//    rejector.setRefineModel(true);      //Default: false
//    rejector.setMaximumIterations(10000); //Default: 1000

//    rejector.setInputSource(source_keypoints);
//    rejector.setInputTarget(target_keypoints);
//    rejector.setInputCorrespondences(correspondences_);
//    rejector.getCorrespondences(*correspondences_);

//    std::cout << "Final #: "<< correspondences_->size() << std::endl;

    return correspondences_->size();
}

template<typename DescriptorExtractor, typename DescriptorType>
void Feature_Matching_3D<DescriptorExtractor, DescriptorType>::one_way_correspondences(
        const typename pcl::PointCloud<DescriptorType>::ConstPtr &source,
        const typename pcl::PointCloud<DescriptorType>::ConstPtr &target,
                                       std::vector<int>& correspondences)
{
    correspondences.resize (source->size());

    // Use a KdTree to search for the nearest matches in feature space
    pcl::search::KdTree<DescriptorType> descriptor_kdtree;
    descriptor_kdtree.setInputCloud (target);

    // Find the index of the best match for each keypoint, and store it in "correspondences_out"
    const int k = 1;
    std::vector<int> k_indices (k);
    std::vector<float> k_squared_distances (k);
    for (size_t i = 0; i < source->size (); ++i)
    {
        descriptor_kdtree.nearestKSearch (*source, i, k, k_indices, k_squared_distances);
        correspondences[i] = k_indices[0];
    }
}

template<typename DescriptorExtractor, typename DescriptorType>
void Feature_Matching_3D<DescriptorExtractor, DescriptorType>::load_models()
{
    std::cout << "Loading object models..."<<std::endl;
    // ** Read models directory and retrieve the PCD descriptors
    DIR *dir;
    struct dirent *ent;
    std::string models_path = RAS_Names::models_3D_path;
    std::cout << "Models path: " << models_path << std::endl;

    if((dir = opendir(models_path.c_str())) != NULL)
    {
        // ** Read entities (files or folders) and process folders only
        while((ent = readdir(dir)) != NULL)
        {
            if(ent->d_type == DT_DIR && ent->d_name[0] != '.')
            {
                std::string model_name = ent->d_name;
                std::string model_path = models_path + model_name + "/descriptors/";
                load_object_model(model_path, model_name);
            }
        }

    }
    else
    {
        std::cout <<"Can't read directory " << models_path<<std::endl;
    }
}

template<typename DescriptorExtractor, typename DescriptorType>
void Feature_Matching_3D<DescriptorExtractor, DescriptorType>::load_object_model(const std::string &path, const std::string &model_name)
{
    // ** Read models directory and retrieve the PCD descriptors
    std::cout << "Loading object model: "<< model_name << std::endl;
    DIR *dir;
    struct dirent *ent;
    const char* path_c = path.c_str();
    if((dir = opendir(path_c)) != NULL)
    {
        // ** Read entities (files or folders) and process folders only
        while((ent = readdir(dir)) != NULL)
        {
            if(ent->d_type == DT_REG)
            {
                std::string model_path = path + ent->d_name;
                typename pcl::PointCloud<DescriptorType>::Ptr descriptors (new pcl::PointCloud<DescriptorType>);
                if(pcl::io::loadPCDFile(model_path, *descriptors) != 0)
                    ROS_ERROR("Error reading PCD file");
                std::cout << "Pointcloud: "<<descriptors->size()<<std::endl;
                Object_Model<DescriptorType> obj(model_name, descriptors);
                objects_model_.push_back(obj);
            }
        }
    }
    else
    {
        std::cout <<"Can't read directory "<<path<<std::endl;
    }
}

#endif // FEATURE_MATCHING_3D_H
