#ifndef FEATURE_MATCHING_3D_H
#define FEATURE_MATCHING_3D_H

#include <object_recognition/object_model_3d.hpp>
#include <ras_utils/ras_utils.h>
#include <dirent.h>
#include <object_recognition/vfh_recognition.h>
// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/correspondence.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#define MIN_N_CORRESPONDENCES 10

#define P_RATIO_TH  10

template<typename DescriptorExtractor, typename DescriptorType>
class Feature_Matching_3D
{
public:
    Feature_Matching_3D();
    void match(const typename pcl::PointCloud<DescriptorType>::ConstPtr &descriptors, std::vector<double> &class_probabilities);
    void match_vfh(const pcl::PointCloud<pcl::VFHSignature308>::ConstPtr &descriptors,
                                              std::vector<double> &class_probabilities);
private:
    std::vector<std::vector<Object_Model<DescriptorType> > > objects_model_;
    std::vector<std::vector<Object_Model<pcl::VFHSignature308> > > objects_model_vfh_;

    void geometricCorrespondences(const typename pcl::PointCloud<DescriptorType>::ConstPtr &test_descriptors,
                                  const typename pcl::PointCloud<DescriptorType>::ConstPtr &model_descriptors,
                                  pcl::CorrespondencesPtr &correspondences);

    int getCorrespondences(const typename pcl::PointCloud<DescriptorType>::ConstPtr &test_descriptors,
                           const typename pcl::PointCloud<DescriptorType>::ConstPtr &model_descriptors);
    void one_way_correspondences(const typename pcl::PointCloud<DescriptorType>::ConstPtr &source,
                                 const typename pcl::PointCloud<DescriptorType>::ConstPtr &target,
                                 std::vector<int>& correspondences);
    void load_models();
    void load_object_model(const std::string &path, const std::string &model_name);

    VFH_Recognition vfh_recognition_;
};


// =============================================================================
// =============================================================================
template<typename DescriptorExtractor, typename DescriptorType>
Feature_Matching_3D<DescriptorExtractor, DescriptorType>::Feature_Matching_3D()
{
    load_models();
}

template<typename DescriptorExtractor, typename DescriptorType>
void Feature_Matching_3D<DescriptorExtractor, DescriptorType>::match(const typename pcl::PointCloud<DescriptorType>::ConstPtr &descriptors,
                                                                            std::vector<double> &class_probabilities)
{
    class_probabilities.resize(objects_model_.size()+1); // The last one is the "other objects" category

    double p_sum = 0;
    for(std::size_t i = 0; i < objects_model_.size(); ++i)
    {
        const std::vector<Object_Model<DescriptorType> > &class_model = objects_model_[i];
        double max_prob = 0.0;

        for(std::size_t j = 0; j < class_model.size(); ++j)
        {
//            pcl::CorrespondencesPtr correspondences(new pcl::Correspondences);
//            geometricCorrespondences(descriptors, class_model[j].descriptors_, correspondences);
//            int n_correspondences = correspondences->size();
            int n_correspondences = getCorrespondences(descriptors, class_model[j].descriptors_);

            double probability = (double)n_correspondences / descriptors->size();

            if(probability > max_prob)
            {
                max_prob = probability;
                class_probabilities[i] = max_prob;
            }
        }
        std::cout << "Prob "<<class_model[0].name_<<": "<<class_probabilities[i]<<std::endl;
        p_sum += class_probabilities[i];
    }

    // ** Compute "others" probability
    double p_ratio = class_probabilities[0] / class_probabilities[1];
    double p_others;
    if(std::max(p_ratio, 1.0/p_ratio) > P_RATIO_TH) // It's really a cube or a ball
        p_others = std::min(class_probabilities[0], class_probabilities[1]);
    else
        p_others = 4.0/10.0; // Prior probability

    class_probabilities[class_probabilities.size()-1] = p_others;
    p_sum += p_others;
    // ** Normalize probabilities
    for(std::size_t i = 0; i < class_probabilities.size(); ++i)
    {
        class_probabilities[i] /= p_sum;
//        std::cout << "Prob "<<i<<": "<<class_probabilities[i]<<std::endl;
    }
}

template<typename DescriptorExtractor, typename DescriptorType>
void Feature_Matching_3D<DescriptorExtractor, DescriptorType>::match_vfh(const pcl::PointCloud<pcl::VFHSignature308>::ConstPtr &descriptors,
                                                                                        std::vector<double> &class_probabilities)
{
    class_probabilities.resize(objects_model_vfh_.size() +1);
    for(std::size_t i = 0; i < objects_model_vfh_.size(); ++i)
    {
        const std::vector<Object_Model<pcl::VFHSignature308> > &class_model = objects_model_vfh_[i];
        double max_prob = 0.0;

        for(std::size_t j = 0; j < class_model.size(); ++j)
        {
            double probability = vfh_recognition_.matchDescriptor(descriptors, class_model[j].descriptors_);
            if(probability > max_prob)
            {
                max_prob = probability;
                class_probabilities[i] = max_prob;
            }
        }
    }
}

template<typename DescriptorExtractor, typename DescriptorType>
void Feature_Matching_3D<DescriptorExtractor, DescriptorType>::geometricCorrespondences(const typename pcl::PointCloud<DescriptorType>::ConstPtr &test_descriptors,
                                                                                        const typename pcl::PointCloud<DescriptorType>::ConstPtr &model_descriptors,
                                                                                        pcl::CorrespondencesPtr &correspondences)
{
    // ** Correspondence extraction
    pcl::CorrespondencesPtr correspondences_tmp(new pcl::Correspondences);
    pcl::search::KdTree<DescriptorType> match_search;
    match_search.setInputCloud(model_descriptors);

    double min_distance = std::numeric_limits<double>::infinity();
    //  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
    for (size_t i = 0; i < test_descriptors->size (); ++i)
    {
        std::vector<int> neigh_indices (1);
        std::vector<float> neigh_sqr_dists (1);
        int found_neighs = match_search.nearestKSearch (test_descriptors->at(i), 1, neigh_indices, neigh_sqr_dists);
        if(found_neighs == 1 )//&& neigh_sqr_dists[0] < 0.25f) //
        {
            if(neigh_sqr_dists[0] < min_distance)
                min_distance = neigh_sqr_dists[0];

            pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
            correspondences_tmp->push_back (corr);
        }
    }

    // ** Filter correspondences too far away
    for(std::size_t i = 0; i < correspondences_tmp->size(); ++i)
    {
        const pcl::Correspondence &corr = correspondences_tmp->at(i);
        if(corr.distance < 3 * min_distance)
        {
            correspondences->push_back(corr);
        }
    }

    // ** Correspondence Grouping
//    pcl::GeometricConsistencyGrouping<pcl::PointXYZRGB, pcl::PointXYZRGB> gc_clusterer;
//    gc_clusterer.setGCSize (cg_size_);
//    gc_clusterer.setGCThreshold (cg_thresh_);

//    gc_clusterer.setInputCloud (model_keypoints);
//    gc_clusterer.setSceneCloud (scene_keypoints);
//    gc_clusterer.setModelSceneCorrespondences (model_scene_corrs);

//    //gc_clusterer.cluster (clustered_corrs);
//    gc_clusterer.recognize (rototranslations, clustered_corrs);
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

//    std::cout << "Correspondences: "<<src2tgt.size()<<","<<tgt2src.size()<<std::endl;
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
    std::string models_path = RAS_Names::MODELS_3D_PATH;
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
//    std::vector<Object_Model<DescriptorType> > class_models;
    std::vector<Object_Model<pcl::VFHSignature308> > class_models;

    if((dir = opendir(path_c)) != NULL)
    {
        // ** Read entities (files or folders) and process folders only
        while((ent = readdir(dir)) != NULL)
        {
            if(ent->d_type == DT_REG)
            {
                std::string model_path = path + ent->d_name;
                std::cout << "Feature matching. Reading "<<model_path << std::endl;
//                typename pcl::PointCloud<DescriptorType>::Ptr descriptors (new pcl::PointCloud<DescriptorType>);
                typename pcl::PointCloud<pcl::VFHSignature308>::Ptr descriptors (new pcl::PointCloud<pcl::VFHSignature308>);

                if(pcl::io::loadPCDFile(model_path, *descriptors) != 0)
                    ROS_ERROR("Error reading PCD file");
                std::cout << "Pointcloud: "<<descriptors->size()<<std::endl;
//                Object_Model<DescriptorType> obj(model_name, descriptors);
                Object_Model<pcl::VFHSignature308> obj(model_name, descriptors);
                class_models.push_back(obj);
            }
        }
//        objects_model_.push_back(class_models);
        objects_model_vfh_.push_back(class_models);
    }
    else
    {
        std::cout <<"Can't read directory "<<path<<std::endl;
    }
}

#endif // FEATURE_MATCHING_3D_H
