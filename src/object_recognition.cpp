#include <object_recognition/object_recognition.h>

Object_Recognition::Object_Recognition()
{
    load_models();
}

std::string Object_Recognition::recognize(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud_in)
{
    std::clock_t begin = clock();
    pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr descriptors (new pcl::PointCloud<pcl::PFHRGBSignature250>);
    // ** Extract local 3D feature descriptors
    feat_extractor.get_descriptors(cloud_in, descriptors);

    // ** Matching with model
    int max_correspondences = 0;
    std::string best_model;

    for(unsigned int i = 0; i < objects_model_.size(); ++i)
    {
        int n_correspondences = getCorrespondences(descriptors, objects_model_[i].descriptors_);
//        std::cout << "Model "<<objects_model_[i].name_ <<": "<<n_correspondences << " correspondences"<<std::endl;
        if(n_correspondences > max_correspondences)
        {
            max_correspondences = n_correspondences;
            best_model = objects_model_[i].name_;
        }
    }
    double t = RAS_Utils::time_diff_ms(begin, clock());
    std::cout << "Object recognition: "<< t << " ms." << std::endl;
    // ** Output result
    return best_model;
}

int Object_Recognition::getCorrespondences(const pcl::PointCloud<pcl::PFHRGBSignature250>::ConstPtr &test_descriptors,
                                           const pcl::PointCloud<pcl::PFHRGBSignature250>::ConstPtr &model_descriptors)
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

void Object_Recognition::one_way_correspondences(const pcl::PointCloud<pcl::PFHRGBSignature250>::ConstPtr &source,
                             const pcl::PointCloud<pcl::PFHRGBSignature250>::ConstPtr &target,
                             std::vector<int>& correspondences)
{
    correspondences.resize (source->size());

    // Use a KdTree to search for the nearest matches in feature space
    pcl::search::KdTree<pcl::PFHRGBSignature250> descriptor_kdtree;
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

void Object_Recognition::load_models()
{
    std::cout << "Loading object models..."<<std::endl;
    // ** Read models directory and retrieve the PCD descriptors
    DIR *dir;
    struct dirent *ent;
    std::string models_path = std::string(getenv("HOME")) + models_rel_path_;
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


void Object_Recognition::load_object_model(const std::string &path, const std::string &model_name)
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
                pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr descriptors (new pcl::PointCloud<pcl::PFHRGBSignature250>);
                if(pcl::io::loadPCDFile(model_path, *descriptors) != 0)
                    ROS_ERROR("Error reading PCD file");
                std::cout << "Pointcloud: "<<descriptors->size()<<std::endl;
                Object_Model<pcl::PFHRGBSignature250> obj(model_name, descriptors);
                objects_model_.push_back(obj);
            }
        }
    }
    else
    {
        std::cout <<"Can't read directory "<<path<<std::endl;
    }
}
