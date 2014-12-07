#include <object_recognition/train_color_models.h>

int main(int argc, char* argv[])
{
    std::vector<Color_Model> models;

    // ** Train color models
    train_models(models);

    // ** Save to file
    save_data(models);
}


void train_models(std::vector<Color_Model> &models)
{
    // ** Get list of directories
    std::string main_path = std::string(getenv("HOME")) + models_rel_path;
    std::vector<std::pair<std::string, std::string> > model_directories;
    getDirectories(main_path, model_directories);

    // ** Process each color
    for (std::size_t i = 0; i < model_directories.size(); ++i)
    {
        Color_Model m;
        const std::pair<std::string, std::string> &p = model_directories[i];
        computeModel(p.second, p.first, m);
        models.push_back(m);
    }

}

void getDirectories(const std::string &path, std::vector<std::pair<std::string, std::string> > &directories)
{
    for(std::size_t i = 0; i < RAS_Names::COLOR_NAMES.size(); ++i)
    {
        std::string model_name = RAS_Names::COLOR_NAMES[i];
        std::string model_path = path + model_name;
        directories.push_back(std::pair<std::string, std::string>(model_name, model_path));
    }
}

void getFiles(const std::string &path, std::vector<std::string> &files)
{
    DIR *dir;
    struct dirent *ent;
    const char* path_c = path.c_str();
    // ** Open base directory, which contains a folder for every model
    if((dir = opendir(path_c)) != NULL)
    {
        // ** Read entities (files or folders) and process folders only
        while((ent = readdir(dir)) != NULL)
        {
            if(ent->d_type == DT_REG)
            {
                std::string file_name = ent->d_name;
                std::string file_path = path + std::string("/")+file_name;
                files.push_back(file_path);
            }
        }

    }
    else
    {
        std::cout <<"Can't read directory "<<path<<std::endl;
    }
}

void computeModel(const std::string &path, const std::string &name, Color_Model &model)
{
    std::cout << "Model of "<< path << std::endl;
    // ** Get a list of images
    std::vector<std::string> image_paths;
    getFiles(path, image_paths);

    // ** Gather H and S data from the images
    std::vector<double> data_h;
    std::vector<double> data_s;

    for(std::size_t i = 0; i < image_paths.size(); ++i)
    {
        processImage(image_paths[i], data_h, data_s);
    }

    // ** Compute mu and sigma
    double mu_h    = RAS_Utils::mean(data_h);
    double mu_s    = RAS_Utils::mean(data_s);
    double sigma_h = RAS_Utils::std(data_h, mu_h);
    double sigma_s = RAS_Utils::std(data_s, mu_s);

    model.name = name;
    model.h_mu = mu_h;
    model.h_sigma = sigma_h;
    model.s_mu = mu_s;
    model.s_sigma = sigma_s;
}

void processImage(const std::string &path, std::vector<double> &data_h, std::vector<double> &data_s)
{
    // ** Read image
    cv::Mat img = cv::imread(path);

    // ** Convert from RGB to HSV and get channels
    cv::cvtColor(img, img, CV_RGB2HSV);
    std::vector<cv::Mat> channels;
    cv::split(img, channels);

    // ** Store H and S values in the vector
    for(unsigned int i = 0; i < img.rows; ++i)
    {
        for(unsigned int j = 0; j< img.cols; ++j)
        {
            data_h.push_back(channels[0].at<uint8_t>(i,j));
            data_s.push_back(channels[1].at<uint8_t>(i,j));
        }
    }

}

void save_data(const std::vector<Color_Model> &models)
{
    std::cout << "Saving data..."<<std::endl;
    std::ofstream myfile;
    std::string path = std::string(getenv("HOME")) + models_rel_path + std::string("models.txt");
    myfile.open(path.c_str());
    myfile << models.size() << std::endl;
    for(std::size_t i = 0; i < models.size(); ++i)
    {
        const Color_Model &m = models[i];
        myfile << m.name << std::endl;
        myfile << m.h_mu << std::endl;
        myfile << m.h_sigma << std::endl;
        myfile << m.s_mu << std::endl;
        myfile << m.s_sigma << std::endl;
    }

    myfile.close();
}
