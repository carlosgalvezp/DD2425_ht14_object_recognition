#ifndef TRAIN_COLOR_MODELS_H
#define TRAIN_COLOR_MODELS_H

#include <ras_utils/ras_utils.h>
#include <iostream>
#include <string>
#include <sstream>
#include <dirent.h>
#include <fstream>

// OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
struct Color_Model
{
    std::string name;
    double h_mu;
    double h_sigma;
    double s_mu;
    double s_sigma;
};

void train_models(std::vector<Color_Model> &models);
void save_data(const std::vector<Color_Model> &models);

void getDirectories(const std::string &path, std::vector<std::pair<std::string, std::string> > &directories);
void getFiles(const std::string &path, std::vector<std::string> &files);

void computeModel(const std::string &path, const std::string &name, Color_Model &model);
void processImage(const std::string &path, std::vector<double> &data_h, std::vector<double> &data_s);

#endif // TRAIN_COLOR_MODELS_H
