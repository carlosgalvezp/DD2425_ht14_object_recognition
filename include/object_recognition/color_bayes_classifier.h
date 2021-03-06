#ifndef COLOR_BAYES_CLASSIFIER_H
#define COLOR_BAYES_CLASSIFIER_H

#include <ras_utils/ras_utils.h>
#include <object_recognition/train_color_models.h>
#include <iostream>
#include <fstream>

#include <vector>

#define TH_MAHALANOBIS 2        // Reject points further than 2*sigma distance
#define STEP 5                 // How much we subsample when analysing the image
struct Model_Constants // To speed up operations
{
    double log_alpha;
    double log_sigma_h;
    double log_sigma_s;
    double inv_2_sigma_h_2;
    double inv_2_sigma_s_2;
    double th_h_mahalanobis;
    double th_s_mahalanobis;
};

class Color_Bayes_Classifier
{
public:
    Color_Bayes_Classifier();

    int classify(const int &h, const int &s);
    void classify(const cv::Mat &hsv_img, const cv::Mat &mask, const std::vector<int> &color_classes, std::vector<double> &color_probabilities);
private:

    void read_models();
    double discriminant(const double &h, const double &s, const int &idx);
    std::vector<Color_Model> models_;
    std::vector<Model_Constants> models_constants_;
};

#endif // COLOR_BAYES_CLASSIFIER_H
