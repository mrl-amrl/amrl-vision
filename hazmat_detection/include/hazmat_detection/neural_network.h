#ifndef NEURAL_NETWORK_H
#define NEURAL_NETWORK_H

#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <ros/ros.h>
#include <opencv2/dnn.hpp>

typedef struct _rect
{
    cv::Rect rect;
    std::string name;
} detection_rect;

class NeuralNetwork
{
public:
    NeuralNetwork();
    NeuralNetwork(std::string cfg_path, std::string weights_path, std::string labels_path);
    std::vector<detection_rect> detect(cv::Mat image);

private:
    std::vector<std::string> classes;
    cv::dnn::Net net;
    std::vector<cv::String> getOutputsNames(const cv::dnn::Net &net);
};

#endif