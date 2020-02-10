#ifndef NEURAL_NETWORK_H
#define NEURAL_NETWORK_H

#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <ros/ros.h>
#include <opencv2/dnn.hpp>

class NeuralNetwork
{
public:
    NeuralNetwork();
    void detect(cv::Mat image);

private:
    std::vector<std::string> classes;
    cv::dnn::Net net;
    std::vector<cv::String> getOutputsNames(const cv::dnn::Net& net);
};

#endif