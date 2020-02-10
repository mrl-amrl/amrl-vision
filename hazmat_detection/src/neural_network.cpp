#include <hazmat_detection/neural_network.h>

NeuralNetwork::NeuralNetwork()
{
}

NeuralNetwork::NeuralNetwork(std::string cfg_path, std::string weights_path, std::string labels_path)
{
    std::ifstream ifs(labels_path.c_str());
    std::string line;
    while (std::getline(ifs, line))
        classes.push_back(line);

    net = cv::dnn::readNetFromDarknet(cfg_path, weights_path);
    net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
}

std::vector<cv::String> NeuralNetwork::getOutputsNames(const cv::dnn::Net &net)
{
    static std::vector<cv::String> names;
    if (names.empty())
    {
        std::vector<int> outLayers = net.getUnconnectedOutLayers();
        std::vector<cv::String> layersNames = net.getLayerNames();
        names.resize(outLayers.size());
        for (size_t i = 0; i < outLayers.size(); ++i)
            names[i] = layersNames[outLayers[i] - 1];
    }
    return names;
}

std::vector<detection_rect> NeuralNetwork::detect(cv::Mat image)
{
    cv::Mat blob;
    cv::dnn::blobFromImage(image, blob, 1 / 255.0, cv::Size(576, 576), cv::Scalar(0, 0, 0), true, false);

    net.setInput(blob);

    std::vector<cv::Mat> outs;
    std::vector<cv::String> names = getOutputsNames(net);
    net.forward(outs, names);

    std::vector<int> classIds;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;

    for (size_t i = 0; i < outs.size(); ++i)
    {
        float *data = (float *)outs[i].data;
        for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols)
        {
            cv::Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
            cv::Point classIdPoint;
            double confidence;
            minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
            if (confidence > 0.8)
            {
                int centerX = (int)(data[0] * image.cols);
                int centerY = (int)(data[1] * image.rows);
                int width = (int)(data[2] * image.cols);
                int height = (int)(data[3] * image.rows);
                int left = centerX - width / 2;
                int top = centerY - height / 2;

                classIds.push_back(classIdPoint.x);
                confidences.push_back((float)confidence);
                boxes.push_back(cv::Rect(left, top, width, height));
            }
        }
    }
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, 0.8, 0.3, indices);

    std::vector<detection_rect> output;
    for (size_t i = 0; i < indices.size(); ++i)
    {
        int idx = indices[i];
        cv::Rect box = boxes[idx];
        detection_rect r;
        r.rect = boxes[idx];
        r.name = classes[classIds[idx]];
        output.push_back(r);
    }
    return output;
}
