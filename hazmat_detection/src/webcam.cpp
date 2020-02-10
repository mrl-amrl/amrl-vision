#include <hazmat_detection/neural_network.h>
#include <opencv2/highgui.hpp>

int main(int argc, char **argv)
{
    std::cout << "OpenCV version : " << CV_VERSION << std::endl;
    std::cout << "Major version : " << CV_MAJOR_VERSION << std::endl;
    std::cout << "Minor version : " << CV_MINOR_VERSION << std::endl;
    std::cout << "Subminor version : " << CV_SUBMINOR_VERSION << std::endl;

    cv::VideoCapture camera(0);
    NeuralNetwork nn;
    while (cv::waitKey(1) < 0)
    {
        cv::Mat frame;
        camera >> frame;
        if (frame.empty())
            break;
        nn.detect(frame);
        cv::imshow("frame", frame);
    }
    return 0;
}
