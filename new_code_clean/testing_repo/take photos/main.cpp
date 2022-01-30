#include "opencv2/opencv.hpp"
#include "iostream"
#include <eigen3/Eigen/Dense>
#include <boost/locale.hpp>

// 4.5.1
int main(int, char**) {
    // open the first webcam plugged in the computer
    cv::VideoCapture camera(2);
    if (!camera.isOpened()) {
        std::cerr << "ERROR: Could not open camera" << std::endl;
        return 1;
    }
    std::cout<< camera.get(cv::CAP_PROP_FRAME_WIDTH) <<std::endl;
    std::cout<< camera.get(cv::CAP_PROP_FRAME_HEIGHT)<<std::endl;

    // create a window to display the images from the webcam
    cv::namedWindow("Webcam", cv::WINDOW_AUTOSIZE);

    // this will contain the image from the webcam
    cv::Mat frame;
        
    // capture the next frame from the webcam
    camera >> frame;
    
    // display the frame until you press a key
    while (1) {
        // show the image on the window
        cv::imshow("Webcam", frame);
        // wait (10ms) for a key to be pressed
        if (cv::waitKey(10) >= 0)
            break;
    }
    return 0;
}
