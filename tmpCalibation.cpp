#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include "opencv2/core.hpp"
#include <opencv2/core/utility.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include <cstdlib>

// g++ -o main  calibration.cpp `pkg-config opencv --cflags eigen3 --libs`  -lglog -lboost_system -lboost_filesystem -lX11; ./main  -w=7 -h=4  -o=camera.yml -op -oe /home/nybo/tmp/opencv-3.2.0/samples/cpp/imagelist.YALM

// g++ -o main  tmpCalibation.cpp `pkg-config opencv --cflags eigen3 --libs`  -lglog -lboost_system -lboost_filesystem -lX11; ./main DJI_0019.JPG


using namespace cv;
using namespace std;
string inputFilename = "imagelist.yaml";

vector<string> imageList;



static bool readStringList( const string& filename, vector<string>& l )
{
    l.resize(0);
    FileStorage fs(filename, FileStorage::READ);
    if( !fs.isOpened() )
        return false;
    FileNode n = fs.getFirstTopLevelNode();
    if( n.type() != FileNode::SEQ )
        return false;
    FileNodeIterator it = n.begin(), it_end = n.end();
    for( ; it != it_end; ++it )
        l.push_back((string)*it);
    return true;
}


void readCal(cv::Mat &cameraMatrix_fil,
    cv::Mat &distCoeffs_fil){

    FileStorage fs;
    char* yml_filename ="/home/nybo/Documents/FishFarmGeometry/calibration/camera.yml";
    fs.open(yml_filename, FileStorage::READ);
    if( !fs.isOpened() ){
        cerr << " Fail to open "  << endl;
        exit(EXIT_FAILURE);
    }
    // Get camera parameters
    fs["camera_matrix"] >> cameraMatrix_fil;
    fs["distortion_coefficients"] >> distCoeffs_fil; 
    fs.release();

    cout << "\n -- Camera parameters -- " << endl;
    cout << "\n CameraMatrix = " << endl << " " << cameraMatrix_fil << endl << endl;
    cout << " Distortion coefficients = " << endl << " " << distCoeffs_fil << endl << endl;
}


int main( int argc, char** argv )
{
    if( argc != 2)
    {
     cout <<" Usage: display_image ImageToLoadAndDisplay" << endl;
     return -1;
    }

    Mat image;
    Mat image_undist_newDim;
    std::vector<Mat> array;    // make room for 10 integers,

    cv::Mat cameraMatrix_fil;
    cv::Mat distCoeffs_fil;
    readCal(cameraMatrix_fil,distCoeffs_fil );
    
    
    readStringList(inputFilename,imageList);

    for (int i = 0; i <= (imageList.size()-1); ++i) {
     image=imread(imageList[i], CV_LOAD_IMAGE_COLOR);
     array.push_back(image);   // Read the file
     Mat image_undist;
     cv::undistort(image,image_undist,  cameraMatrix_fil, distCoeffs_fil);
     array.push_back(image_undist);
    }

    cout <<  imageList[2] << std::endl ;


        cout << array.size()<< std::endl ;
    if(! image.data )                              // Check for invalid input
    {
        cout <<  "Could not open or find the image" << std::endl ;
        return -1;
    }





    







    namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.    
    for (int i = 0; i <= array.size()-1; ++i) {
    if(array[i].data )                              // Check for invalid input
    {   
        cv::resize(array[i], array[i], cv::Size(), 0.3, 0.3);
        imshow( "Display window", array[i] );                   
        cout <<  "disp" << std::endl ;
    }
    
    waitKey(0);                                          // Wait for a keystroke in the window
    }
    return 0;
}
