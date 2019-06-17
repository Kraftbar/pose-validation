
#include <stdio.h>


// recorces
// https://dsp.stackexchange.com/questions/1881/camera-calibration-pin-hole-camera-model-and-working-out-3d-position
// https://stackoverflow.com/questions/7836134/get-3d-coordinates-from-2d-image-pixel-if-extrinsic-and-intrinsic-parameters-are/10750648
// http://cvrs.whu.edu.cn/downloads/ebooks/Multiple%20View%20Geometry%20in%20Computer%20Vision%20(Second%20Edition).pdf (p 196)

// csv
#include <fstream>
#include <algorithm>

//  rotate
#include <opencv2/opencv.hpp>
# include <Eigen/Core>

// inverse
#include <Eigen/Dense>
// psudo inv
#include <Eigen/QR>    


// camera param
#include "utils/camParameters.h"
#include <opencv2/core/eigen.hpp>
#include <boost/algorithm/string.hpp>

// triangulate
#include "utils/triangulate.h"




std::vector<Eigen::Vector3f> logPos_eigen;
std::vector<Eigen::Vector3f> logAtt_eigen;

int  readCsv(const char *lol)
{   
    Eigen::Vector3f vecTmp(3);
    std::ifstream cFile (lol);
    if (cFile.is_open())
    {
        std::string line;
        while(getline(cFile, line)){
            if(line[0] == '#' || line.empty())
                continue;
            std::vector<std::string> strs;
            boost::split(strs,line,boost::is_any_of(","));
            vecTmp(0)= std::stod(strs[0]);
            vecTmp(1)=std::stod(strs[1]);
            vecTmp(2)=std::stod(strs[2]);
            logAtt_eigen.emplace_back(vecTmp);
            vecTmp(0)= std::stod(strs[3]);
            vecTmp(1)=std::stod(strs[4]);
            vecTmp(2)=std::stod(strs[5]);
            logPos_eigen.emplace_back(vecTmp);
       }
        
    }
    else {
        std::cerr << "Couldn't open config file for reading.\n";
    }
    return 1;
}

Eigen::Matrix3f getRotationMat(Eigen::Vector3f angles){

    double roll, pitch, yaw;
    roll=angles(0)*M_PI/180.0;
    pitch=angles(1)*M_PI/180.0;
    yaw=angles(2)*M_PI/180.0+M_PI/2;
    Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitZ());
     Eigen::Quaternion<float> q =  yawAngle*pitchAngle *rollAngle;
    Eigen::Matrix3f rotationMatrix = q.matrix();
    return rotationMatrix;
 }



std::vector<cv::Mat> readImgs(const char *folder){


std::vector<cv::String> fn;
cv::glob(folder, fn, false);

std::vector<cv::Mat> images;
size_t count = fn.size(); //number of png files in images folder
for (size_t i=0; i<count; i++)
    images.push_back(imread(fn[i]));
    return images;
}

int main(int argc, char** argv )
{
// read cam param


    auto imgList=readImgs("../../*.png");
    


	cv::Mat cameraMatrix;
	cv::Mat cameraDistCoeffs;
	cv::FileStorage fs("../../../../camera.xml", cv::FileStorage::READ);
	fs["camera_matrix"] >> cameraMatrix;
	fs["distortion_coefficients"] >> cameraDistCoeffs;


	cv::Size sz = imgList[0].size();
    cameraMatrix.at<double>(0, 2) = sz.width/2;
	cameraMatrix.at<double>(1, 2) =sz.height/2;


// for dist
  cv::Mat new_camera_matrix= getOptimalNewCameraMatrix(cameraMatrix, cameraDistCoeffs, sz, 1,sz, 0);


  Eigen::Matrix3f K;
  cv::cv2eigen(cameraMatrix,K);    




// read csv
    char const *file="../../attPos_log (copy).csv";
   readCsv (file);


Eigen::Vector3f Kp1(431 , 131,
1);
Eigen::Vector3f Kp2(435 , 226,
1);
Eigen::Vector3f Kp3(441 , 313,
1);
Eigen::Vector3f Kp4(420 , 280,
1);



Eigen::Vector3f Kps []={Kp1,Kp2,Kp3,Kp4};


Eigen::Vector3f Kp1_d(429 , 113,1);
Eigen::Vector3f Kp2_d(427 , 228,1);
Eigen::Vector3f Kp3_d(387 , 284,1);
Eigen::Vector3f Kp4_d(440, 255,1);
std::vector<cv::Vec2d> Kpsdes;
Kpsdes.push_back(cv::Vec2d(429 , 114));
Kpsdes.push_back(cv::Vec2d(430 , 226));
Kpsdes.push_back(cv::Vec2d(387 , 284));
Kpsdes.push_back(cv::Vec2d(442 , 252));
Eigen::Vector3f Kpsd []={Kp1_d,Kp2_d,Kp3_d,Kp4_d};
std::vector<cv::Point2f> KpsCV;
KpsCV.push_back(cv::Point2f(429 , 114));
KpsCV.push_back(cv::Point2f(430 , 226));
KpsCV.push_back(cv::Point2f(387 , 284));
KpsCV.push_back(cv::Point2f(442 , 252));
KpsCV.push_back(cv::Point2f(518 , 244));


std::vector<cv::Mat> cameraPoses;




// Distorted mode
int distort=1;
int debug =0;

std::vector<cv::Vec2d> undistp;

if(distort==0){

    cv::cv2eigen(new_camera_matrix,K);    
    std::copy(std::begin(Kpsd), std::end(Kpsd), std::begin(Kps));
    std::cout<<"Using dist cam K matrix:"<<std::endl;
    std::cout<<new_camera_matrix<<std::endl;


	cv::undistortPoints(Kpsdes, undistp, cameraMatrix, cameraDistCoeffs);
//        Kps[i](0)=Newpoists[i](0);
//        Kps[i](1)=Newpoists[i](1);


}

cv::viz::Viz3d visualizer("Debug");
visualizer.setBackgroundColor(cv::viz::Color::white());
        
// roate and plot
for (int i = 0; i < logAtt_eigen.size(); i++) {

        auto attVect = logAtt_eigen[i];
        auto posVect = logPos_eigen[i];
        auto p       = Kps[i];



        // NED
        posVect(2)=-posVect(2);

        
        



        // full rot ext matrix
        Eigen::Matrix3f rotation; 
        Eigen::MatrixXf rotationTrans_full = Eigen::MatrixXf::Identity(3, 4);
        rotation<< getRotationMat(attVect);
        rotationTrans_full << rotation,posVect ;

        // for triangulation later
        cv::Mat poseTmp;
        cv::eigen2cv(rotationTrans_full,poseTmp);
        cameraPoses.push_back(poseTmp);


        // projection matrix        
        Eigen::MatrixXf projectionM = K*rotationTrans_full;





        Eigen::Vector3f N;
        N<<0,0,-1;
        // 1. Assume Plane - Eq 8.1, Hartley and Zisserman   - cyan
        Eigen::MatrixXf H_tmp(3,3);
        H_tmp                           << rotationTrans_full.col(0), rotationTrans_full.col(1),   
              
                                                       rotationTrans_full.col(3);    
 //       float d= posVect.norm();
 //       Eigen::MatrixXf H               = rotation-(1/d)*rotation*posVect*N.transpose();
 //       Eigen::VectorXf projectionNorm_m1 = H.inverse() * p;
 //       projectionNorm_m1=projectionNorm_m1/projectionNorm_m1(2);



                Eigen::MatrixXf H               = H_tmp;    
                Eigen::MatrixXf H_norm          = H/(posVect(2));   
                Eigen::MatrixXf projection      = H.inverse() * p;   
                Eigen::VectorXf projectionNorm_m1 = projection;
        




        // 2. Løser pinhole camera model eq    - Brown
        Eigen::MatrixXf projectionM_inv   = projectionM.completeOrthogonalDecomposition().pseudoInverse();
        Eigen::VectorXf world_cord        = projectionM_inv*p;
        Eigen::VectorXf projectionNorm_m2 = (world_cord) *(-posVect(2))   ;    




        // 3. Project rays
        Eigen::Vector3f imgKp=K.inverse()*p;
        imgKp=imgKp.normalized();
        

        
        Eigen::Vector3f imgKpNonHom;
        imgKpNonHom<<imgKp(0),imgKp(1),imgKp(2);
        imgKpNonHom=imgKpNonHom+posVect;


        //        Eigen::Vector4f imgKpHom;
        //        imgKpHom<<imgKp(0),imgKp(1),imgKp(2),1;
        //        imgKpHom=imgKpHom/imgKpHom(3);
        //        std::cout<<rotationTrans_full*imgKpHom<<std::endl;






        // Visualize
        if(debug=1){
        std::cout<<std::endl;




        // 1.
        cv::Point3d                testpoint1(projectionNorm_m1(0),projectionNorm_m1(1),projectionNorm_m1(2));
	    cv::viz::WSphere           point3Dm1(testpoint1, 2, 30, cv::viz::Color::cyan());
        
        // 2.
        cv::Point3d                testpoint2(projectionNorm_m2(0),projectionNorm_m2(1),projectionNorm_m2(2));
	    cv::viz::WSphere           point3Dm2(testpoint2/3, 2, 30, cv::viz::Color::brown());

        //3
        cv::Point3d ray(imgKp(0),imgKp(1),imgKp(2));
        ray=ray*(-posVect(2));
	    cv::viz::WSphere           point3Dm3(ray , 2, 30, cv::viz::Color::red());

        

        // 4



    



        cv::Mat rvec, tvec;
        cv::eigen2cv(attVect,rvec);
        cv::eigen2cv(posVect,tvec);
        cv::Matx33d cameraMatrixVis=cameraMatrix;

        cv::viz::WCameraPosition   cam( cameraMatrixVis,  imgList[i], 30.0,cv::viz::Color::blue() );
    	cv::viz::WLine             line(cv::Point3d(0., 0., 0.),ray, cv::viz::Color::green());
        cv::viz::WCoordinateSystem coordinate(30);
        cv::viz::WGrid             grid(cv::Vec2i(20  , 14),cv::Vec2i(20  , 14),cv::viz::Color::gray());
	    cv::viz::WCircle            groundTruth( 30,0.7,cv::viz::Color::gray()     );

        

        std::string inum=std::to_string(i);




        visualizer.showWidget("Ground grid circle"+inum, grid);
        visualizer.showWidget("Coordinate Widget"+inum, coordinate);
        visualizer.showWidget("Camera"+inum, cam);
    	visualizer.showWidget("Line"+inum, line);
	    visualizer.showWidget("Point method 1"+inum, point3Dm1);
	    visualizer.showWidget("Point method 2"+inum, point3Dm2);
	    visualizer.showWidget("Point method 3"+inum, point3Dm3);
        visualizer.showWidget("Ground truth buoy"+inum, groundTruth);



    	cv::Mat rotationFromEig;
        cv::eigen2cv(rotation,rotationFromEig);


        //old code - expecting rad maybe
        //    	  cv::Mat rotationCv;


        cv::Mat rotGrid = cv::Mat::zeros(1,3,CV_32F);
        cv::Vec3f Rg(0, 0, M_PI/4);
        cv::Vec3f null(0, 0,0);
        cv::Vec3f Tg(-103, 103, 0);
        cv::Vec3f Tcircle(-30, 75, 0);
        cv::Mat rot_mat;
        cv::Rodrigues(Rg, rot_mat);
        cv::Affine3f affineGrid(rot_mat,Tg);
        cv::Affine3f affineCircle(null,Tcircle);





        visualizer.setWidgetPose("Ground grid circle"        +inum,affineGrid);
        visualizer.setWidgetPose("Ground truth buoy"        +inum,affineCircle);

        //        cv::Rodrigues(rvec, rotationCv);
        //        tvec.at<float>(2)=-tvec.at<float>(2);




    

    	cv::Affine3f poseCv(rotationFromEig, tvec);
    	visualizer.setWidgetPose("Camera"        +inum,poseCv);
	    visualizer.setWidgetPose("Line"          +inum, poseCv);






        }
	}

    if(debug=1){
	    // visualization loop
	    while(cv::waitKey(100)==-1 && !visualizer.wasStopped())
        {

            visualizer.spinOnce(0.1,     // pause 1ms 
			                    true); // redraw
        }
    }




    return 0;
}
