
#include <stdio.h>


// recorces
// https://dsp.stackexchange.com/questions/1881/camera-calibration-pin-hole-camera-model-and-working-out-3d-position
// https://stackoverflow.com/questions/7836134/get-3d-coordinates-from-2d-image-pixel-if-extrinsic-and-intrinsic-parameters-are/10750648
// http://cvrs.whu.edu.cn/downloads/ebooks/Multiple%20View%20Geometry%20in%20Computer%20Vision%20(Second%20Edition).pdf (p 196)

// csv
# include <fstream>
# include <algorithm>

//  rotate
# include <opencv2/opencv.hpp>
# include <Eigen/Core>

// inverse
# include <Eigen/Dense>
// psudo inv
# include <Eigen/QR>    


// camera param

# include <opencv2/core/eigen.hpp>
# include <boost/algorithm/string.hpp>

// triangulate
# include "utils/triangulate.h"

# include "utils/triangulateOrb.h"


// get Kps
# include "utils/getKeypoints.h"
// read data
# include "utils/readcsv.h"

// convert 
# include "utils/conversions.h"

// display (uses convertions)
# include "utils/display.h"


# include "utils/bresenham2D.h"


// gridMap 
# include "classes/GridMap.h"

// drawing 
# include "classes/Observations.h"

// intersection
# include "helpers/helper.h"



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




int main(int argc, char** argv )
{
    // read images
    auto imgList=readImgs("../../*.png");

    double boy_r  = 7;  // 60; // radius ~5 px
    double cage_r = 73; // 16571.46; // radius ~75 px

    auto KpImgs_b = cotour_filter(imgList,boy_r);
    auto KpImgs_c = cotour_filter(imgList,cage_r);




    // read cam param
	cv::Mat K_cv;
	cv::Mat cameraDistCoeffs;
	cv::FileStorage fs("../../../../camera.xml", cv::FileStorage::READ);
	fs["camera_matrix"] >> K_cv;
	fs["distortion_coefficients"] >> cameraDistCoeffs;


    // correct param
	cv::Size sz = imgList[0].size();
    K_cv.at<double>(0, 2) = sz.width/2;
	K_cv.at<double>(1, 2) = sz.height/2;


  // for dist
  cv::Mat new_camera_matrix= getOptimalNewCameraMatrix(K_cv, cameraDistCoeffs, sz, 1,sz, 0);


  // K to eigen
  Eigen::Matrix3f K;
  cv::cv2eigen(K_cv,K);    





    std::vector<Eigen::Vector3f> r_log;
    std::vector<Eigen::Vector3f> t_log;
    // Filling t_log and r_log
    char const *file="../../attPos_log.csv";
    readCsv (file,r_log, t_log);


    Eigen::Vector3f Kp1(373 , 154,1);
    Eigen::Vector3f Kp2(430 , 135,

 1);
    Eigen::Vector3f Kp3(394 , 286,

    1);
    Eigen::Vector3f Kp4(408 , 258,


    1);
    Eigen::Vector3f Kp5(409 , 271,

    1);



Eigen::Vector3f Kps []={Kp1,Kp2,Kp3,Kp4,Kp5};
std::vector<Eigen::Vector3f> Kps_buoy_wc;
std::vector<Eigen::Vector3f> Kps_cage_wc;



// ----- TESTING

int cellsize_mm=12;
int gridsize=60;
Eigen::Vector3f pos_usv(-220,330, 0);

GridMap gridMap(cellsize_mm,gridsize,pos_usv,cage_r);

cv::Vec3f pos_usv_cv;
cv::eigen2cv(pos_usv,pos_usv_cv);
// ----- TESTING



// Distorted mode

int debug =1;







cv::viz::Viz3d visualizer("Debug");
visualizer.setBackgroundColor(cv::viz::Color::white());
        

std::vector<cv::Mat> g_n;
std::vector<cv::Mat> m_n;

int p1=3;
int p2=4;
for (int i = 0; i < imgList.size(); i++) {

        auto r = r_log[i];
        auto t = t_log[i];
        auto p_1 = Kps[i];
        auto p_n_b=KpImgs_b[i]; // Image prosessing
        auto p_n_c=KpImgs_c[i]; // Image prosessing
        // NED
        t(2)=-t(2);
        std::cout<<std::endl;

        

        // Getting rotation matrix
        Eigen::Matrix3f R; 
        R                 << getRotationMat(r); // DRONE TO CAMERAFRAME

        // Rigid-body motion
        Eigen::MatrixXf g = Eigen::MatrixXf::Identity(4, 4);
        g                 << R,t ,0,0,0,1;

        // Projection matrix        
        Eigen::MatrixXf P = K*g.block<3,4>(0,0);


        // Normal to plane
        Eigen::Vector3f N;
        N               <<0,0,-1;


        // Kp in focal plane
        Eigen::Vector3f m             = K.inverse()*p_1;
        Eigen::Vector3f m_wc=R*m+t;



        // 1. Assume Plane - Eq 8.1, Hartley and Zisserman   - cyan
        Eigen::MatrixXf H(3,3);
        H                               <<  R.col(0), R.col(1), t;    
        Eigen::MatrixXf H_norm          =   H;   
        Eigen::MatrixXf projection      =   H * m;   
        Eigen::VectorXf X1              =   projection;
        




        // 2. Intersect plane
        Eigen::VectorXf abc        = m_wc-t;
        Eigen::Vector3f X2 ;
                        X2 << t(0)- abc(0)*t(2),   
                              t(1)- abc(1)*t(2),   
                              0;



    // image pros
        auto intersectPlane = [K,R,t](double kp_u, double kp_v) {
            Eigen::Vector3f p_tmp      (kp_u , kp_v,  1);
            Eigen::VectorXf abc_tmp  = (R*K.inverse()*p_tmp)*t(2);
            Eigen::Vector3f Xn_tmp ;
                             Xn_tmp << t(0)- abc_tmp(0),   
                                       t(1)- abc_tmp(1),   
                                       0;
            return Xn_tmp; 
        };


        for (auto kp : p_n_b) {
            auto p_n_b_wc=intersectPlane(kp[0],kp[1]);
            Kps_buoy_wc.push_back(p_n_b_wc);
        }
        for (auto kp : p_n_c) {
            auto p_n_c_wc=intersectPlane(kp[0],kp[1]);
            Kps_cage_wc.push_back(p_n_c_wc);

        }
    // image pros




        // 3. Project   rays
        Eigen::VectorXf X3_c          = m*(-t(2));
        Eigen::Vector3f X3            = R*X3_c+t;


        // ---- Testing -----
 
       gridMap.addObs_b(Kps_buoy_wc);
       gridMap.addObs_c(Kps_cage_wc);
        displayKps(visualizer,i,Kps_buoy_wc);
        displayKps(visualizer,i*10,Kps_cage_wc);
        Kps_buoy_wc.clear();
        Kps_cage_wc.clear();
            
        // ---- Testing -----
        





            std::cout<<std::endl;

            // 1.
            cv::Point3d                testpoint1(X1(0),X1(1),X1(2));
	        cv::viz::WSphere           point3Dm1(testpoint1, 2, 30, cv::viz::Color::cyan());
            
            // 2.
            cv::Point3d                testpoint2(X2(0),X2(1),X2(2));
	        cv::viz::WSphere           point3Dm2(testpoint2, 2, 30, cv::viz::Color::pink());

            //3
            cv::Point3d                testpoint3(X3(0),X3(1),0);
	        cv::viz::WSphere           point3Dm3(testpoint3 , 0.001, 30, cv::viz::Color::brown());



      

            
            cv::Mat r_cv, t_cv;
            cv::eigen2cv(r,r_cv);
            cv::eigen2cv(t,t_cv);
        	cv::Mat R_cv;
            cv::eigen2cv(R,R_cv);
        	cv::Affine3f T_cv(R_cv, t_cv);




            // for compatibility
            cv::Matx33d K_cvVis=K_cv;
            cv::viz::WCameraPosition   cam( K_cvVis,  imgList[i], 30.0,cv::viz::Color::blue() );


            cv::Point3d ray(m(0),m(1),m(2));
            ray=ray*(-t(2)+60);
        	cv::viz::WLine             line(cv::Point3d(0., 0., 0.),ray, cv::viz::Color::green());

            

            std::string inum=std::to_string(i);





            visualizer.showWidget("Camera"+inum, cam);


//	        visualizer.showWidget("Point method 1"+inum, point3Dm1);
//	        visualizer.showWidget("Point method 2"+inum, point3Dm2);
//	        visualizer.showWidget("Point method 3"+inum, point3Dm3);
 

          	visualizer.setWidgetPose("Camera"     +inum, T_cv);







      // ------------------- TESTING triang ------------------- 


 //      cv::Mat P_cv(3,4,CV_32F);
        cv::Mat Tcw(3,4,CV_32F);
        R_cv.copyTo(Tcw.colRange(0,3));
        t_cv.copyTo(Tcw.col(3));

        cv::Mat Rwc = R_cv.t();
        cv::Mat Ow = -Rwc*t_cv;
        cv::Mat Twc = cv::Mat::eye(4,4,Tcw.type());
        Rwc.copyTo(Twc.rowRange(0,3).colRange(0,3));
        Ow.copyTo(Twc.rowRange(0,3).col(3));

//        cv::eigen2cv(P,P_cv);

        cv::Mat xn = (cv::Mat_<float>(3,1) << m(0),m(1), 1.0);

        g_n.push_back( Twc );
        m_n.push_back( xn );

      // ------------------- TESTING triang ------------------- 
 
    

    





	}

       // ------------------- TESTING ------------------- 
        // Line(15,15,-75,75,visualizer);
        gridMap.updateMap(visualizer);
       // ------------------- TESTING ------------------- 


        // ------------ Testing tiangulation

    	auto testPointt = triangulateOrb(g_n[p1], g_n[p2],m_n[p1], m_n[p2]);
        std::cout << "M = "<< std::endl << " "  << testPointt << std::endl << std::endl;
        cv::viz::WSphere point3DD(cv::Point3d(testPointt(0),testPointt(1), testPointt(2)), 2, 30, cv::viz::Color::brown());
		visualizer.showWidget("Triangulated", point3DD);

        // ------------ Testing tiangulation

 
    if(debug=1){ 
        cv::viz::WCoordinateSystem  coordinate(30);
        cv::viz::WGrid              a4(cv::Vec2i(1  , 1),cv::Vec2i(297*1.2  , 210*1.2),cv::viz::Color::gray());
        cv::viz::WPlane             usv(cv::Size2d(40  , 20),cv::viz::Color::gray()  );
        cv::viz::WCircle            groundTruth( 30,0.7 ,cv::viz::Color::gray()     );






        cv::viz::WGrid    gridCells(cv::Vec2i(gridsize  , gridsize),cv::Vec2i(cellsize_mm, cellsize_mm),cv::viz::Color::gray());


        visualizer.showWidget("Grid cells", gridCells);
        visualizer.showWidget("A4 Paper", a4);
        visualizer.showWidget("USV", usv);
        visualizer.showWidget("Coordinate Widget", coordinate);


        // clean this 
        cv::Vec3f Rg(0, 0, M_PI/4);
        cv::Vec3f t_zero(0, 0,0);
        cv::Vec3f Tg(-120,140, 0);


        cv::Mat   rot_mat;
        cv::Mat   rot_id = cv::Mat::eye(3,3,CV_32F);

        cv::Rodrigues(Rg, rot_mat);

        // https://docs.opencv.org/3.4/dd/d99/classcv_1_1Affine3.html
        cv::Affine3f affineA4(rot_mat,Tg);

        cv::Affine3f affineSUV(rot_mat,pos_usv_cv);



      visualizer.setWidgetPose("A4 Paper"   ,affineA4);
        visualizer.setWidgetPose("USV"        ,affineSUV);




        gridMap.genPolyline();    

	    // visualization loop
	    while(cv::waitKey(100)==-1 && !visualizer.wasStopped())
        {

            visualizer.spinOnce(0.1,     // pause 1ms 
			                    true); // redraw
        }
    }




    return 0;
}
