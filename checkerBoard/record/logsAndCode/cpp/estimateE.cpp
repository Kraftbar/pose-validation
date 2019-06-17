/*------------------------------------------------------------------------------------------*\
This file contains material supporting chapter 11 of the book:
OpenCV3 Computer Vision Application Programming Cookbook
Third Edition
by Robert Laganiere, Packt Publishing, 2016.

This program is free software; permission is hereby granted to use, copy, modify,
and distribute this source code, or portions thereof, for any purpose, without fee,
subject to the restriction that the copyright notice may not be removed
or altered from any source or altered source distribution.
The software is released on an as-is basis and without any warranties of any kind.
In particular, the software is not guaranteed to be fault-tolerant or free from failure.
The author disclaims all warranties with regard to this software, any use,
and any consequent failure, is purely the responsibility of the user.

Copyright (C) 2016 Robert Laganiere, www.laganiere.name
\*------------------------------------------------------------------------------------------*/

#include <iostream>
#include <vector>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/viz.hpp>
#include "utils/triangulate.h"



std::vector<cv::Mat> readImgs(const char *folder){


std::vector<cv::String> fn;
cv::glob(folder, fn, false);

std::vector<cv::Mat> images;
size_t count = fn.size(); //number of png files in images folder
for (size_t i=0; i<count; i++)
    images.push_back(imread(fn[i]));
    return images;
}



int main()
{

    std::cout<<"asd"<<std::endl;
    auto imgList=readImgs("../../*.png");
	cv::Mat image1= imgList[0];
	cv::Mat image2= imgList[1];
    

	// Read input images

	if (!image1.data || !image2.data)
		return 0; 
    
    // Display the images
	cv::namedWindow("Right Image");
	cv::imshow("Right Image",image1);
	cv::namedWindow("Left Image");
	cv::imshow("Left Image",image2);

	// Read the camera calibration parameters
	cv::Mat cameraMatrix;
	cv::Mat cameraDistCoeffs;
	cv::FileStorage fs("../../../../camera.xml", cv::FileStorage::READ);
	fs["camera_matrix"] >> cameraMatrix;
	fs["distortion_coefficients"] >> cameraDistCoeffs;

    // ajusting the camera params
	cv::Size sz = image1.size();
    cameraMatrix.at<double>(0, 2) = sz.width/2;
	cameraMatrix.at<double>(1, 2) =sz.height/2;




	cv::Matx33f cMatrix(cameraMatrix);

	// vector of keypoints and descriptors
	std::vector<cv::KeyPoint> keypoints1;
	std::vector<cv::KeyPoint> keypoints2;
	cv::Mat descriptors1, descriptors2;

	// Construction of the SIFT feature detector 
	cv::Ptr<cv::Feature2D> ptrFeature2D = cv::xfeatures2d::SIFT::create(500);

	// Detection of the SIFT features and associated descriptors
	ptrFeature2D->detectAndCompute(image1, cv::noArray(), keypoints1, descriptors1);
	ptrFeature2D->detectAndCompute(image2, cv::noArray(), keypoints2, descriptors2);

	std::cout << "Number of feature points (1): " << keypoints1.size() << std::endl;
	std::cout << "Number of feature points (2): " << keypoints2.size() << std::endl;

	// Match the two image descriptors

	// Construction of the matcher with crosscheck 
	cv::BFMatcher matcher(cv::NORM_L2, true);
	// matching
	std::vector<cv::DMatch> matches;



	matcher.match(descriptors1, descriptors2, matches);


    //// -------- good matches




    std::vector<cv::DMatch> good_matches;
    for (size_t i = 0; i < matches.size(); i++)
    {
		// Get the position of left keypoints
        if(matches[i].distance<50)
            good_matches.push_back(matches[i]);
	}
    matches =good_matches; 
    //// -------- good matches



	// draw the matches
	cv::Mat imageMatches;
	cv::drawMatches(image1, keypoints1,  // 1st image and its keypoints
		image2, keypoints2,  // 2nd image and its keypoints
		matches,			// the matches
		imageMatches,		// the image produced
		cv::Scalar(255, 255, 255),  // color of the lines
		cv::Scalar(255, 255, 255),  // color of the keypoints
		std::vector<char>(),
		2);
	cv::namedWindow("Matches");
	cv::imshow("Matches", imageMatches);

	// Convert keypoints into Point2f
	std::vector<cv::Point2f> points1, points2;
	for (std::vector<cv::DMatch>::const_iterator it = matches.begin();
	it != matches.end(); ++it) {

		// Get the position of left keypoints
		float x = keypoints1[it->queryIdx].pt.x;
		float y = keypoints1[it->queryIdx].pt.y;
		points1.push_back(cv::Point2f(x, y));
		// Get the position of right keypoints
		x = keypoints2[it->trainIdx].pt.x;
		y = keypoints2[it->trainIdx].pt.y;
		points2.push_back(cv::Point2f(x, y));
	}

	std::cout << "Number of matches: " << points2.size() << std::endl;






	// Find the essential between image 1 and image 2
	cv::Mat inliers;
	cv::Mat essential = cv::findEssentialMat(
		points1, points2, 
		cMatrix,	          // intrinsic parameters 
		cv::RANSAC, 0.8, 1.2, // RANSAC method  
		inliers);             // extracted inliers

	int numberOfPts(cv::sum(inliers)[0]);
	std::cout << "Number of inliers: " << numberOfPts << std::endl;

	// draw the inlier matches
	cv::drawMatches(image1, keypoints1,  // 1st image and its keypoints
		image2, keypoints2,  // 2nd image and its keypoints
		matches,			// the matches
		imageMatches,		// the image produced
		cv::Scalar(255, 255, 255),  // color of the lines
		cv::Scalar(255, 255, 255),  // color of the keypoints
		inliers,
		2);
	cv::namedWindow("Inliers matches");
	cv::imshow("Inliers matches", imageMatches);

	// recover relative camera pose from essential matrix
	cv::Mat rotation, translation;
	cv::recoverPose(essential,   // the essential matrix
		points1, points2,        // the matched keypoints
		cameraMatrix,            // matrix of intrinsics 
		rotation, translation,   // estimated motion
		inliers);                // inliers matches


	std::cout << "translation:" << translation << std::endl;

	// compose projection matrix from R,T 
	cv::Mat projection2(3, 4, CV_64F);        // the 3x4 projection matrix
	rotation.copyTo(projection2(cv::Rect(0, 0, 3, 3)));
	translation.copyTo(projection2.colRange(3, 4));

	// compose generic projection matrix 
	cv::Mat projection1(3, 4, CV_64F, 0.);    // the 3x4 projection matrix
	cv::Mat diag(cv::Mat::eye(3, 3, CV_64F));
	diag.copyTo(projection1(cv::Rect(0, 0, 3, 3)));


	// to contain the inliers
	std::vector<cv::Vec2d> inlierPts1;
	std::vector<cv::Vec2d> inlierPts2;

	// create inliers input point vector for triangulation
	int j(0); 
	for (int i = 0; i < inliers.rows; i++) {

		if (inliers.at<uchar>(i)) {
			inlierPts1.push_back(cv::Vec2d(points1[i].x, points1[i].y));
			inlierPts2.push_back(cv::Vec2d(points2[i].x, points2[i].y));
		}
	}

	// undistort and normalize the image points TO X AND Y
	std::vector<cv::Vec2d> points1u;
    std::cout<< inlierPts1[0]<<std::endl;
	cv::undistortPoints(inlierPts1, points1u, cameraMatrix, cameraDistCoeffs);
   std::cout<< points1u[0][0]*646+320<<std::endl;
	std::vector<cv::Vec2d> points2u;
	cv::undistortPoints(inlierPts2, points2u, cameraMatrix, cameraDistCoeffs);




	for (int i = 0; i < points1u.size(); i++) {
            // std::cout<<points1u[i]<<std::endl;
	}


	// triangulation
	std::vector<cv::Vec3d> points3D;
	triangulate(projection1, projection2, points1u, points2u, points3D);





	// -------------------
	// Create a viz window
	cv::viz::Viz3d visualizer("Viz window");
	visualizer.setBackgroundColor(cv::viz::Color::white());

	/// Construct the scene
	// Create one virtual camera
	cv::viz::WCameraPosition cam1(cMatrix,  // matrix of intrinsics
		image1,                             // image displayed on the plane
		1.0,                                // scale factor
		cv::viz::Color::black());
	// Create a second virtual camera
	cv::viz::WCameraPosition cam2(cMatrix,  // matrix of intrinsics
		image2,                             // image displayed on the plane
		1.0,                                // scale factor
		cv::viz::Color::black());

	// choose one point for visualization
	cv::Vec3d testPoint = triangulate(projection1, projection2, points1u[124], points2u[124]);
	cv::viz::WSphere point3D(testPoint, 0.05, 10, cv::viz::Color::red());
	// its associated line of projection
	double lenght(4.);
	cv::viz::WLine line1(cv::Point3d(0., 0., 0.), cv::Point3d(lenght*points1u[124](0), lenght*points1u[124](1), lenght), cv::viz::Color::green());
	cv::viz::WLine line2(cv::Point3d(0., 0., 0.), cv::Point3d(lenght*points2u[124](0), lenght*points2u[124](1), lenght), cv::viz::Color::green());

	
    // the reconstructed cloud of 3D points
	cv::viz::WCloud cloud(points3D, cv::viz::Color::blue());
	cloud.setRenderingProperty(cv::viz::POINT_SIZE, 3.);
	// Add the virtual objects to the environment
	visualizer.showWidget("Camera1", cam1);
	visualizer.showWidget("Camera2", cam2);
	visualizer.showWidget("Cloud", cloud);

	visualizer.showWidget("Line1", line1);
	visualizer.showWidget("Line2", line2);
	visualizer.showWidget("Triangulated", point3D);

	// Move the second camera	
	cv::Affine3d pose(rotation, translation);
	visualizer.setWidgetPose("Camera2", pose);

	visualizer.setWidgetPose("Line2", pose);

	// visualization loop
	while (cv::waitKey(100) == -1 && !visualizer.wasStopped())
	{
		visualizer.spinOnce(1,     // pause 1ms 
			                true); // redraw
	}
	
	cv::waitKey();
	return 0;
}
