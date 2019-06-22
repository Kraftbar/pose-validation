#pragma once


#include "conversions.h"

void displayKps (cv::viz::Viz3d visualizer,int i, std::vector<Eigen::Vector3f> Kps_imagePros_wc){



            
            // ----------- testing
            for(int j=0; j<Kps_imagePros_wc.size(); j++){
                std::string inum_n=std::to_string(i*100+j+1);
                cv::Point3d                testpoint4(Kps_imagePros_wc[j](0),Kps_imagePros_wc[j](1),Kps_imagePros_wc[j](2));
	            cv::viz::WSphere           point3Dm4(testpoint4 , 2, 30, cv::viz::Color::pink());

	            visualizer.showWidget("Point method 4"+inum_n, point3Dm4);
            }
            // ----------- testing


}



// find  a better way
int BADGLOBAL=0;
void paintGrid(float x, float y,cv::viz::Viz3d visualizer ,int cellSize, int color=0){

        cv::viz::WPlane            notNull;


    if(color==0)
{        cv::viz::WPlane            gridCell(cv::Size2d(cellSize, cellSize),cv::viz::Color::red()     );
            notNull=gridCell;
}    if(color==1)
{        cv::viz::WPlane            gridCell(cv::Size2d(cellSize, cellSize),cv::viz::Color::black()     );
            notNull=gridCell;
}    if(color==2){
        cv::viz::WPlane            gridCell(cv::Size2d(cellSize, cellSize),cv::viz::Color::yellow()   );
            notNull=gridCell;
        }


        
        std::string badInum      = std::to_string(BADGLOBAL);
 
        BADGLOBAL++;

        visualizer.showWidget("Cel"+badInum ,notNull);

    
        cv::Vec3f Rg(0, 0, 0);
        cv::Vec3f Tg(x, y, 0);
        cv::Mat rot_mat;
        cv::Rodrigues(Rg, rot_mat);
        cv::Affine3f affineCircle(rot_mat,Tg);
        visualizer.setWidgetPose("Cel"+badInum   ,affineCircle);


}




void paintCellWithMap(float x, float y,cv::viz::Viz3d visualizer ,int cellSize){




}

