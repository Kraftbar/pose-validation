
#pragma once



Eigen::Vector3f Vec2dToEig3 (cv::Vec2d Kp){
    Eigen::Vector3f ret(Kp(0) ,Kp(1),  1);
    return  ret;
}


