#pragma once


# include <Eigen/Core>

 class Observations 
{
    public:   


    void addObsOfFrame( std::vector<Eigen::Matrix3f> p_n){
            for(int j=0; j<p_n.size(); j++){
                _points3D.push_back(p_n[j]);
            }
    }
    
    std::vector<Eigen::Matrix3f> getObs(){
       return _points3D;
    }

    private:    

    std::vector<Eigen::Matrix3f>    _points3D;
    


}; 
  
