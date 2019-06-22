#pragma once


# include "../utils/display.h"



# include <Eigen/Core>
# include <math.h>       /* ceil */
// clean this?
# include "../helpers/localMin.h"
# include "../helpers/dbscan.h"
# include "../helpers/helper.h"
 class GridMap 
{

    public: 
  
        GridMap( double cellSize, int gridSize,Eigen::Vector3f usv, double cage_r){

            int roundedgridSize=(int)gridSize;
            // if not pair number -> abort()
            _cellSize=cellSize;
            _gridSize=gridSize;
            if ( _gridSize % 2 == 1){
                   std::cerr <<"Grid size needs to be even \n";
                   std::terminate();
           }
            
            _halfgridSize=_gridSize/2;
            _cage_r      = cage_r;
            _usv         = usv;
            _gridMap_occ = Eigen::MatrixXf::Zero(roundedgridSize,roundedgridSize);
            _gridMap_obs_b = Eigen::MatrixXf::Zero(roundedgridSize,roundedgridSize);
            _gridMap_obs_c = Eigen::MatrixXf::Zero(roundedgridSize,roundedgridSize);


        }






       std::vector<int> ne2rc(float p_x,float p_y){
            // 1. To cell units
            float n_g=p_x/ _cellSize; // [cell] unit
            float e_g=p_y/ _cellSize; 

            // 2  To cell
            n_g=detCell(n_g);
            e_g=detCell(e_g);



            // 3 
            int row =      _halfgridSize - n_g;
            int col = -1 + _halfgridSize + e_g;
            // 4 
            std::vector<int> ret;
            ret.push_back(row);
            ret.push_back(col);
            return ret;
        }




        // returning Midpoint of cell
        std::vector<float> rc2ne(int row, int col){

            float n_g = -row  + _halfgridSize   ;
            float e_g =  col  - _halfgridSize +1;

            // undefined for 0? -> lets hope not
            float p_x = n_g*_cellSize -_cellSize/2;
            float p_y = e_g*_cellSize -_cellSize/2;
            std::vector<float> ret;
            ret.push_back(p_x);
            ret.push_back(p_y);
            return ret;
        }




        // ---------- Adding obs start ----------

        void addObs_b(std::vector<Eigen::Vector3f > X_n){

            for(int j=0; j<X_n.size(); j++){
                _X_n_b.push_back(X_n[j]);
                _pontCloud_b.push_back(X_n[j]);
            }
        }
        void addObs_b(Eigen::Vector3f  X){

                _X_n_b.push_back(X);
                _pontCloud_b.push_back(X);
        }


        void addObs_c(std::vector<Eigen::Vector3f > X_n){

            for(int j=0; j<X_n.size(); j++){
                _X_n_c.push_back(X_n[j]);
                _pontCloud_c.push_back(X_n[j]);
            }
        }
        void addObs_c(Eigen::Vector3f  X){

                _X_n_c.push_back(X);
                _pontCloud_c.push_back(X);
        }

        // ---------- Adding obs END   ----------





        float detCell(float  num){
                return ceil(num);
        }





        



        void updateMap( cv::viz::Viz3d tempVis){
            if(_X_n_b.size()==0){
                std::cout<<"Warning: no new points to update."<<std::endl;
            }else{
                for(int j=0; j<_X_n_b.size(); j++){
                    auto  X      = _X_n_b[j];
                    auto  ne_g = ne2rc(X(0),X(1)); 

                     _gridMap_obs_b(ne_g[0],ne_g[1])=_gridMap_obs_b(ne_g[0],ne_g[1]) +1;
                    // test visualise
                    auto  X_ned=rc2ne(ne_g[0],ne_g[1]);
//                    paintGrid(X_ned[0],X_ned[1],tempVis, _cellSize);
                    // test visualise
                }
                for(int j=0; j<_X_n_c.size(); j++){
                    auto  X      = _X_n_c[j];
                    auto  ne_g = ne2rc(X(0),X(1)); 

                     _gridMap_obs_c(ne_g[0],ne_g[1])=_gridMap_obs_c(ne_g[0],ne_g[1])+1;
                    // test visualise
                    auto  X_ned=rc2ne(ne_g[0],ne_g[1]);
//                    paintGrid(X_ned[0],X_ned[1],tempVis, _cellSize);
                    // test visualise
                }
                std::cout<<"Updated gridmap and did not clear observations:"<<std::endl;

            }
    // minmax 
        std::cout<<_gridMap_obs_b<<std::endl;
        std::cout<<std::endl;
        _gridMap_obs_b=imregionalmin(_gridMap_obs_b);
        _gridMap_obs_c=imregionalmin(_gridMap_obs_c);
        auto minmax=_gridMap_obs_b-_gridMap_obs_c;
                std::cout<<minmax<<std::endl;


    // minmax paint
        for(int i = 0;i<minmax.rows(); i++){
        for(int j = 0;j<minmax.cols(); j++){
            auto nedMinMax=rc2ne(i,j);
//            if(minmax(i,j)==1)  
//                    paintGrid(nedMinMax[0],nedMinMax[1],tempVis, _cellSize,1);
//            if(minmax(i,j)==-1)
//                    paintGrid(nedMinMax[0],nedMinMax[1],tempVis, _cellSize,2);
        }
        }



    // rays:
        std::vector<std::vector<int>> polylines=
        {   {-90,306 },
//            {-66,186 },
             {-106-8,226+8 },
            {-198,186},
            {-106-8,226 +8}
         }   ;

       int intersect=0;

        for(auto polyline : polylines){
            auto nedMinMax=rc2ne(polyline[0],polyline[1]);
            
  paintGrid(polyline[0],polyline[1],tempVis, _cellSize,2);
        }

        for(int i = 0;i<minmax.rows(); i++){
        for(int j = 0;j<minmax.cols(); j++){
            auto nedMinMax=rc2ne(i,j);  

            int lol1=geeks::doIntersect(polylines[0][0],polylines[0][1],
                                      polylines[1][0],polylines[1][1],
                                          nedMinMax[0],nedMinMax[1],
                                            _usv(0),_usv(1));
            int lol2=geeks::doIntersect(polylines[2][0],polylines[2][1],
                                      polylines[3][0],polylines[3][1],
                                          nedMinMax[0],nedMinMax[1],
                                            _usv(0),_usv(1));
            if(lol1 || lol2)  
                    paintGrid(nedMinMax[0],nedMinMax[1],tempVis, _cellSize,1);
/*
            if(minmax(i,j)==-1)
                    paintGrid(nedMinMax[0],nedMinMax[1],tempVis, _cellSize,2);
*/


        }
        }

        // Line(15,15,-75,75,visualizer);






        std::cout<<std::endl;
/*    
*/

/*    
        dbscan::DBCAN dbScan(13,1,_pontCloud_b);
        dbScan.run();
        auto lol=dbScan.getCluster();
         for(auto l : lol)
             std::cout<<l.size()<<std::endl; 
*/
       }


    // finish this


        void genPolyline(){
/*    

          int check=0;
            for (auto cage : _pontCloud_c){
                check=0;
                auto dist_uc=cage-_usv;
                for (auto buoy : _pontCloud_b){
                        auto dist_ub=buoy-_usv;
                        if( (dist_ub.norm() <dist_uc.norm()) 
                           &&      _cage_r*2.2>(cage-buoy).norm() )
                                {
                                    check++;
                                }
                    }
                    std::cout<<"Number of (polylinepoints): "<<cage.transpose()<<" "<<check<<std::endl;
               }

*/
        }


    void traverseMap_genPolyPoints(int pos_usv){


       if(polyPoints_c.size()<1){
        std::cout<<"To few mesuments \n";
        return;
        }


     }

    


    private:    
        // [m/cell]
        int                     _gridSize;
        // row/ col m/n 
        int                     _cellSize; 
        int                 _halfgridSize;
        cv::viz::Viz3d         visualizer;
        Eigen::MatrixXf      _gridMap_occ;
        Eigen::MatrixXf      _gridMap_obs_b;
        Eigen::MatrixXf      _gridMap_obs_c;
        Eigen::MatrixXf      _gridMap_obs_locmax;
        std::vector<Eigen::Vector3f> _X_n_b;  // temporary obs
        std::vector<Eigen::Vector3f> _X_n_c;  // temporary obs
        std::vector<Eigen::Vector3f> _pontCloud_b; // obs
        std::vector<Eigen::Vector3f> _pontCloud_c;
           
        // 
        double                            _cage_r;
    	Eigen::Vector3f                      _usv;
    	std::vector<Eigen::Vector3f> polyPoints_b;
    	std::vector<Eigen::Vector3f> polyPoints_c;

}; 
