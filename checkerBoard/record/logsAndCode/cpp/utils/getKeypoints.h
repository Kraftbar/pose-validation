#pragma once

std::vector<std::vector<cv::Vec2d> >  cotour_filter(std::vector<cv::Mat> images, double r){ 
    double const pi = 3.14159265;
    int areaEst=r*r*pi;
    int area_h=areaEst+areaEst*0.7;
    int area_l=areaEst-areaEst*0.7;    

    std::vector<std::vector<cv::Vec2d> > KpCentImages ;
    cv::Mat image;
    cv::Mat imageg;
    for (size_t i=0; i<images.size(); i++){


        // make binary
        cv::cvtColor(  images[i],image, cv::COLOR_BGR2GRAY );
        cv::blur(  image,  image, cv::Size(3,3) );
        cv::Canny( image, image, 140, 140*2, 3 );




        // find contours
        std::vector<cv::Vec4i> hierarchy;
        std::vector<std::vector<cv::Point> > contours;
        std::vector<std::vector<cv::Point> > filteredBlobs;
        cv::findContours( image, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);



        std::vector<cv::Vec2d> kpImgcenters;

            for(int j = 0; j < contours.size(); j++)
            {
              if (hierarchy[j][2] == -1)  //has parent, inner (hole) contour of a closed edge (looks good)
                {
                // calculate circularity
                double area_cont = cv::contourArea(contours[j]);
                double arclength = cv::arcLength(contours[j], true);
                double circularity = 4 * M_PI * area_cont / (arclength * arclength);
                if((circularity > 0.8) && (area_cont>area_l) && (area_cont<area_h) )
                {
                       filteredBlobs.push_back(contours[j]); // error

                        // get center moment
                        cv::Moments mu =  cv::moments( contours[j], false );                          
                         // get the centroid of figures.




                        kpImgcenters.push_back( cv::Vec2d( mu.m10/mu.m00 , mu.m01/mu.m00 ));
                        if(r>10)
                            std::cout << cv::Vec2d( mu.m10/mu.m00 , mu.m01/mu.m00 ) << std::endl;
                        


                }
               }

            }

         if(contours.size() == 0)
        {
            printf("no contours found");
         continue;
        }




        KpCentImages.push_back(kpImgcenters);
//        drawContours(images[i], filteredBlobs, -1, cv::Scalar(0,0,255), CV_FILLED, 8);





         
    }
    return KpCentImages ;
}

