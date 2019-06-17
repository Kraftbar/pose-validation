
std::vector<cv::Mat> readImgs(const char *folder){


    std::vector<cv::String> fn;
    cv::glob(folder, fn, false);

    std::vector<cv::Mat> images;
 
    for (size_t i=0; i<fn.size(); i++)
    images.push_back(imread(fn[i]));

    return images;
}



int  readCsv(const char *file,std::vector<Eigen::Vector3f> &logAtt_eigen,std::vector<Eigen::Vector3f> &logPos_eigen)
{   
    Eigen::Vector3f vecTmp(3);
    std::ifstream cFile (file);
    if (cFile.is_open())
    {
        std::string line;
        while(getline(cFile, line)){
            if(line[0] == '#' || line.empty())
                continue;
            std::vector<std::string> strs;
            boost::split(strs,line,boost::is_any_of(","));
            vecTmp(0)= -std::stod(strs[0]);
            vecTmp(1)=-std::stod(strs[1]);
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
