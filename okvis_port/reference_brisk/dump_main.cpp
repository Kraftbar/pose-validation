// SPDX-License-Identifier: MIT
// Drives real pinned OKVIS2 camera code and BRISK; no algorithm replacement.
#include <opencv2/opencv.hpp>
#include <fstream>
#include <filesystem>
#include <iostream>
#include <cstdio>
#include <cstring>
#include <brisk/brisk.h>
#include <okvis/cameras/PinholeCamera.hpp>
#include <okvis/cameras/RadialTangentialDistortion.hpp>
static FILE *dump=nullptr;
void ok_trace(const char *tag,const void *data,size_t bytes){
 if(!dump)return;char name[32]={0};std::strncpy(name,tag,31);uint32_t n=bytes;
 if(fwrite(name,32,1,dump)!=1||fwrite(&n,4,1,dump)!=1||(bytes&&fwrite(data,bytes,1,dump)!=1))throw std::runtime_error("write");
}
class Extractor:public brisk::BriskDescriptorExtractor {
public:Extractor():BriskDescriptorExtractor(true,false){}
 void pattern(){float log2=0.693147180559945;float lb=log(30.f)/log2;float b06=12.f*0.6;int s=int(64/lb*(log(1.45*12.f/b06)/log2)+0.5);ok_trace("scale",&s,4);ok_trace("border",sizeList_+s,4);ok_trace("pattern",patternPoints_+s*1024*points_,1024*points_*12);ok_trace("short_pairs",shortPairs_,noShortPairs_*8);ok_trace("long_pairs",longPairs_,noLongPairs_*16);}
};
int main(int argc,char**argv){try{
 if(argc<5){std::cerr<<"dump_brisk <outdir> <camera 0|1> <mode 0 plain|1 gravity|2 axial> <images...>\n";return 2;}
 cv::setNumThreads(1);cv::setUseOptimized(true);std::filesystem::create_directories(argv[1]);int cam=atoi(argv[2]),mode=atoi(argv[3]);
 double intr[2][8]={{458.654880721,457.296696463,367.215803962,248.37534061,-0.28340811217,0.0739590738929,0.000193595028569,1.76187114545e-05},{457.587426604,456.13442556,379.99944652,255.238185386,-0.283683654496,0.0745128430929,-0.000104738949098,-3.55590700274e-05}};
 auto p=intr[cam];using namespace okvis::cameras;PinholeCamera<RadialTangentialDistortion> camera(752,480,p[0],p[1],p[2],p[3],RadialTangentialDistortion(p[4],p[5],p[6],p[7]));
 cv::Mat rays,jacs;if(mode){camera.initialiseCameraAwarenessMaps();camera.getCameraAwarenessMaps(rays,jacs);std::string mp=std::string(argv[1])+"/cam"+argv[2]+".maps";FILE*f=fopen(mp.c_str(),"wb");fwrite(rays.data,rays.total()*12,1,f);fwrite(jacs.data,jacs.total()*24,1,f);fclose(f);}
 Extractor extractor;cv::Vec3f dir=mode==2?cv::Vec3f(0,0,-1):cv::Vec3f(.0148655429818f,-.999880929698f,.00414029679422f); // gravity for a controlled camera pose
 extractor.setExtractionDirection(dir);if(mode)extractor.setCameraProperties(rays,jacs,float(p[0]));
 std::string pattern=std::string(argv[1])+"/pattern.bin";dump=fopen(pattern.c_str(),"wb");extractor.pattern();fclose(dump);dump=nullptr;
 for(int i=4;i<argc;i++){
  cv::Mat im=cv::imread(argv[i],cv::IMREAD_GRAYSCALE);if(im.cols!=752||im.rows!=480)throw std::runtime_error("Expected EuRoC 752x480");
  std::string path=std::string(argv[1])+"/"+std::filesystem::path(argv[i]).stem().string()+"_m"+argv[3]+".bin";dump=fopen(path.c_str(),"wb");
  int hdr[4]={im.cols,im.rows,mode,cam};fwrite(hdr,sizeof(hdr),1,dump);fwrite(dir.val,12,1,dump);float focal=p[0];fwrite(&focal,4,1,dump);fwrite(im.data,im.total(),1,dump);
  brisk::ScaleSpaceFeatureDetector<brisk::HarrisScoreCalculator> detector(38,0,150,700);std::vector<cv::KeyPoint> kp;cv::Mat desc;detector.detect(im,kp);ok_trace("detected",kp.data(),kp.size()*sizeof(cv::KeyPoint));extractor.compute(im,kp,desc);ok_trace("described",kp.data(),kp.size()*sizeof(cv::KeyPoint));ok_trace("descriptors",desc.data,desc.total());fclose(dump);dump=nullptr;std::cout<<path<<" "<<kp.size()<<"\n";
 }
}catch(const std::exception&e){std::cerr<<e.what()<<"\n";return 1;}}
