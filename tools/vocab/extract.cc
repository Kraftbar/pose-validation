// SPDX-License-Identifier: MIT
// Uses the public stella extractor API; no vocabulary is loaded.
#include <stella_vslam/feature/orb_extractor.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <fstream>
#include <iostream>
#include <sstream>
#include <cstdint>
int main(int argc,char**argv){try{
 if(argc<3)throw std::runtime_error("extract output descriptors-manifest");
 std::ifstream list(argv[2]);std::ofstream out(argv[1],std::ios::binary);if(!list||!out)throw std::runtime_error("file open");
 out.write("SVORBD01",8);uint32_t documents=0,count=0;out.write((char*)&documents,4);out.write((char*)&count,4);
 stella_vslam::feature::orb_params params("training",1.2f,8,20,7);
 stella_vslam::feature::orb_extractor extractor(&params,800);
 std::string path;
 while(std::getline(list,path)){
  auto rgb=cv::imread(path,cv::IMREAD_COLOR);if(rgb.empty())throw std::runtime_error("unreadable "+path);
  cv::Mat gray,desc;cv::cvtColor(rgb,gray,cv::COLOR_BGR2GRAY);std::vector<cv::KeyPoint>kp;extractor.extract(gray,cv::Mat(),kp,desc);
  if(desc.type()!=CV_8UC1||desc.cols!=32)throw std::runtime_error("wrong ORB descriptor layout");
  // Uniform deterministic subsampling limits one image's contribution.
  unsigned n=std::min(1200,desc.rows);
  for(unsigned j=0;j<n;j++){unsigned i=(uint64_t)j*desc.rows/n;out.write((char*)&documents,4);out.write((char*)desc.ptr(i),32);count++;}
  documents++;if(documents%100==0)std::cout<<documents<<" images, "<<count<<" descriptors\n"<<std::flush;
 }
 if(!list.eof()||!documents)throw std::runtime_error("empty or invalid image list");
 out.seekp(8);out.write((char*)&documents,4);out.write((char*)&count,4);if(!out)throw std::runtime_error("write failed");
 std::cout<<documents<<" images, "<<count<<" descriptors complete\n";
 }catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
