// SPDX-License-Identifier: MIT
// Observe-only M7 stream; copied into the isolated reference by patch 0008.
#pragma once
#include <opencv2/opencv.hpp>
#include <cstdio>
#include <cstdlib>
#include <cstring>
namespace rdvio { namespace extra { namespace portcv {
inline FILE *file() {
 static FILE *f=[](){const char *p=std::getenv("RDVIO_CV_STREAM");if(!p)return (FILE*)nullptr;
   FILE *q=std::fopen(p,"wb");if(!q)std::abort();return q;}();
 return f;
}
inline void raw(const void *p,size_t n) {if(n&&std::fwrite(p,1,n,file())!=n)std::abort();}
inline void record(const char *s,const void *p,size_t n) {
 char name[32]={0};std::strncpy(name,s,31);uint32_t z=(uint32_t)n;raw(name,32);raw(&z,4);raw(p,n);
}
inline void matrix(const char *s,const cv::Mat &m) {cv::Mat c=m.isContinuous()?m:m.clone();record(s,c.data,c.total()*c.elemSize());}
template<class T> inline void vector(const char *s,const std::vector<T>&v) {record(s,v.data(),v.size()*sizeof(T));}
inline void begin(uint32_t kind,const cv::Mat &image) {raw(&kind,4);uint32_t dims[2]={(uint32_t)image.cols,(uint32_t)image.rows};raw(dims,8);}
inline void done() {if(std::fflush(file()))std::abort();}
inline void preprocess(const cv::Mat &input,const cv::Mat &output,const std::vector<cv::Mat>&pyr) {
 if(!file())return;begin(1,input);matrix("input",input);matrix("clahe",output);
 uint32_t n=(uint32_t)pyr.size()/2;record("levels",&n,4);
 for(size_t i=0;i<pyr.size();i++) {cv::Mat a=pyr[i];a.adjustROI(21,21,21,21);matrix(i%2?"derivative_padded":"image_padded",a);}done();
}
inline void detect(const cv::Mat &image,int max,const std::vector<cv::KeyPoint>&a,const std::vector<cv::KeyPoint>&b) {
 if(!file())return;begin(2,image);matrix("input",image);record("max_points",&max,4);vector("gftt_harris",a);vector("rdvio_sorted",b);done();
}
inline void track_begin(const cv::Mat &a,const cv::Mat &b,const std::vector<cv::Point2f>&pts,const std::vector<cv::Point2f>&initial) {
 if(!file())return;begin(3,a);matrix("input",a);matrix("input_next",b);vector("points",pts);vector("initial_flow",initial);
}
inline void forward(const std::vector<cv::Point2f>&pts,const cv::Mat &s,const cv::Mat&e) {
 if(!file())return;vector("forward_points",pts);matrix("forward_status",s);matrix("forward_err",e);
}
inline void reverse(const std::vector<cv::Point2f>&pts,const std::vector<uchar>&s,const std::vector<float>&e,const std::vector<char>&status) {
 if(!file())return;vector("reverse_points",pts);vector("reverse_status",s);vector("reverse_err",e);vector("rdvio_status",status);done();
}
}}}
