// SPDX-License-Identifier: MIT
#include <opencv2/opencv.hpp>
#include "m7b_io.hpp"
#include <vector>
#include <string>
#include <limits>
using namespace cv;
static void dump(const Mat &raw,const double *K,const double *D,int fish,int custom) {
 uint32_t hdr[4]={(uint32_t)raw.cols,(uint32_t)raw.rows,(uint32_t)fish,(uint32_t)custom};wr(hdr,sizeof(hdr));
 rec("K",K,32);rec("D",D,32);mat("input",raw);
 Mat a,b,out;
 Mat km=(Mat_<double>(3,3)<<K[0],0,K[2],0,K[1],K[3],0,0,1),dm(1,4,CV_64F,(void*)D);
 if(fish)fisheye::initUndistortRectifyMap(km,dm,Mat::eye(3,3,CV_64F),km,raw.size(),CV_32FC1,a,b);
 else initUndistortRectifyMap(km,dm,Mat(),km,raw.size(),CV_32FC1,a,b);
 if(custom)for(int y=0;y<a.rows;y++)for(int x=0;x<a.cols;x++) {
   size_t i=(size_t)y*a.cols+x;
   float specials[]={-1.f,-.5f,-1.f/64,0.f,1.f/64,3.f/64,32767.f,1e20f,-1e20f,std::numeric_limits<float>::infinity(),std::numeric_limits<float>::quiet_NaN()};
   a.at<float>(y,x)=i%3?((int)(i%130)-35)/32.f:specials[i%11];
   b.at<float>(y,x)=i%2?((int)(i%67)-5)/64.f:specials[(i/11)%11];
 }
 mat(custom?"map_input1":"map1",a);mat(custom?"map_input2":"map2",b);
 remap(raw,out,a,b,INTER_LINEAR);mat("remap",out);
}
int main(int argc,char **argv) {try {
 if(argc!=3)return 2;setNumThreads(1);m7b_out=fopen(argv[2],"wb");if(!m7b_out)return 2;
 wr("RDUND01\0",8);uint32_t state=123;
 double Ks[5][4]={{458.654,457.296,367.215,248.375},{340.125,310.75,319.5,239.5},{90.75,98.3,-13,21},{1,1,0,0},{340.125,310.75,319.5,239.5}};
 double Ds[5][4]={{-.28340811,.07395907,.00019359,1.76187114e-5},{-.01,.003,-.0002,.00001},{.12,-.017,.01,-.008},{0,0,0,0},{0,0,0,0}};
 for(int c=0;c<5;c++)for(int fish=0;fish<2;fish++)for(Size sz: {Size(752,480),Size(641,479),Size(7,5),Size(1,1)})for(int pattern=0;pattern<2;pattern++) {
   Mat raw(sz,CV_8U);for(int y=0;y<sz.height;y++)for(int x=0;x<sz.width;x++){state^=state<<13;state^=state>>17;state^=state<<5;raw.at<uchar>(y,x)=pattern?((x/4+y/4)%2)*255:state>>24;}
   dump(raw,Ks[c],Ds[c],fish,0);
 }
 FILE *f=fopen(argv[1],"rb");char magic[8];uint32_t h[3];
 if(!f||fread(magic,1,8,f)!=8||memcmp(magic,"OKGRAY1\0",8)||fread(h,4,3,f)!=3)throw std::runtime_error("gray header");
 for(int index: {0,1,100,700,1800,3000,3680,3681}) {
   Mat raw(h[1],h[0],CV_8U);fseek(f,20+(long)index*(8+(long)h[0]*h[1])+8,SEEK_SET);
   if(fread(raw.data,1,raw.total(),f)!=raw.total())throw std::runtime_error("gray read");
   for(int fish=0;fish<2;fish++)dump(raw,Ks[0],Ds[0],fish,0);
 }
 fclose(f);
 for(int w: {1,7,16,31,32,65,641,752}) {Mat raw(37,w,CV_8U);for(size_t i=0;i<raw.total();i++)raw.data[i]=(uchar)(i*13);dump(raw,Ks[0],Ds[0],0,1);}
 if(fclose(m7b_out))return 1;return 0;
 }catch(const std::exception &e){fprintf(stderr,"%s\n",e.what());return 1;}}
