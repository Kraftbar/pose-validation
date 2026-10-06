// SPDX-License-Identifier: MIT
// Native OpenCV oracle: only calls real public functions; does not replace kernels.
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <cstdint>
#include <string>
#include <vector>
#include <stdexcept>
using namespace cv;
static FILE *out;
static void bytes(const void *p,size_t n) { if(n&&fwrite(p,1,n,out)!=n)throw std::runtime_error("write"); }
static void rec(const char *name,const void *p,size_t n) { char s[32]={0}; std::strncpy(s,name,31); uint32_t z=(uint32_t)n; bytes(s,32);bytes(&z,4);bytes(p,n); }
static void mat(const char *name,const Mat &m) { Mat c=m.isContinuous()?m:m.clone(); rec(name,c.data,c.total()*c.elemSize()); }
template<class T> static void vec(const char *s,const std::vector<T>&v) { rec(s,v.data(),v.size()*sizeof(T)); }
static void sentinel(std::vector<float>&e,size_t n) { uint32_t u=0x7fc12345; float f;memcpy(&f,&u,4);e.assign(n,f); }
static void case_dump(const std::string &path,const Mat &raw,const Mat &raw_next,int variant) {
 out=fopen(path.c_str(),"wb"); if(!out)throw std::runtime_error(path);
 bytes("RDCV001\0",8); uint32_t hdr[3]={(uint32_t)raw.cols,(uint32_t)raw.rows,(uint32_t)variant};bytes(hdr,12);
 mat("input",raw);mat("input_next",raw_next);
 Mat a,b;auto clahe=createCLAHE(6,Size(8,8));clahe->apply(raw,a);clahe->apply(raw_next,b);
 mat("clahe",a);mat("clahe_next",b);
 std::vector<Mat> pa,pb;buildOpticalFlowPyramid(a,pa,Size(21,21),3,true);buildOpticalFlowPyramid(b,pb,Size(21,21),3,true);
 uint32_t levels=(uint32_t)pa.size()/2;rec("levels",&levels,4);
 for(auto *py: {&pa,&pb}) for(size_t i=0;i<py->size();i++) {
   Mat expanded=(*py)[i];expanded.adjustROI(21,21,21,21);mat(i%2?"derivative_padded":"image_padded",expanded);
 }
 Mat dx,dy,cov(a.size(),CV_32FC3),harris,eigen;
 Sobel(a,dx,CV_32F,1,0,3,1.0/(4*3*255));Sobel(a,dy,CV_32F,0,1,3,1.0/(4*3*255));
 for(int y=0;y<a.rows;y++)for(int x=0;x<a.cols;x++) {float gx=dx.at<float>(y,x),gy=dy.at<float>(y,x);cov.at<Vec3f>(y,x)=Vec3f(gx*gx,gx*gy,gy*gy);}
 boxFilter(cov,cov,-1,Size(3,3),Point(-1,-1),false);
 mat("sobel_dx",dx);mat("sobel_dy",dy);mat("covariance",cov);
 cornerHarris(a,harris,3,3,.04);cornerMinEigenVal(a,eigen,3,3);mat("harris",harris);mat("min_eigen",eigen);
 std::vector<KeyPoint> k,ke;
 GFTTDetector::create(200,.001,20,3,true)->detect(a,k);
 GFTTDetector::create(200,.001,20,3,false)->detect(a,ke);
 static_assert(sizeof(KeyPoint)==28,"KeyPoint ABI");vec("gftt_harris",k);vec("gftt_eigen",ke);
 std::sort(k.begin(),k.end(),[](const KeyPoint &x,const KeyPoint &y){return x.response>y.response;});vec("rdvio_sorted",k);
 std::vector<Point2f> pts,flow;
 for(auto &p:k)pts.push_back(p.pt);
 int w=a.cols,h=a.rows;
 for(Point2f p: {Point2f(-40,-40),Point2f(-11,-11),Point2f(0,0),Point2f(w-1.f,h-1.f),Point2f(w+50.f,h/2.f),Point2f(w/2.f,h/2.f),Point2f(20.25f,20.75f),Point2f(w-20.5f,h-20.25f)})pts.push_back(p);
 flow=pts;
 if(variant%2)for(size_t i=0;i<flow.size();i++) { flow[i].x+=(float)(int(i%5)-2)*.37f;flow[i].y+=(float)(int(i%7)-3)*.29f; }
 if(variant%3==2)for(size_t i=0;i<flow.size();i+=7) { flow[i].x+=w*.8f; flow[i].y-=h*.7f; }
 vec("points",pts);vec("initial_flow",flow);
 std::vector<uchar> status;std::vector<float> err;sentinel(err,pts.size());
 calcOpticalFlowPyrLK(pa,pb,pts,flow,status,err,Size(21,21),3,TermCriteria(TermCriteria::COUNT+TermCriteria::EPS,30,.01),OPTFLOW_USE_INITIAL_FLOW);
 vec("forward_points",flow);vec("forward_status",status);vec("forward_err",err);
 std::vector<Point2f> reverse=pts;std::vector<uchar> rs;std::vector<float> re;sentinel(re,pts.size());
 calcOpticalFlowPyrLK(pb,pa,flow,reverse,rs,re,Size(21,21),3,TermCriteria(TermCriteria::COUNT+TermCriteria::EPS,30,.01),OPTFLOW_USE_INITIAL_FLOW);
 vec("reverse_points",reverse);vec("reverse_status",rs);vec("reverse_err",re);
 std::vector<double> norms;
 for(size_t i=0;i<pts.size();i++) {
   Point2f d=flow[i]-pts[i]; double disp=sqrt((double)d.x*d.x+(double)d.y*d.y);
   double n=norm(pts[i]-reverse[i]);norms.push_back(n);
   if(flow[i].x<20||flow[i].y<20||flow[i].x>=w-20||flow[i].y>=h-20)status[i]=0;
   if(status[i]&&disp>h/4)status[i]=0;
   if(status[i]&&(!rs[i]||n>.5))status[i]=0;
 }
 vec("norm",norms);vec("rdvio_status",status);
 if(fclose(out))throw std::runtime_error("close");
 printf("%s points=%zu\n",path.c_str(),pts.size());
}
static Mat gray(FILE *f,int w,int h,int index) {
 uint64_t off=20+(uint64_t)index*(8+(uint64_t)w*h)+8;
 if(fseek(f,(long)off,SEEK_SET))throw std::runtime_error("seek");
 Mat m(h,w,CV_8UC1);if(fread(m.data,1,(size_t)w*h,f)!=(size_t)w*h)throw std::runtime_error("gray read");return m;
}
int main(int argc,char **argv) {
 try {
 if(argc<3) {fprintf(stderr,"dump_main gray_file output_dir [quick]\n");return 2;}
 setNumThreads(1);printf("FEATURES %s\n",getCPUFeaturesLine().c_str());
 bool quick=argc>3;std::string dir=argv[2];
 uint32_t rng=0x12345678;int variant=0;
 std::vector<Size> sizes=quick?std::vector<Size>{Size(753,479)}:std::vector<Size>{Size(752,480),Size(640,480),Size(753,479),Size(751,480),Size(640,481),Size(31,27),Size(1,1),Size(43,43)};
 for(auto size:sizes)for(int kind=0;kind<5;kind++) {
   if(quick&&kind!=4)continue;
   Mat m(size,CV_8UC1),b(size,CV_8UC1);
   for(int y=0;y<m.rows;y++)for(int x=0;x<m.cols;x++) {
     rng^=rng<<13;rng^=rng>>17;rng^=rng<<5;
     m.at<uchar>(y,x)=kind==0?0:kind==1?255:kind==2?(x*3+y*7)%256:kind==3?((x/8+y/8)%2)*255:rng>>24;
   }
   for(int y=0;y<m.rows;y++)for(int x=0;x<m.cols;x++)b.at<uchar>(y,x)=m.at<uchar>(borderInterpolate(y-9,m.rows,BORDER_REFLECT_101),borderInterpolate(x-37,m.cols,BORDER_REFLECT_101));
   case_dump(dir+"/synthetic_"+std::to_string(size.width)+"x"+std::to_string(size.height)+"_"+std::to_string(kind)+".bin",m,b,variant++);
 }
 FILE *f=fopen(argv[1],"rb");if(!f)throw std::runtime_error("gray open");
 char magic[8];uint32_t hdr[3];if(fread(magic,1,8,f)!=8||memcmp(magic,"OKGRAY1\0",8)||fread(hdr,4,3,f)!=3)throw std::runtime_error("gray header");
 Mat K=(Mat_<double>(3,3)<<458.654,0,367.215,0,457.296,248.375,0,0,1);
 Mat D=(Mat_<double>(1,4)<<-.28340811,.07395907,.00019359,1.76187114e-5),m1,m2;
 initUndistortRectifyMap(K,D,Mat(),K,Size(hdr[0],hdr[1]),CV_32FC1,m1,m2);
 for(int base: {0,100,700,1800,3000}) {
   if(quick&&base!=100)continue;
   for(int gap=1;gap<=5;gap++) {
     if(quick&&gap!=1)continue;
     if((uint32_t)(base+gap)>=hdr[2])throw std::runtime_error("missing frames");
     Mat a=gray(f,hdr[0],hdr[1],base),b=gray(f,hdr[0],hdr[1],base+gap),ua,ub;
     remap(a,ua,m1,m2,INTER_LINEAR);remap(b,ub,m1,m2,INTER_LINEAR);
     case_dump(dir+"/undistorted_"+std::to_string(base)+"_"+std::to_string(gap)+".bin",ua,ub,variant++);
     if(base==0&&!quick)case_dump(dir+"/raw_0_"+std::to_string(gap)+".bin",a,b,variant++);
   }
 }
 fclose(f);return 0;
 } catch(const std::exception &e) {fprintf(stderr,"%s\n",e.what());return 1;}
}
