// SPDX-License-Identifier: MIT
#include <opencv2/opencv.hpp>
#include "m7b_io.hpp"
#include "epnp_trace.h"
#include <vector>
using namespace cv;
static uint32_t state=912345;
static double rnd() {state^=state<<13;state^=state>>17;state^=state<<5;return ((double)state/4294967296.0)*2-1;}
static void one(int n,int kind) {
 double X[18],x[12];
 double scale=kind==4?1e-6:kind==5?1e6:1;
 Mat rv0=(Mat_<double>(3,1)<<rnd()*2,rnd()*2,rnd()*2),rot;Rodrigues(rv0,rot);
 for(int i=0;i<n;i++) {
  double p[3]={rnd()*2,rnd()*2,rnd()*2};
  if(kind==1)p[2]=0;
  if(kind==2)p[2]*=1e-7;
  if(kind==6){p[1]=p[0]*2;p[2]=p[0]*3;}
  if(kind==7)p[0]=p[1]=p[2]=0;
  for(int j=0;j<3;j++)X[3*i+j]=p[j]*scale;
  double q[3]={.12,-.07,kind==3?-5.0:5.0};
  for(int j=0;j<3;j++)for(int k=0;k<3;k++)q[j]+=rot.at<double>(j,k)*p[k];
  x[2*i]=q[0]/q[2];x[2*i+1]=q[1]/q[2];
  if(kind==8){x[2*i]+=rnd()*.05;x[2*i+1]+=rnd()*.05;}
 }
 uint32_t hdr[2]={(uint32_t)n,(uint32_t)kind};wr(hdr,8);rec("X",X,n*3*8);rec("x",x,n*2*8);
 std::vector<Point3f> op;std::vector<Point2f> ip;
 for(int i=0;i<n;i++){op.emplace_back(X[3*i],X[3*i+1],X[3*i+2]);ip.emplace_back(x[2*i],x[2*i+1]);}
 Mat K=Mat::eye(3,3,CV_32F),rv,tv;
 solvePnP(op,ip,K,noArray(),rv,tv,false,SOLVEPNP_EPNP);
 Mat Kd;K.convertTo(Kd,CV_64F);Mat und;undistortPoints(ip,und,Kd,noArray());
 rd_epnp_trace model(Kd,Mat(op),und);Mat Rt,tt,rr;model.compute_pose(Rt,tt);Rodrigues(Rt,rr);
 if(memcmp(rr.data,rv.data,24)||memcmp(tt.data,tv.data,24))throw std::runtime_error("trace copy != installed solvePnP");
 mat("R",Rt);mat("rvec",rv);mat("tvec",tv);
 Mat rf,tf,R;rv.convertTo(rf,CV_32F);tv.convertTo(tf,CV_32F);Rodrigues(rf,R);
 double T[16]={0};T[15]=1;
 for(int i=0;i<3;i++){for(int j=0;j<3;j++)T[j*4+i]=R.at<float>(i,j);T[12+i]=tf.at<float>(i);}
 rec("T",T,sizeof(T));
}
int main(int argc,char **argv) {try {
 if(argc<2)return 2;setNumThreads(1);m7b_out=fopen(argv[1],"wb");if(!m7b_out)return 2;wr("RDPNP01\0",8);
 int count=argc>2?atoi(argv[2]):100;
 for(int n: {4,6})for(int kind=0;kind<9;kind++)for(int i=0;i<count;i++)one(n,kind);
 return fclose(m7b_out)?1:0;
 }catch(const std::exception &e){fprintf(stderr,"%s\n",e.what());return 1;}}
