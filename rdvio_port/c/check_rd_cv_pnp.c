/* SPDX-License-Identifier: MIT */
#include "rd_cv_pnp.h"
#include "../reference_cv/m7b_io.h"
int main(int argc,char **argv) {
 if(argc!=2)return 2;
 input=fopen(argv[1],"rb");char magic[8];
 if(!input||fread(magic,1,8,input)!=8)return 2;
 int real=!memcmp(magic,"RDPNPR1\0",8);
 if(!real&&memcmp(magic,"RDPNP01\0",8))return 2;
 for(;;) {
  uint32_t hdr[2]={0};size_t header=real?4:8;size_t got=fread(hdr,1,header,input);if(!got&&feof(input))break;
  if(got!=header||(hdr[0]!=4&&hdr[0]!=6)){invalid=1;break;}cases++;size_t z;int n=hdr[0];
  if(real) {
   double X[18],x[12],T[16],ref[16];
   if(fread(X,24,n,input)!=(size_t)n||fread(x,16,n,input)!=(size_t)n||fread(ref,8,16,input)!=16){invalid=1;break;}
   if(n==6)rd_cv_pnp6(NULL,(const double (*)[3])X,(const double (*)[2])x,T);
   else rd_cv_pnp4(NULL,(const double (*)[3])X,(const double (*)[2])x,T);
   for(size_t j=0;j<sizeof(T);j++)if(((uint8_t*)T)[j]!=((uint8_t*)ref)[j]) {
    if(mismatches<10)fprintf(stderr,"real case=%zu byte=%zu\n",cases,j);
    mismatches++;
   }
   compared+=sizeof(T);continue;
  }
  double *X=record("X",&z);if(!X||z!=(size_t)n*24){invalid=1;free(X);break;}
  double *x=record("x",&z);if(!x||z!=(size_t)n*16){invalid=1;free(X);free(x);break;}
  rd_cv_pnp_trace trace={compare,NULL};double T[16],r[3],t[3];
  rd_cv_pnp(n,X,x,T,r,t,&trace);compare(NULL,"rvec",r,24);compare(NULL,"tvec",t,24);compare(NULL,"T",T,128);
  free(X);free(x);if(invalid)break;
 }
 fclose(input);return finish("m7b_pnp");
}
