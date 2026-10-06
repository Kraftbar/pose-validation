/* SPDX-License-Identifier: MIT */
#include "rd_cv_undist.h"
#include "../reference_cv/m7b_io.h"
static int pack(const char *raw,const char *expected) {
 FILE *f=fopen(raw,"rb"),*g=fopen(expected,"rb");uint8_t head[20],ref[20];
 if(!f||!g||fread(head,1,20,f)!=20||fread(ref,1,20,g)!=20||memcmp(head,ref,20)||memcmp(head,"OKGRAY1\0",8))return 2;
 uint32_t w,h,n;memcpy(&w,head+8,4);memcpy(&h,head+12,4);memcpy(&n,head+16,4);
 if(!w||!h||w>=32767||h>=32767)return 2;
 size_t z=(size_t)w*h;
 uint8_t *s=malloc(z),*d=malloc(z),*e=malloc(z);float *m1=malloc(z*4),*m2=malloc(z*4);
 if(!s||!d||!e||!m1||!m2)return 2;
 double K[4]={458.654,457.296,367.215,248.375},D[4]={-.28340811,.07395907,.00019359,1.76187114e-5};
 if(!rd_cv_undistort_maps(K,D,0,w,h,m1,m2))return 2;
 compared=20;
 for(uint32_t i=0;i<n;i++) {
   uint64_t t,u;if(fread(&t,8,1,f)!=1||fread(&u,8,1,g)!=1||fread(s,1,z,f)!=z||fread(e,1,z,g)!=z){invalid=1;break;}
   rd_cv_remap_linear(s,w,h,m1,m2,d);cases++;
   for(size_t j=0;j<z;j++)mismatches+=d[j]!=e[j];
   for(int j=0;j<8;j++)mismatches+=((uint8_t*)&t)[j]!=((uint8_t*)&u)[j];
   compared+=8+z;
 }
 if(fgetc(f)!=EOF||fgetc(g)!=EOF)invalid=1;
 free(s);free(d);free(e);free(m1);free(m2);fclose(f);fclose(g);return finish("m7b_pack");
}
int main(int argc,char **argv) {
 if(argc==4&&!strcmp(argv[1],"--pack"))return pack(argv[2],argv[3]);
 if(argc!=2)return 2;
 input=fopen(argv[1],"rb");char magic[8];
 if(!input||fread(magic,1,8,input)!=8||memcmp(magic,"RDUND01\0",8))return 2;
 for(;;) {
  uint32_t hdr[4];size_t got=fread(hdr,1,16,input);if(!got&&feof(input))break;
  if(got!=16||!hdr[0]||!hdr[1]||hdr[0]>=32767||hdr[1]>=32767){invalid=1;break;}
  int w=hdr[0],h=hdr[1];size_t n=(size_t)w*h,z;cases++;
  double *K=record("K",&z);if(!K||z!=32){free(K);invalid=1;break;}
  double *D=record("D",&z);if(!D||z!=32){free(K);free(D);invalid=1;break;}
  uint8_t *s=record("input",&z),*d=malloc(n);float *a=NULL,*b=NULL;
  if(!s||z!=n||!d){invalid=1;goto done;}
  if(hdr[3]) {a=record("map_input1",&z);if(!a||z!=n*4){invalid=1;goto done;}b=record("map_input2",&z);if(!b||z!=n*4){invalid=1;goto done;}}
  else {a=malloc(n*4);b=malloc(n*4);if(!a||!b||!rd_cv_undistort_maps(K,D,hdr[2],w,h,a,b)){invalid=1;goto done;}compare(NULL,"map1",a,n*4);compare(NULL,"map2",b,n*4);}
  if(!rd_cv_remap_linear(s,w,h,a,b,d)){invalid=1;goto done;}compare(NULL,"remap",d,n);
 done:free(K);free(D);free(s);free(d);free(a);free(b);if(invalid)break;
 }
 fclose(input);return finish("m7b_undist");
}
