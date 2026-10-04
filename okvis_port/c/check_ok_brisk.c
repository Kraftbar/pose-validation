/* OK_PORT_SOURCES: check_ok_brisk.c ok_brisk_detector.c ok_brisk_descriptor.c ok_brisk_camera.c
 * SPDX-License-Identifier: MIT
 * Native BRISK trace comparator. No tolerances or reference feedback.
 */
#include "ok_brisk.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
typedef struct {FILE*f;size_t bytes,bad,records;const char*file;} compare;
static void read_exact(FILE*f,void*p,size_t n){if(n&&fread(p,1,n,f)!=n){fprintf(stderr,"truncated fixture\n");exit(2);}}
static void check(void*user,const char*tag,const void*p,size_t n){
 compare*c=user;char name[33]={0};uint32_t size;read_exact(c->f,name,32);read_exact(c->f,&size,4);
 if(strcmp(name,tag)||size!=n){fprintf(stderr,"%s record %zu: expected %s/%u got %s/%zu\n",c->file,c->records,name,size,tag,n);exit(3);}
 uint8_t*ref=malloc(n?n:1);if(!ref)exit(2);read_exact(c->f,ref,n);size_t bad=0,first=0;for(size_t i=0;i<n;i++)if(ref[i]!=((const uint8_t*)p)[i]){if(!bad)first=i;bad++;}
 if(bad&&c->bad<40){size_t i=first/4;uint32_t a=0,b=0;float af=0,bf=0;if((i+1)*4<=n){memcpy(&a,ref+4*i,4);memcpy(&b,(const uint8_t*)p+4*i,4);memcpy(&af,&a,4);memcpy(&bf,&b,4);}fprintf(stderr,"%s record %zu %s: %zu bytes differ first word[%zu] %08"PRIx32"/%08"PRIx32" float %.9g/%.9g\n",c->file,c->records,tag,bad,i,a,b,af,bf);}
 c->bad+=bad;c->bytes+=n;c->records++;free(ref);
}
int main(int argc,char**argv){
 if(argc<3){fprintf(stderr,"usage: check_ok_brisk <reference-dir> <case.bin...>\n");return 2;}
 char path[4096];snprintf(path,sizeof(path),"%s/pattern.bin",argv[1]);compare c={0};c.file="pattern";c.f=fopen(path,"rb");if(!c.f)return 2;ok_brisk_trace trace={check,&c};ok_brisk_context*ctx=ok_brisk_create(&trace);if(!ctx)return 2;if(fgetc(c.f)!=EOF)return 2;fclose(c.f);size_t total=c.bytes,bad=c.bad,records=c.records;
 for(int a=2;a<argc;a++){
  memset(&c,0,sizeof(c));c.file=argv[a];c.f=fopen(argv[a],"rb");if(!c.f)return 2;int hdr[4];float dir[3],focal;read_exact(c.f,hdr,16);read_exact(c.f,dir,12);read_exact(c.f,&focal,4);int w=hdr[0],h=hdr[1];if(w<20||h<20||w>4096||h>4096)return 2;size_t n=(size_t)w*h;uint8_t*im=malloc(n);float*rays=NULL,*jacs=NULL;read_exact(c.f,im,n);
  if(hdr[2]){snprintf(path,sizeof(path),"%s/cam%d.maps",argv[1],hdr[3]);FILE*f=fopen(path,"rb");if(!f)return 2;rays=malloc(n*12);jacs=malloc(n*24);if(!rays||!jacs)return 2;read_exact(f,rays,n*12);read_exact(f,jacs,n*24);fclose(f);}
  ok_brisk_keypoint*k=NULL;size_t nk=0;uint8_t*d=NULL;
  if(ok_brisk_detect(im,w,h,38,150,700,&k,&nk,&trace)||ok_brisk_describe(ctx,im,w,h,rays,jacs,focal,dir,k,&nk,&d,&trace))return 2;
  if(fgetc(c.f)!=EOF){fprintf(stderr,"trailing trace\n");return 2;}fclose(c.f);printf("%s %zu features: %zu/%zu bytes differ (%zu records)\n",argv[a],nk,c.bad,c.bytes,c.records);
  total+=c.bytes;bad+=c.bad;records+=c.records;free(im);free(rays);free(jacs);free(k);free(d);
 }
 ok_brisk_destroy(ctx);printf("%s %d cases: %zu/%zu bytes differ, %zu trace records\n",bad?"FAIL":"PASS",argc-2,bad,total,records);return bad?1:0;
}
