/* OK_PORT_SOURCES: check_ok_brisk_api.c ok_brisk_detector.c ok_brisk_descriptor.c ok_brisk_camera.c
 * SPDX-License-Identifier: MIT
 * Boundary checks for the C API's documented domain and ownership contract.
 */
#include "ok_brisk.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#define REQUIRE(x) do { checks++; if(!(x)){fprintf(stderr,"API check failed line %d: %s\n",__LINE__,#x);return 1;} } while(0)
int main(void){
 unsigned checks=0;
 uint8_t image[32*32]={0},*desc=NULL;
 float rays[32*32*3]={0},jacs[32*32*6]={0},dir[3]={0,1,0};
 ok_brisk_keypoint *kp=NULL;
 size_t n=0;
 REQUIRE(ok_brisk_detect(NULL,32,32,38,150,700,&kp,&n,NULL)==-1);
 REQUIRE(ok_brisk_detect(image,19,32,38,150,700,&kp,&n,NULL)==-1);
 REQUIRE(ok_brisk_detect(image,32,32,NAN,150,700,&kp,&n,NULL)==-1);
 REQUIRE(ok_brisk_detect(image,32,32,.5,150,700,&kp,&n,NULL)==-1);
 REQUIRE(ok_brisk_detect(image,32,32,38,0,700,&kp,&n,NULL)==-1);
 REQUIRE(ok_brisk_detect(image,32,32,38,150,0,&kp,&n,NULL)==-1);
 REQUIRE(ok_brisk_detect(image,32,32,38,150,700,NULL,&n,NULL)==-1);
 REQUIRE(ok_brisk_detect(image,32,32,38,150,700,&kp,NULL,NULL)==-1);
 REQUIRE(ok_brisk_detect(image,65535,65535,38,150,700,&kp,&n,NULL)==-1);
 REQUIRE(ok_brisk_detect(image,32,32,38,150,700,&kp,&n,NULL)==0 && n==0 && kp==NULL);
 ok_brisk_context*ctx=ok_brisk_create(NULL);
 REQUIRE(ctx!=NULL);
 REQUIRE(ok_brisk_describe(ctx,image,32,32,NULL,NULL,0,NULL,kp,&n,&desc,NULL)==0 && n==0 && desc!=NULL);
 free(desc);desc=NULL;
 REQUIRE(ok_brisk_describe(NULL,image,32,32,NULL,NULL,0,NULL,kp,&n,&desc,NULL)==-1);
 REQUIRE(ok_brisk_describe(ctx,image,32,32,rays,NULL,458,dir,kp,&n,&desc,NULL)==-1);
 REQUIRE(ok_brisk_describe(ctx,image,32,32,rays,jacs,0,dir,kp,&n,&desc,NULL)==-1);
 REQUIRE(ok_brisk_describe(ctx,image,32,32,rays,jacs,458,NULL,kp,&n,&desc,NULL)==-1);
 ok_brisk_keypoint border={0,0,12,-1,1000,0,-1};n=1;
 REQUIRE(ok_brisk_describe(ctx,image,32,32,NULL,NULL,0,NULL,&border,&n,&desc,NULL)==0 && n==0);
 free(desc);desc=NULL;
 border.x=NAN;n=1;
 REQUIRE(ok_brisk_describe(ctx,image,32,32,NULL,NULL,0,NULL,&border,&n,&desc,NULL)==-1);
 REQUIRE(ok_brisk_describe(ctx,image,32,32,NULL,NULL,0,NULL,NULL,&n,&desc,NULL)==-1);
 REQUIRE(ok_brisk_describe(ctx,image,32,32,NULL,NULL,0,NULL,&border,NULL,&desc,NULL)==-1);
 REQUIRE(ok_brisk_describe(ctx,image,32,32,NULL,NULL,0,NULL,&border,&n,NULL,NULL)==-1);
 ok_brisk_destroy(ctx);ok_brisk_destroy(NULL);
 printf("PASS %u API checks\n",checks);
 return 0;
}
