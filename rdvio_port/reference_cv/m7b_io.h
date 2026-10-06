/* SPDX-License-Identifier: MIT; harness-only fixture framing. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
static FILE *input;
static size_t compared,mismatches,cases;
static int invalid;
static void *record(const char *label,size_t *n) {
 char name[32];uint32_t z;
 if(invalid)return NULL;
 if(fread(name,1,32,input)!=32||!memchr(name,0,32)||strcmp(name,label)||fread(&z,4,1,input)!=1||z>100000000)goto error;
 void *p=malloc(z?z:1);if(!p)goto error;
 if(z&&fread(p,1,z,input)!=z){free(p);goto error;}*n=z;return p;
error:fprintf(stderr,"bad fixture at %s case %zu\n",label,cases);invalid=1;return NULL;
}
static void compare(void *u,const char *label,const void *p,size_t n) {
 (void)u;size_t z=0;uint8_t *v=record(label,&z);if(!v)return;
 if(z!=n) {fprintf(stderr,"size %s %zu != %zu\n",label,z,n);invalid=1;free(v);return;}
 size_t b=0;for(size_t i=0;i<n;i++)if(v[i]!=((const uint8_t*)p)[i]) {
 if(b==0&&mismatches<1000)fprintf(stderr,"case=%zu %s byte=%zu expected=%02x got=%02x\n",cases,label,i,v[i],((const uint8_t*)p)[i]);
 b++;
 }
 compared+=n;mismatches+=b;free(v);
}
static int finish(const char *label) {
 if(!cases)invalid=1;
 if(invalid){compared++;mismatches++;}
 printf("cases=%zu\n%s: %zu/%zu\n",cases,label,mismatches,compared);return mismatches?1:0;
}
