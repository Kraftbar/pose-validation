/* SV_PORT_SOURCES: check_sv_landmark_descriptor.c sv_landmark_descriptor.c */
/* SPDX-License-Identifier: MIT */
#include "sv_landmark_descriptor.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

static int read_hex(FILE *in,uint8_t *desc)
{
    char s[65];if(fscanf(in,"%64s",s)!=1 || strlen(s)!=64)return -1;
    for(unsigned i=0;i<32;++i){
        char pair[3]={s[2*i],s[2*i+1],0},*end;
        unsigned long x=strtoul(pair,&end,16);if(*end)return -1;desc[i]=(uint8_t)x;
    }
    return 0;
}
static int run(const char *label,const char *commands,const char *expected)
{
    FILE *in=fopen(commands,"r"),*ref=fopen(expected,"r");
    if(!in || !ref){if(in)fclose(in);if(ref)fclose(ref);return 2;}
    char tag;size_t total=0,bad=0,cases=0;int error=0;
    while(fscanf(in," %c",&tag)==1){
        unsigned qid,n;
        if(tag!='Q' || fscanf(in,"%u%u",&qid,&n)!=2 || !n || n>100000){error=1;break;}
        sv_descriptor_observation *obs=calloc(n,sizeof(*obs));
        uint8_t *descs=calloc(n,32);uint16_t *medians=calloc(n,sizeof(*medians));
        if(!obs || !descs || !medians){free(obs);free(descs);free(medians);error=1;break;}
        for(unsigned i=0;i<n;++i){
            obs[i].descriptor=descs+32*i;
            if(fscanf(in,"%" SCNu32 " %d",&obs[i].keyframe_id,&obs[i].erased)!=2
                || read_hex(in,descs+32*i)){error=1;break;}
        }
        sv_landmark_descriptor_result result={0};
        if(!error && sv_landmark_select_descriptor(obs,n,&result,medians))error=1;
        unsigned rid,index,id,median,rn;uint8_t rd[32];
        if(!error && (fscanf(ref," %c%u%u%u%u",&tag,&rid,&index,&id,&median)!=5 || tag!='Q'
            || rid!=qid || read_hex(ref,rd) || fscanf(ref,"%u",&rn)!=1 || rn!=n))error=1;
        if(!error){
            size_t before=bad;
            bad+=result.observation_index!=index;bad+=result.keyframe_id!=id;bad+=result.median_distance!=median;total+=3;
            for(unsigned i=0;i<32;++i){bad+=result.descriptor[i]!=rd[i];++total;}
            for(unsigned i=0;i<n;++i){
                unsigned m;if(fscanf(ref,"%u",&m)!=1){error=1;break;}
                bad+=medians[i]!=m;++total;
            }
            if(bad!=before && before<5)fprintf(stderr,"case %u: %zu mismatches\n",qid,bad-before);
        }
        free(obs);free(descs);free(medians);
        if(error)break;
        ++cases;
    }
    if(!cases || ferror(in) || ferror(ref) || fscanf(ref," %c",&tag)==1)error=1;
    fclose(in);fclose(ref);printf("%s: %zu/%zu\n",label,bad,total);
    return error?2:bad?1:0;
}
static int selftest(void)
{
    unsigned total=0,bad=0;
    #define CHECK(x) do{++total;if(!(x)){++bad;fprintf(stderr,"API line %d\n",__LINE__);}}while(0)
    uint8_t a[32]={0},b[32];memset(b,255,32);
    sv_descriptor_observation obs[]={{UINT32_MAX,a,0},{0,b,0}};
    sv_landmark_descriptor_result out;uint16_t medians[2]={99,99};
    CHECK(!sv_landmark_select_descriptor(obs,2,&out,medians) && out.observation_index==1 && out.keyframe_id==0);
    CHECK(out.median_distance==0 && medians[0]==0 && medians[1]==0);
    CHECK(!memcmp(out.descriptor,b,32));
    b[0]=1;CHECK(out.descriptor[0]==255);b[0]=255;
    obs[1].erased=1;obs[1].descriptor=NULL;
    CHECK(!sv_landmark_select_descriptor(obs,2,&out,medians) && out.observation_index==0 && medians[1]==UINT16_MAX);
    CHECK(!sv_landmark_select_descriptor(obs,2,&out,NULL));
    memset(&out,0x72,sizeof(out));sv_landmark_descriptor_result saved;memcpy(&saved,&out,sizeof(out));
    medians[0]=99;medians[1]=99;
    CHECK(sv_landmark_select_descriptor(NULL,2,&out,medians)==-1);
    CHECK(sv_landmark_select_descriptor(obs,0,&out,medians)==-1);
    CHECK(sv_landmark_select_descriptor(obs,2,NULL,medians)==-1);
    CHECK(sv_landmark_select_descriptor(obs,SIZE_MAX,&out,medians)==-1);
    obs[1].keyframe_id=UINT32_MAX;
    CHECK(sv_landmark_select_descriptor(obs,2,&out,medians)==-1);obs[1].keyframe_id=0;
    obs[0].erased=1;CHECK(sv_landmark_select_descriptor(obs,2,&out,medians)==-1);obs[0].erased=0;
    obs[0].descriptor=NULL;CHECK(sv_landmark_select_descriptor(obs,2,&out,medians)==-1);obs[0].descriptor=a;
    CHECK(!memcmp(&out,&saved,sizeof(out)) && medians[0]==99 && medians[1]==99);
    #undef CHECK
    printf("api: %u/%u\n",bad,total);return bad?1:0;
}
int main(int argc,char **argv)
{
    if(argc==2 && !strcmp(argv[1],"--selftest"))return selftest();
    if(argc==5 && !strcmp(argv[1],"--case"))return run(argv[2],argv[3],argv[4]);
    if(argc==4 || argc==5){
        char in[4096],ref[4096];
        int a=snprintf(in,sizeof(in),"%s/../../landmark_descriptor/fixtures/%s/commands.txt",argv[3],argv[1]);
        int b=snprintf(ref,sizeof(ref),"%s/../../landmark_descriptor/fixtures/%s/expected.tsv",argv[3],argv[1]);
        if(a<0 || b<0 || a>=(int)sizeof(in) || b>=(int)sizeof(ref))return 2;
        return run(argv[1],in,ref);
    }
    fprintf(stderr,"usage: --selftest | --case label input expected | seq fixtures dumps [max_frames]\n");return 2;
}
