/* SV_PORT_SOURCES: check_sv_match_bow.c sv_match_bow.c */
/* SPDX-License-Identifier: MIT */
/* Fixture reader and exact complete-output comparator. */
#include "sv_match_bow.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <math.h>

typedef struct {
    sv_match_bow_view view;
    sv_keypoint *points;
    uint8_t *desc, *erased;
    uint64_t *tokens;
    sv_bow_feat_vector features;
    int defined;
} frame;

static void release(frame *f)
{
    free(f->points); free(f->desc); free(f->erased); free(f->tokens);
    for (uint32_t i=0;i<f->features.count;++i) free(f->features.nodes[i].kp_indices);
    free(f->features.nodes);
}
static void *array(size_t n, size_t size)
{
    void *p=calloc(n?n:1,size);
    if (!p) { fprintf(stderr,"out of memory\n"); exit(2); }
    return p;
}

static int run(const char *label,const char *commands,const char *expected)
{
    FILE *in=fopen(commands,"r"), *ref=fopen(expected,"r");
    if (!in || !ref) { if(in)fclose(in); if(ref)fclose(ref); return 2; }
    frame *frames=NULL; size_t capacity=0, queries=0, total=0, bad=0;
    char op; int error=0;
    while (fscanf(in," %c",&op)==1) {
        if (op=='F') {
            unsigned id,n;
            if (fscanf(in,"%u%u",&id,&n)!=2 || id>100000 || n>100000) {error=1;break;}
            if (id>=capacity) {
                size_t next=(size_t)id+32;
                frame *p=realloc(frames,next*sizeof(*p));
                if(!p){error=1;break;}
                memset(p+capacity,0,(next-capacity)*sizeof(*p)); frames=p;capacity=next;
            }
            frame *f=&frames[id];
            if(f->defined){error=1;break;}
            f->defined=1; f->view.count=n;
            f->points=array(n,sizeof(*f->points));f->desc=array(n,32);
            f->erased=array(n,1);f->tokens=array(n,sizeof(*f->tokens));
            for(unsigned i=0;i<n;++i){
                char hex[65];unsigned erased;
                if(fscanf(in,"%f %64s %" SCNu64 " %u",&f->points[i].angle,hex,&f->tokens[i],&erased)!=4
                   || strlen(hex)!=64 || erased>1){error=1;break;}
                f->erased[i]=(uint8_t)erased;
                for(unsigned j=0;j<32;++j){
                    char pair[3]={hex[2*j],hex[2*j+1],0},*end;
                    unsigned long v=strtoul(pair,&end,16);
                    if(*end){error=1;break;}f->desc[32*i+j]=(uint8_t)v;
                }
            }
            if(error)break;
            if(fscanf(in,"%u",&f->features.count)!=1 || f->features.count>100000){error=1;f->features.count=0;break;}
            f->features.nodes=array(f->features.count,sizeof(*f->features.nodes));
            for(unsigned i=0;i<f->features.count;++i){
                sv_bow_feat_node *node=&f->features.nodes[i];
                if(fscanf(in,"%u%u",&node->node_id,&node->count)!=2 || node->count>n){error=1;break;}
                node->kp_indices=array(node->count,sizeof(*node->kp_indices));
                for(unsigned j=0;j<node->count;++j)
                    if(fscanf(in,"%u",&node->kp_indices[j])!=1){error=1;break;}
            }
            if(error)break;
        }
        else if(op=='Q'){
            unsigned id,a,b,mode,orient;float ratio;
            if(fscanf(in,"%u%u%u%u%f%u",&id,&mode,&a,&b,&ratio,&orient)!=6 || mode>1
                || a>=capacity || b>=capacity || !frames[a].defined || !frames[b].defined){error=1;break;}
            /* realloc moves frame structs; refresh borrowed feature pointers. */
            for(unsigned side=0;side<2;++side){
                frame *f=&frames[side?b:a];
                f->view.keypoints=f->points;f->view.descriptors=f->desc;
                f->view.features=&f->features;f->view.landmarks=f->tokens;f->view.erased=f->erased;
            }
            unsigned n=frames[mode?a:b].view.count, count=UINT32_MAX;
            uint64_t *matched=array(n,sizeof(*matched));
            for(unsigned i=0;i<n;++i)matched[i]=UINT64_MAX;
            int rc=mode ? sv_match_bow_keyframes(&frames[a].view,&frames[b].view,ratio,orient,matched,&count)
                        : sv_match_bow_frame(&frames[a].view,&frames[b].view,ratio,orient,matched,&count);
            char tag;unsigned rid,rn,r_count;
            if(rc || fscanf(ref," %c%u%u%u",&tag,&rid,&r_count,&rn)!=4 || tag!='Q' || rid!=id || rn!=n){
                fprintf(stderr,"invalid/reference query %u\n",id);free(matched);error=1;break;
            }
            total++;bad+=count!=r_count;
            for(unsigned i=0;i<n;++i){
                uint64_t token;
                if(fscanf(ref,"%" SCNu64,&token)!=1){error=1;break;}
                ++total;
                if(token!=matched[i]){
                    if(bad<8)fprintf(stderr,"query %u slot %u: got %" PRIu64 " expected %" PRIu64 "\n",id,i,matched[i],token);
                    ++bad;
                }
            }
            free(matched);if(error)break;++queries;
        }
        else {error=1;break;}
    }
    if(ferror(in) || ferror(ref) || !queries || fscanf(ref," %c",&op)==1)error=1;
    fclose(in);fclose(ref);
    for(size_t i=0;i<capacity;++i)release(&frames[i]);
    free(frames);
    printf("%s: %zu/%zu\n",label,bad,total);
    return error?2:bad?1:0;
}

static int selftest(void)
{
    unsigned bad=0,total=0,count=99;
    uint64_t result=77,token=4;
    uint8_t desc[32]={0},erased=0;
    sv_keypoint kp={0};uint32_t idx=0;
    sv_bow_feat_node node={5,&idx,1};sv_bow_feat_vector feat={&node,1};
    sv_match_bow_view v={1,&kp,desc,&feat,&token,&erased};
    #define CHECK(x) do{++total;if(!(x)){++bad;fprintf(stderr,"API line %d\n",__LINE__);}}while(0)
    CHECK(!sv_match_bow_frame(&v,&v,.6f,1,&result,&count) && result==token && count==1);
    CHECK(!sv_match_bow_keyframes(&v,&v,.6f,1,&result,&count) && result==token && count==1);
    erased=1;
    CHECK(!sv_match_bow_frame(&v,&v,.6f,1,&result,&count) && !result && !count);erased=0;
    token=0;
    CHECK(!sv_match_bow_keyframes(&v,&v,.6f,1,&result,&count) && !result && !count);token=4;
    result=77;count=99;
    CHECK(sv_match_bow_frame(NULL,&v,.6f,1,&result,&count)==-1);
    CHECK(sv_match_bow_frame(&v,NULL,.6f,1,&result,&count)==-1);
    CHECK(sv_match_bow_frame(&v,&v,NAN,1,&result,&count)==-1);
    CHECK(sv_match_bow_frame(&v,&v,INFINITY,1,&result,&count)==-1);
    CHECK(sv_match_bow_frame(&v,&v,-1,1,&result,&count)==-1);
    CHECK(sv_match_bow_frame(&v,&v,.6f,1,NULL,&count)==-1);
    CHECK(sv_match_bow_frame(&v,&v,.6f,1,&result,NULL)==-1);
    idx=1;CHECK(sv_match_bow_frame(&v,&v,.6f,1,&result,&count)==-1);idx=0;
    kp.angle=NAN;CHECK(sv_match_bow_frame(&v,&v,.6f,1,&result,&count)==-1);kp.angle=0;
    uint32_t twice[]={0,0};node.count=2;node.kp_indices=twice;
    CHECK(sv_match_bow_frame(&v,&v,.6f,1,&result,&count)==-1);
    node.count=1;node.kp_indices=&idx;
    sv_bow_feat_node unsorted[]={{5,&idx,1},{4,NULL,0}};feat.nodes=unsorted;feat.count=2;
    CHECK(sv_match_bow_frame(&v,&v,.6f,1,&result,&count)==-1);
    feat.nodes=&node;feat.count=1;
    CHECK(result==77 && count==99);
    sv_bow_feat_vector empty={0};sv_match_bow_view e={0};e.features=&empty;
    CHECK(!sv_match_bow_frame(&e,&e,.6f,1,NULL,&count) && !count);
    CHECK(!sv_match_bow_keyframes(&e,&e,.6f,1,NULL,&count) && !count);
    #undef CHECK
    printf("api: %u/%u\n",bad,total);return bad?1:0;
}

int main(int argc,char **argv)
{
    if(argc==2 && !strcmp(argv[1],"--selftest"))return selftest();
    if(argc==5 && !strcmp(argv[1],"--case"))return run(argv[2],argv[3],argv[4]);
    if(argc==4 || argc==5){
        char commands[4096],expected[4096];
        /* dump_dir = .../reference_dumps/<seq>; lifecycle tests always full. */
        int a=snprintf(commands,sizeof(commands),"%s/../../match_bow/fixtures/%s/commands.txt",argv[3],argv[1]);
        int b=snprintf(expected,sizeof(expected),"%s/../../match_bow/fixtures/%s/expected.tsv",argv[3],argv[1]);
        if(a<0 || b<0 || a>=(int)sizeof(commands) || b>=(int)sizeof(expected))return 2;
        return run(argv[1],commands,expected);
    }
    fprintf(stderr,"usage: --selftest | --case label commands expected | seq fixtures dumps [max_frames]\n");return 2;
}
