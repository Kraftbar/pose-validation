#include "../c/sv_pnp.h"
#include "../c/sv_track.h"
#include "trace_compare.h"
static int check_pnp_case(const char*input,const char*trace,int recompute,size_t*bad,size_t*total){
 FILE*f=fopen(input,"rb");if(!f)return 2;
 unsigned n,levels;trace_read(f,&n,4);trace_read(f,&levels,4);
 if(n>100000||!levels||levels>16)return 2;
 float scales[16];trace_read(f,scales,4*levels);
 double*b=calloc(n?n:1,24),*p=calloc(n?n:1,24),*bits=calloc(n?n:1,8);
 int*oct=calloc(n?n:1,sizeof(int));unsigned char*mask=calloc(n?n:1,1);
 if(!b||!p||!bits||!oct||!mask)return 2;
 for(unsigned i=0;i<n;i++){trace_read(f,b+3*i,24);trace_read(f,p+3*i,24);trace_read(f,oct+i,4);}
 if(fgetc(f)!=EOF||ferror(f))return 2;fclose(f);
 trace_comparison c={0};c.case_name=input;c.f=fopen(trace,"rb");if(!c.f)return 2;
 sv_pnp_result result;
 if(sv_pnp_ransac(b,p,oct,n,scales,levels,10,30,10,recompute,NULL,&result,mask,compare_trace,&c))return 2;
 compare_scalar(&c,"final_valid",result.valid);
 for(unsigned i=0;i<n;i++)bits[i]=mask[i];
 compare_trace(&c,"final_mask",bits,n<10?0:n);
 if(result.valid){compare_trace(&c,"final_rotation",result.rotation,9);compare_trace(&c,"final_translation",result.translation,3);}
 if(fgetc(c.f)!=EOF||ferror(c.f))return 2;fclose(c.f);
 free(b);free(p);free(bits);free(oct);free(mask);*bad+=c.bad;*total+=c.total;
 return c.bad?1:0;
}
#ifndef SV_RELOC_EMBEDDED
int main(int argc,char**argv){size_t bad=0,total=0;if(argc!=4)return 2;int rc=check_pnp_case(argv[1],argv[2],atoi(argv[3]),&bad,&total);printf("pnp: %zu/%zu\n",bad,total);return rc;}
#endif
