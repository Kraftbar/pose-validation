/* SV_PORT_SOURCES: check_sv_pnp.c sv_pnp.c sv_rng.c sv_eigen_svd.c sv_eigen_qr.c sv_linalg.c ../reference_reloc/c/sv_eigen_pnp.c */
/* SPDX-License-Identifier: MIT */
#define SV_RELOC_EMBEDDED
#include "../reference_reloc/check_pnp.c"
#include "../reference_reloc/suite.h"
int main(int argc,char**argv){
 char folder[4096],listpath[4352];size_t bad=0,total=0;unsigned cases=0;int code=0;
 if(suite_folder(argc,argv,folder))return 2;
 snprintf(listpath,sizeof(listpath),"%s/pnp/cases.txt",folder);
 FILE*list=fopen(listpath,"r");if(!list)return 2;
 char input[256],trace[256];int recompute;
 while(fscanf(list,"%255s %255s %d",input,trace,&recompute)==3){
  char a[4608],b[4608];snprintf(a,sizeof(a),"%s/pnp/%s",folder,input);snprintf(b,sizeof(b),"%s/pnp/%s",folder,trace);
  int rc=check_pnp_case(a,b,recompute,&bad,&total);if(rc>code)code=rc;cases++;if(rc==2)break;
 }
 if(ferror(list)||!feof(list)||!cases||!total)code=2;
 fclose(list);printf("%s: %zu/%zu\n",argv[1],bad,total);return code;
}
