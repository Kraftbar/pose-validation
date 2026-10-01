#ifndef SV_RELOC_SUITE_H
#define SV_RELOC_SUITE_H
/* Shared runner ABI plus a direct <sequence> <leaf-fixtures> invocation. */
static int suite_folder(int argc,char**argv,char folder[4096]){
 int n;
 if(argc==3)n=snprintf(folder,4096,"%s",argv[2]);
 else if(argc==4||argc==5)n=snprintf(folder,4096,"%s/../../reference_reloc/fixtures/%s",argv[3],argv[1]);
 else return 2;
 return n<0||n>=4096?2:0;
}
#endif
