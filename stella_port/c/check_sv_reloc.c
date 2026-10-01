/* SV_PORT_SOURCES: check_sv_reloc.c sv_pnp.c sv_rng.c sv_eigen_svd.c sv_eigen_qr.c sv_linalg.c sv_relocalizer.c sv_bow_db.c sv_track_frame.c sv_frame_tracker.c sv_local_map.c sv_frame.c sv_undistort.c sv_bow.c sv_match_bow.c sv_eigen_mat4.c sv_eigen_quaternion.c sv_g2o_se3.c sv_g2o_edge.c sv_g2o_pose_optimizer.c sv_eigen_llt.c sv_solve_essential_5pt.c sv_solve_essential_ransac.c sv_eigen_fullpivlu.c sv_eigen_eigensolver.c ../reference_reloc/c/sv_eigen_pnp.c */
/* SPDX-License-Identifier: MIT */
#define SV_RELOC_EMBEDDED
#include "../reference_reloc/check_reloc.c"
#include "../reference_reloc/suite.h"
int main(int argc,char**argv){
 char folder[4096],listpath[4352];size_t bad=0,total=0;unsigned cases=0;int code=0;
 if(suite_folder(argc,argv,folder))return 2;
 snprintf(listpath,sizeof(listpath),"%s/relocalization/cases.txt",folder);
 FILE*list=fopen(listpath,"r");if(!list)return 2;
 unsigned frame;int mode;char trace[256];
 while(fscanf(list,"%u %d %255s",&frame,&mode,trace)==3){
  char map[4352],vocab[4352],path[4608],m[32];
  snprintf(map,sizeof(map),"%s/%u.map",folder,frame);snprintf(vocab,sizeof(vocab),"%s/../../../../../external/candidates/orb_vocab.fbow",folder);
  snprintf(path,sizeof(path),"%s/relocalization/%s",folder,trace);snprintf(m,sizeof(m),"%d",mode);
  char*args[]={argv[0],map,vocab,path,m};int rc=check_reloc_case(5,args,&bad,&total);if(rc>code)code=rc;cases++;if(rc==2)break;
 }
 if(ferror(list)||!feof(list)||!cases||!total)code=2;
 fclose(list);printf("%s: %zu/%zu\n",argv[1],bad,total);return code;
}
