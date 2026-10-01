/* Staged: publish check_sv_reloc.c only after complete fixture validation. */
#include "map_loader.h"
#include "trace_compare.h"
static int check_reloc_case(int argc,char**argv,size_t*bad,size_t*total){
 if(argc!=5)return 2;sv_bow_vocab v;unsigned char*buf=load_vocab(argv[2],&v);loaded_map w;load_map(argv[1],&v,&w);int mode=atoi(argv[4]);
 trace_comparison c={0};c.f=fopen(argv[3],"rb");c.case_name=argv[1];if(!c.f)return 2;
 sv_reloc_config cfg;sv_reloc_config_init(&cfg);if(mode==3)cfg.min_valid_obs=10000;if(mode==4)cfg.search_neighbor=0;if(mode==2||mode==7)sv_bow_db_clear(w.db);
 unsigned ref=(unsigned)w.current.ref_kf;
 sv_tracker tracking={0};tracking.cfg=&w.cfg;tracking.curr_frm=w.current;tracking.last_reloc_frm_id=17;tracking.last_reloc_frm_timestamp=3.0;
 int ok=(mode==6||mode==7)?sv_reloc_tracking_glue(&cfg,&tracking,&w.map,w.db,NULL,compare_trace,&c):(mode==1||mode==5)?sv_reloc_by_candidates(&cfg,&w.cfg,&w.map,&w.current,&ref,1,mode==5,compare_trace,&c):sv_relocalize(&cfg,&w.cfg,&w.map,w.db,&w.query,&w.current,compare_trace,&c);
 if(mode==6||mode==7)w.current=tracking.curr_frm;
 compare_scalar(&c,"result",ok);compare_frame(&c,"result",&w.current);if(mode==6||mode==7){compare_scalar(&c,"tracking_id",tracking.last_reloc_frm_id);compare_scalar(&c,"tracking_timestamp",tracking.last_reloc_frm_timestamp);}if(fgetc(c.f)!=EOF||ferror(c.f))return 2;fclose(c.f);free_map(&w);free(buf);
 *bad+=c.bad;*total+=c.total;return c.bad?1:0;
}

#ifndef SV_RELOC_EMBEDDED
int main(int argc,char**argv){size_t bad=0,total=0;int rc=check_reloc_case(argc,argv,&bad,&total);printf("relocalizer: %zu/%zu\n",bad,total);return rc;}
#endif
