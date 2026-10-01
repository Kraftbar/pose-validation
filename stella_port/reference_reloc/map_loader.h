#ifndef SV_RELOC_MAP_LOADER_H
#define SV_RELOC_MAP_LOADER_H
#include "../c/sv_relocalizer.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
typedef struct {sv_tr_kf k;sv_tr_obs obs;sv_bow_vector bow;sv_bow_db_keyframe dbkey;} loaded_key;
typedef struct {sv_tr_config cfg;sv_tr_map map;sv_tr_frame current;sv_tr_obs obs;sv_bow_vector query;sv_bow_db*db;loaded_key**keys;} loaded_map;
static void read_exact(FILE*f,void*p,size_t n){if(fread(p,1,n,f)!=n){fprintf(stderr,"truncated map\n");exit(2);}}
static unsigned read_u(FILE*f){unsigned x;read_exact(f,&x,4);if(x>1000000){fprintf(stderr,"map count too large\n");exit(2);}return x;}
static int read_i(FILE*f){int x;read_exact(f,&x,4);return x;}
static double read_d(FILE*f){double x;read_exact(f,&x,8);return x;}
static void *alloc(size_t n,size_t sz){void*p=calloc(n?n:1,sz);if(!p)exit(2);return p;}
static void load_obs(FILE*f,const sv_tr_config*c,sv_tr_obs*o,sv_bow_vector*bow){
 unsigned n=read_u(f);sv_keypoint*kp=alloc(n,sizeof(*kp));uint8_t*desc=alloc(n,32);
 for(unsigned i=0;i<n;i++){read_exact(f,&kp[i].x,4);read_exact(f,&kp[i].y,4);read_exact(f,&kp[i].angle,4);kp[i].octave=read_i(f);read_exact(f,desc+32*i,32);kp[i].size=1;}
 if(sv_tr_obs_init(o,c,kp,desc,n)||sv_bow_transform(c->vocab,desc,n,4,bow,&o->bow_feat))exit(2);o->bow_ready=1;
}
static void free_obs(sv_tr_obs*o){free((void*)o->kp);free((void*)o->desc);sv_tr_obs_free(o);}
static void load_map(const char*path,const sv_bow_vocab*v,loaded_map*w){
 memset(w,0,sizeof(*w));sv_camera_params cam={517.306408,516.469215,318.643040,255.313989,0.262383,-0.953104,-0.005358,0.002628,1.163314};sv_image_bounds bounds;sv_compute_image_bounds(&cam,640,480,&bounds);sv_tr_config_init(&w->cfg,cam.fx,cam.fy,cam.cx,cam.cy,&bounds,v);w->db=sv_bow_db_create();
 FILE*f=fopen(path,"rb");if(!f)exit(2);char magic[8];read_exact(f,magic,8);if(memcmp(magic,"SVRELOC1",8))exit(2);
 unsigned id=read_u(f);double ts=read_d(f);int ref=read_i(f);unsigned nk=read_u(f),nl=read_u(f);
 load_obs(f,&w->cfg,&w->obs,&w->query);sv_tr_frame_init(&w->current,id,ts,&w->obs);w->current.ref_kf=ref;
 double identity[16]={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};sv_tr_frame_set_pose_cw(&w->current,identity);w->current.pose_valid=0;
 for(unsigned i=0;i<nk;i++){
  unsigned id=read_u(f);double ts=read_d(f);loaded_key*k=alloc(1,sizeof(*k));k->k.id=id;k->k.timestamp=ts;k->k.alive=1;k->k.obs=&k->obs;
  if(id>=w->map.kf_cap){unsigned old=w->map.kf_cap;w->map.kf_cap=id+1;w->keys=realloc(w->keys,w->map.kf_cap*sizeof(*w->keys));w->map.kfs=realloc(w->map.kfs,w->map.kf_cap*sizeof(*w->map.kfs));if(!w->keys||!w->map.kfs)exit(2);for(unsigned j=old;j<w->map.kf_cap;j++){w->keys[j]=NULL;w->map.kfs[j]=NULL;}}
  w->keys[id]=k;w->map.kfs[id]=&k->k;double pose[16];read_exact(f,pose,128);sv_tr_kf_set_pose_cw(&k->k,pose);load_obs(f,&w->cfg,&k->obs,&k->bow);
  k->k.lm=alloc(k->obs.num_kp,sizeof(int));for(unsigned j=0;j<k->obs.num_kp;j++)k->k.lm[j]=-1;
  k->k.n_covis=read_u(f);k->k.covis=alloc(k->k.n_covis,sizeof(unsigned));k->k.covis_w=alloc(k->k.n_covis,sizeof(unsigned));
  for(unsigned j=0;j<k->k.n_covis;j++){k->k.covis[j]=read_u(f);k->k.covis_w[j]=read_u(f);}
  k->k.parent=read_i(f);k->k.is_root=k->k.parent<0;k->k.n_children=read_u(f);k->k.children=alloc(k->k.n_children,sizeof(unsigned));for(unsigned j=0;j<k->k.n_children;j++)k->k.children[j]=read_u(f);
  k->dbkey=(sv_bow_db_keyframe){id,&k->bow};if(sv_bow_db_add(w->db,&k->dbkey))exit(2);
 }
 for(unsigned i=0;i<nl;i++){
  unsigned id=read_u(f);sv_tr_lm*lm=alloc(1,sizeof(*lm));lm->id=id;lm->alive=1;read_exact(f,lm->pos_w,24);read_exact(f,lm->mean_normal,24);read_exact(f,&lm->min_valid_dist,4);read_exact(f,&lm->max_valid_dist,4);read_exact(f,lm->desc,32);lm->ref_kf=read_i(f);lm->num_obs=read_u(f);lm->obs_kf=alloc(lm->num_obs,sizeof(unsigned));lm->obs_idx=alloc(lm->num_obs,sizeof(unsigned));
  if(id>=w->map.lm_cap){unsigned old=w->map.lm_cap;w->map.lm_cap=id+1;w->map.lms=realloc(w->map.lms,w->map.lm_cap*sizeof(*w->map.lms));if(!w->map.lms)exit(2);for(unsigned j=old;j<w->map.lm_cap;j++)w->map.lms[j]=NULL;}
  w->map.lms[id]=lm;for(unsigned j=0;j<lm->num_obs;j++){unsigned k=read_u(f),idx=read_u(f);lm->obs_kf[j]=k;lm->obs_idx[j]=idx;if(k>=w->map.kf_cap||!w->map.kfs[k]||idx>=w->map.kfs[k]->obs->num_kp)exit(2);w->map.kfs[k]->lm[idx]=(int)id;}
 }
 w->map.num_keyframes=nk;if(fgetc(f)!=EOF||ferror(f))exit(2);fclose(f);
}
static void free_map(loaded_map*w){
 sv_bow_db_destroy(w->db);sv_tr_frame_free(&w->current);free_obs(&w->obs);sv_bow_vector_free(&w->query);
 for(unsigned i=0;i<w->map.kf_cap;i++)if(w->keys[i]){loaded_key*k=w->keys[i];free(k->k.lm);free(k->k.covis);free(k->k.covis_w);free(k->k.children);free_obs(&k->obs);sv_bow_vector_free(&k->bow);free(k);}
 for(unsigned i=0;i<w->map.lm_cap;i++)if(w->map.lms[i]){sv_tr_lm*lm=w->map.lms[i];free(lm->obs_kf);free(lm->obs_idx);free(lm);}
 free(w->keys);free(w->map.kfs);free(w->map.lms);
}
static unsigned char*load_vocab(const char*path,sv_bow_vocab*v){FILE*f=fopen(path,"rb");if(!f)exit(2);fseek(f,0,SEEK_END);long n=ftell(f);if(n<128)exit(2);rewind(f);unsigned char*p=alloc((size_t)n,1);read_exact(f,p,(size_t)n);fclose(f);if(sv_bow_load_memory(p,(size_t)n,v))exit(2);return p;}
#endif
