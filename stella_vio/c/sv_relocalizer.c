/* SPDX-License-Identifier: BSD-2-Clause */
/* BSD 2-Clause License
 * Copyright (c) 2019, National Institute of Advanced Industrial Science
 * and Technology (AIST), All rights reserved.
 * Copyright (c) 2022, stella-cv, All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


/* stella_vslam e445b545 module/relocalizer.cc and its projection paths. */
#include "sv_relocalizer.h"
#include "sv_poselib.h"
#include "sv_match_bow.h"
#include "sv_solve_essential_ransac.h"
#include "sv_linalg.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
typedef struct {sv_pnp_trace_fn fn;void *user;} reloc_trace;
static void emit(reloc_trace*t,const char*s,const double*p,unsigned n){if(t->fn)t->fn(t->user,s,p,n);}
static void scalar(reloc_trace*t,const char*s,double v){emit(t,s,&v,1);}
static void ids(reloc_trace*t,const char*s,const int*p,unsigned n){
 if(!t->fn)return;
 double *a=malloc((n?n:1)*sizeof(double));if(!a)return;
 for(unsigned i=0;i<n;i++)a[i]=p[i];
 emit(t,s,a,n);free(a);
}
static void uids(reloc_trace*t,const char*s,const unsigned*p,unsigned n){
 if(!t->fn)return;
 double *a=malloc((n?n:1)*sizeof(double));if(!a)return;
 for(unsigned i=0;i<n;i++)a[i]=p[i];
 emit(t,s,a,n);free(a);
}
static void frame_trace(reloc_trace*t,const char*name,const sv_tr_frame*f){
 if(!t->fn)return;
 char s[80];snprintf(s,sizeof(s),"%s_valid",name);scalar(t,s,f->pose_valid);
 if(f->pose_valid){snprintf(s,sizeof(s),"%s_pose",name);emit(t,s,f->pose_cw,16);}
 snprintf(s,sizeof(s),"%s_landmarks",name);ids(t,s,f->lm,f->obs->num_kp);
 snprintf(s,sizeof(s),"%s_ref",name);scalar(t,s,f->ref_kf);
}
void sv_reloc_config_init(sv_reloc_config*c){
 *c=(sv_reloc_config){.75f,.9f,.8f,.8f,20,50,10,30,60,1};
}
static int matches(const sv_reloc_config*c,const sv_tr_config*cfg,const sv_tr_map*m,
                    sv_tr_frame*f,const sv_tr_kf*k,int robust,int*out){
 unsigned n=f->obs->num_kp,nk=k->obs->num_kp;int count=0;
 for(unsigned i=0;i<n;i++)out[i]=-1;
 if(!robust){
  if(sv_tr_obs_ensure_bow(f->obs,cfg)||sv_tr_obs_ensure_bow(k->obs,cfg))return -1;
  uint64_t *tokens=calloc(nk?nk:1,sizeof(uint64_t)),*matched=calloc(n?n:1,sizeof(uint64_t));
  if(!tokens||!matched){free(tokens);free(matched);return -1;}
  for(unsigned i=0;i<nk;i++)if(sv_tr_map_lm(m,k->lm[i]))tokens[i]=(uint64_t)k->lm[i]+1;
  sv_match_bow_view kv={nk,k->obs->kp,k->obs->desc,&k->obs->bow_feat,tokens,NULL};
  sv_match_bow_view fv={n,f->obs->kp,f->obs->desc,&f->obs->bow_feat,NULL,NULL};uint32_t nc=0;
  int rc=sv_match_bow_frame(&kv,&fv,c->bow_ratio,0,matched,&nc);
  if(!rc){count=(int)nc;for(unsigned i=0;i<n;i++)if(matched[i])out[i]=(int)(matched[i]-1);}else count=-1;
  free(tokens);free(matched);return count;
 }
 if(sv_tr_obs_ensure_bearings(f->obs,cfg)||sv_tr_obs_ensure_bearings(k->obs,cfg))return -1;
 int *pairs=malloc((n?n:1)*sizeof(int));double *a=malloc((n?n:1)*24),*b=malloc((n?n:1)*24);unsigned char *mask=malloc(n?n:1);
 if(!pairs||!a||!b||!mask){free(pairs);free(a);free(b);free(mask);return -1;}
 for(unsigned i=0;i<n;i++)pairs[i]=-1;
 for(unsigned j=0;j<nk;j++){
  if(!sv_tr_map_lm(m,k->lm[j]))continue;
  unsigned best=256,second=256;int ix=-1;
  for(unsigned i=0;i<n;i++)if(pairs[i]<0){unsigned d=sv_tr_hamming(k->obs->desc+32*j,f->obs->desc+32*i);if(d<best){second=best;best=d;ix=(int)i;}else if(d<second)second=d;}
  if(best>50||ix<0||c->robust_ratio*second<(float)best)continue;
  pairs[ix]=(int)j;
 }
 unsigned nc=0;for(unsigned i=0;i<n;i++)if(pairs[i]>=0){memcpy(a+3*nc,f->obs->bearings+3*i,24);memcpy(b+3*nc,k->obs->bearings+3*pairs[i],24);nc++;}
 sv_essential_result result;int rc=sv_essential_ransac(a,b,nc,1000,1,5,NULL,&result,mask,NULL,NULL);
 if(rc)count=-1;else if(result.valid){nc=0;for(unsigned i=0;i<n;i++)if(pairs[i]>=0&&mask[nc++]){out[i]=k->lm[pairs[i]];count++;}}
 free(pairs);free(a);free(b);free(mask);return count;
}
static unsigned scale_level(const sv_tr_config*c,const sv_tr_lm*lm,float dist){
 float ratio=lm->max_valid_dist/dist;int level=(int)ceilf(logf(ratio)/c->log_scale_factor);
 return level<0?0:(unsigned)level>=c->num_levels?c->num_levels-1:(unsigned)level;
}
static unsigned projection_keyframe(const sv_tr_config*c,const sv_tr_map*m,sv_tr_frame*f,
                                     const sv_tr_kf*k,const unsigned char*seen,float margin,unsigned threshold){
 unsigned *indices=malloc((f->obs->num_kp?f->obs->num_kp:1)*sizeof(unsigned));if(!indices)return 0;unsigned count=0;
 for(unsigned i=0;i<k->obs->num_kp;i++){
  const sv_tr_lm*lm=sv_tr_map_lm(m,k->lm[i]);if(!lm||seen[lm->id])continue;
  double rp[2],v[3];float xr;if(!sv_tr_reproject_to_image(c,f->rot_cw,f->trans_cw,lm->pos_w,rp,&xr))continue;
  for(int j=0;j<3;j++)v[j]=lm->pos_w[j]-f->trans_wc[j];
  double dist=sv_vec3_norm(v);
  if(dist<(1.0/1.3)*lm->min_valid_dist||1.3*lm->max_valid_dist<dist)continue;
  unsigned level=scale_level(c,lm,(float)dist);int lo=level?(int)level-1:0,hi=level+1<c->num_levels?(int)level+1:(int)c->num_levels-1;
  unsigned ni=sv_frame_get_keypoints_in_cell(&f->obs->grid,f->obs->kp,(float)rp[0],(float)rp[1],margin*c->scale_factors[level],lo,hi,indices,f->obs->num_kp);
  unsigned best=256;int ix=-1;
  for(unsigned j=0;j<ni;j++){unsigned q=indices[j];if(f->lm[q]>=0)continue;unsigned d=sv_tr_hamming(lm->desc,f->obs->desc+32*q);if(d<best){best=d;ix=(int)q;}}
  if(best>threshold)continue;
  f->lm[ix]=(int)lm->id;count++;
 }free(indices);return count;
}
static int visible(const sv_tr_config*c,const sv_tr_frame*f,const sv_tr_lm*lm,double rp[2],unsigned*level){
 float xr;if(!sv_tr_reproject_to_image(c,f->rot_cw,f->trans_cw,lm->pos_w,rp,&xr))return 0;
 double v[3];for(int i=0;i<3;i++)v[i]=lm->pos_w[i]-f->trans_wc[i];double dist=sv_vec3_norm(v);float df=(float)dist;
 if(!((float)(1.0/1.3)*lm->min_valid_dist<=df&&df<=1.3f*lm->max_valid_dist))return 0;
 if(sv_vec3_dot(v,lm->mean_normal)/dist<.5)return 0;
 *level=scale_level(c,lm,df);return 1;
}
static unsigned optimize(const sv_tr_config*c,const sv_tr_map*m,sv_tr_frame*f,unsigned char*outlier){
 double pose[16];unsigned count=sv_tr_optimize_pose(c,m,f,pose,outlier);sv_tr_frame_set_pose_cw(f,pose);return count;
}
static void discard(sv_tr_frame*f,const unsigned char*outlier){for(unsigned i=0;i<f->obs->num_kp;i++)if(outlier[i])f->lm[i]=-1;}
static int local_refine(const sv_reloc_config*c,const sv_tr_config*cfg,const sv_tr_map*m,sv_tr_frame*f,const sv_tr_kf*k,reloc_trace*tr,unsigned char*outlier){
 sv_tr_local_map local={0};local.nearest_covisibility=-1;sv_tr_config lc=*cfg;lc.max_num_local_keyfrms=c->max_local_keyframes;
 if(!sv_tr_acquire_local_map(&lc,m,f->lm,f->obs->num_kp,&local)){sv_tr_local_map_free(&local);return 0;}
 uids(tr,"local_keys",local.kfs,local.n_kfs);uids(tr,"local_landmarks",local.lms,local.n_lms);
 unsigned char*seen=calloc(m->lm_cap?m->lm_cap:1,1);unsigned char*cand=calloc(local.n_lms?local.n_lms:1,1);
 double*rp=malloc((local.n_lms?local.n_lms:1)*2*sizeof(double));unsigned*levels=malloc((local.n_lms?local.n_lms:1)*sizeof(unsigned));unsigned*indices=malloc((f->obs->num_kp?f->obs->num_kp:1)*sizeof(unsigned));
 int ok=0;if(!seen||!cand||!rp||!levels||!indices)goto done;
 const float margins[3]={5,15,5};
 for(int iter=0;iter<3;iter++){
  memset(seen,0,m->lm_cap);memset(cand,0,local.n_lms);
  for(unsigned i=0;i<f->obs->num_kp;i++){const sv_tr_lm*lm=sv_tr_map_lm(m,f->lm[i]);if(lm)seen[lm->id]=1;}
  int found=0;for(unsigned i=0;i<local.n_lms;i++){
   const sv_tr_lm*lm=sv_tr_map_lm(m,(int)local.lms[i]);if(!lm||seen[lm->id])continue;
   if(visible(cfg,f,lm,rp+2*i,levels+i)){found=1;cand[i]=1;}
  }
  scalar(tr,"local_visible",found);if(!found)goto done;unsigned added=0;
  for(unsigned i=0;i<local.n_lms;i++){
   if(!cand[i])continue;
   const sv_tr_lm*lm=sv_tr_map_lm(m,(int)local.lms[i]);unsigned level=levels[i];
   int lo=level?(int)level-1:0,hi=level+1<cfg->num_levels?(int)level+1:(int)cfg->num_levels-1;
   unsigned ni=sv_frame_get_keypoints_in_cell(&f->obs->grid,f->obs->kp,(float)rp[2*i],(float)rp[2*i+1],margins[iter]*cfg->scale_factors[level],lo,hi,indices,f->obs->num_kp);
   unsigned best=256,second=256;int ix=-1,best_level=-1,second_level=-1;
   for(unsigned j=0;j<ni;j++){
    unsigned q=indices[j];const sv_tr_lm*old=sv_tr_map_lm(m,f->lm[q]);if(old&&old->num_obs)continue;
    unsigned d=sv_tr_hamming(lm->desc,f->obs->desc+32*q);
    if(d<best){second=best;best=d;second_level=best_level;best_level=f->obs->kp[q].octave;ix=(int)q;}
    else if(d<second){second=d;second_level=f->obs->kp[q].octave;}
   }
   if(best<=100){if(best_level==second_level&&(float)best>.8f*(float)second)continue;f->lm[ix]=(int)lm->id;added++;}
  }
  scalar(tr,"local_additional",added);frame_trace(tr,"local_projection",f);
  unsigned valid=optimize(cfg,m,f,outlier);scalar(tr,"local_valid",valid);discard(f,outlier);
  if(iter==2){unsigned tracked=0;for(unsigned j=0;j<k->obs->num_kp;j++)if(sv_tr_map_lm(m,k->lm[j]))tracked++;if(valid<tracked*.2)goto done;}
 }ok=1;
done:free(seen);free(cand);free(rp);free(levels);free(indices);sv_tr_local_map_free(&local);return ok;
}
static int by_candidate(const sv_reloc_config*c,const sv_tr_config*cfg,const sv_tr_map*m,sv_tr_frame*f,const sv_tr_kf*k,int robust,reloc_trace*tr){
 unsigned n=f->obs->num_kp;int ok=0;int*matched=malloc((n?n:1)*sizeof(int));int*extra=malloc((n?n:1)*sizeof(int));
 unsigned char*inlier=calloc(n?n:1,1),*outlier=malloc(n?n:1),*seen=calloc(m->lm_cap?m->lm_cap:1,1);
 unsigned*indices=malloc((n?n:1)*sizeof(unsigned));int*octaves=malloc((n?n:1)*sizeof(int));double*b=malloc((n?n:1)*24),*p=malloc((n?n:1)*24);
 if(!matched||!extra||!inlier||!outlier||!seen||!indices||!octaves||!b||!p)goto done;
 int count=matches(c,cfg,m,f,k,robust,matched);
 if(count<0)goto done;
 scalar(tr,"initial_matches",count);ids(tr,"initial_landmarks",matched,n);
 if((unsigned)count<c->min_bow_matches)goto pnp_failed;
 if(c->search_neighbor){
  for(unsigned i=0;i<n;i++)if(sv_tr_map_lm(m,matched[i]))seen[matched[i]]=1;
  for(unsigned j=0;j<k->n_covis&&j<c->neighbors;j++){
   const sv_tr_kf*ngh=sv_tr_map_kf(m,(int)k->covis[j]);if(!ngh)continue;
   if(matches(c,cfg,m,f,ngh,robust,extra)<0)goto done;
   for(unsigned i=0;i<n;i++){const sv_tr_lm*lm=sv_tr_map_lm(m,extra[i]);if(!lm||seen[lm->id]||matched[i]>=0)continue;matched[i]=(int)lm->id;seen[lm->id]=1;}
  }
 }
 ids(tr,"expanded_landmarks",matched,n);
 if(sv_tr_obs_ensure_bearings(f->obs,cfg))goto done;
 unsigned nv=0;
 for(unsigned i=0;i<n;i++){
  const sv_tr_lm*lm=sv_tr_map_lm(m,matched[i]);if(!lm)continue;
  indices[nv]=i;octaves[nv]=f->obs->kp[i].octave;memcpy(b+3*nv,f->obs->bearings+3*i,24);memcpy(p+3*nv,lm->pos_w,24);nv++;
 }
 sv_pnp_result result;
 if(c->pnp_lo){if(sv_pnp_lo_ransac(b,p,octaves,nv,cfg->scale_factors,cfg->num_levels,10,1000,NULL,&result,inlier))goto done;}
 else if(sv_pnp_ransac(b,p,octaves,nv,cfg->scale_factors,cfg->num_levels,10,c->max_ransac_iters,10,0,NULL,&result,inlier,tr->fn,tr->user))goto done;
 if(!result.valid)goto pnp_failed;
 double pose[16]={0};for(int j=0;j<3;j++)for(int i=0;i<3;i++)pose[i+4*j]=result.rotation[i+3*j];for(int i=0;i<3;i++)pose[12+i]=result.translation[i];pose[15]=1;
 sv_tr_frame_set_pose_cw(f,pose);scalar(tr,"pnp_ok",1);frame_trace(tr,"pnp",f);
 for(unsigned i=0;i<n;i++)f->lm[i]=-1;
 for(unsigned i=0;i<nv;i++)if(inlier[i])f->lm[indices[i]]=matched[indices[i]];
 unsigned valid=optimize(cfg,m,f,outlier);
 int optimized=valid>=c->min_bow_matches/2;if(optimized)discard(f,outlier);
 scalar(tr,"optimize_ok",optimized);frame_trace(tr,"optimize",f);if(!optimized)goto done;
 memset(seen,0,m->lm_cap);unsigned found=0;
 for(unsigned i=0;i<nv;i++)if(inlier[i]&&!outlier[indices[i]]){unsigned id=(unsigned)matched[indices[i]];if(!seen[id]){seen[id]=1;found++;}}
 unsigned added=projection_keyframe(cfg,m,f,k,seen,10,100);scalar(tr,"projection10",added);frame_trace(tr,"projection10",f);
 if(found+added<c->min_valid_obs)goto refine_failed;
 valid=optimize(cfg,m,f,outlier);
 memset(seen,0,m->lm_cap);for(unsigned i=0;i<n;i++)if(f->lm[i]>=0)seen[f->lm[i]]=1;
 added=projection_keyframe(cfg,m,f,k,seen,3,64);scalar(tr,"projection3",added);frame_trace(tr,"projection3",f);
 if(valid+added<c->min_valid_obs)goto refine_failed;
 valid=optimize(cfg,m,f,outlier);if(valid<c->min_valid_obs)goto refine_failed;
 discard(f,outlier);scalar(tr,"refine_ok",1);frame_trace(tr,"refine",f);
 ok=local_refine(c,cfg,m,f,k,tr,outlier);scalar(tr,"local_ok",ok);frame_trace(tr,"local",f);goto done;
pnp_failed:scalar(tr,"pnp_ok",0);frame_trace(tr,"pnp",f);goto done;
refine_failed:scalar(tr,"refine_ok",0);frame_trace(tr,"refine",f);
done:free(matched);free(extra);free(inlier);free(outlier);free(seen);free(indices);free(octaves);free(b);free(p);return ok;
}
int sv_reloc_by_candidates(const sv_reloc_config*c,const sv_tr_config*cfg,const sv_tr_map*m,sv_tr_frame*f,const unsigned*keys,unsigned count,int robust,sv_pnp_trace_fn trace,void*user){
 reloc_trace tr={trace,user};
 for(unsigned i=0;i<count;i++){
  scalar(&tr,"candidate",keys[i]);const sv_tr_kf*k=sv_tr_map_kf(m,(int)keys[i]);if(!k)continue;
  int ok=by_candidate(c,cfg,m,f,k,robust,&tr);scalar(&tr,"candidate_ok",ok);frame_trace(&tr,"candidate",f);if(ok)return 1;
 }
 f->pose_valid=0;frame_trace(&tr,"failed",f);return 0;
}
int sv_relocalize(const sv_reloc_config*c,const sv_tr_config*cfg,const sv_tr_map*m,const sv_bow_db*db,const sv_bow_vector*query,sv_tr_frame*f,sv_pnp_trace_fn trace,void*user){
 sv_bow_db_result result={0};if(sv_bow_db_query(db,query,0,c->common_words_ratio,NULL,0,&result))return 0;
 unsigned*keys=calloc(result.accepted_count?result.accepted_count:1,sizeof(unsigned));if(!keys){sv_bow_db_result_free(&result);return 0;}
 unsigned count=0;for(size_t i=0;i<result.count;i++)if(result.matches[i].accepted)keys[count++]=result.matches[i].keyframe->id;
 sv_bow_db_result_free(&result);reloc_trace tr={trace,user};uids(&tr,"candidates",keys,count);
 int ok=count?sv_reloc_by_candidates(c,cfg,m,f,keys,count,0,trace,user):0;free(keys);return ok;
}
int sv_reloc_tracking_glue(const sv_reloc_config*c,sv_tracker*t,const sv_tr_map*m,const sv_bow_db*db,const sv_bow_vector*query,sv_pnp_trace_fn trace,void*user){
 if(!db)return 0;
 sv_bow_vector computed={0};sv_bow_feat_vector features={0};
 if(!query){
  if(!t->cfg->vocab||sv_tr_obs_ensure_bow(t->curr_frm.obs,t->cfg))return 0;
  if(sv_bow_transform(t->cfg->vocab,t->curr_frm.obs->desc,t->curr_frm.obs->num_kp,4,&computed,&features))return 0;
  sv_bow_feat_vector_free(&features);query=&computed;
 }
 int ok=sv_relocalize(c,t->cfg,m,db,query,&t->curr_frm,trace,user);
 if(ok){t->last_reloc_frm_id=t->curr_frm.id;t->last_reloc_frm_timestamp=t->curr_frm.timestamp;}
 sv_bow_vector_free(&computed);return ok;
}
