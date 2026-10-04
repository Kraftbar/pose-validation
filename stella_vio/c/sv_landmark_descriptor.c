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

/* Selection from stella_vslam e445b545 data/landmark.cc; ORB distance
 * from match/base.h. A row scratch buffer replaces the full distance matrix.
 * Only integer values are sorted; their tie order cannot affect the median. */
#include "sv_landmark_descriptor.h"
#include <stdlib.h>
#include <string.h>

typedef struct { uint32_t id; size_t index; } ordered_observation;
static int compare_id(const void *a, const void *b)
{
    uint32_t x=((const ordered_observation *)a)->id;
    uint32_t y=((const ordered_observation *)b)->id;
    return (x>y)-(x<y);
}
static int compare_distance(const void *a, const void *b)
{
    unsigned x=*(const unsigned *)a, y=*(const unsigned *)b;
    return (x>y)-(x<y);
}
static unsigned distance(const uint8_t *a,const uint8_t *b)
{
    unsigned sum=0;
    for(unsigned i=0;i<32;++i){
        unsigned v=a[i]^b[i];
        v-= (v>>1)&0x55u;
        v= (v&0x33u)+((v>>2)&0x33u);
        sum+=(v+(v>>4))&0x0fu;
    }
    return sum;
}
int sv_landmark_select_descriptor(const sv_descriptor_observation *obs,
                                  size_t count, sv_landmark_descriptor_result *out,
                                  uint16_t *medians)
{
    if(!obs || !count || !out || count>SIZE_MAX/sizeof(ordered_observation)
        || count>SIZE_MAX/sizeof(unsigned))return -1;
    ordered_observation *order=malloc(count*sizeof(*order));
    unsigned *row=malloc(count*sizeof(*row));
    if(!order || !row){free(order);free(row);return -1;}
    size_t live=0;
    for(size_t i=0;i<count;++i){
        if(!obs[i].erased && !obs[i].descriptor){free(order);free(row);return -1;}
        order[i].id=obs[i].keyframe_id; order[i].index=i;
        live+=!obs[i].erased;
    }
    qsort(order,count,sizeof(*order),compare_id);
    for(size_t i=1;i<count;++i)
        if(order[i-1].id==order[i].id){free(order);free(row);return -1;}
    if(!live){free(order);free(row);return -1;}
    size_t n=0;
    for(size_t i=0;i<count;++i)
        if(!obs[order[i].index].erased)order[n++]=order[i];
    if(medians)for(size_t i=0;i<count;++i)medians[i]=UINT16_MAX;
    unsigned best_median=256;
    size_t best=order[0].index;
    for(size_t i=0;i<live;++i){
        size_t index=order[i].index;
        for(size_t j=0;j<live;++j)
            row[j]=distance(obs[index].descriptor,obs[order[j].index].descriptor);
        qsort(row,live,sizeof(*row),compare_distance);
        unsigned median=row[(live-1)/2];
        if(medians)medians[index]=(uint16_t)median;
        if(median<best_median){best_median=median;best=index;}
    }
    out->observation_index=best;out->keyframe_id=obs[best].keyframe_id;
    out->median_distance=best_median;
    memcpy(out->descriptor,obs[best].descriptor,32);
    free(order);free(row);return 0;
}
