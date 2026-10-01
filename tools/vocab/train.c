/* SPDX-License-Identifier: MIT
 * Independent binary vocabulary-tree trainer. Standard-library C99 + libm.
 * D^2 k-means++ seeding with Hamming distance, bit-majority centers (ties 0),
 * Lloyd assignment, deterministic lowest-center ties, empty-cluster removal.
 * Own implementation; no DBoW/ORB-SLAM vocabulary or trainer is consulted.
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#define KMAX 16
/* The file uses explicit little-endian fields, never dumped C structs. */
typedef struct {uint32_t doc;uint8_t bits[32];} Record;
typedef struct {uint8_t center[32];uint32_t child[KMAX],count,word,block,parent;} Node;
static Record *data;static Node *nodes;static uint32_t nn,cap,words,branch,depth,steps;static uint64_t rng;
static void die(const char*s){fprintf(stderr,"%s\n",s);exit(1);}
static void*memory(size_t n,size_t size){if(n>SIZE_MAX/size)die("allocation overflow");void*p=calloc(n?n:1,size);if(!p)die("out of memory");return p;}
static uint64_t random64(void){uint64_t z=(rng+=UINT64_C(0x9e3779b97f4a7c15));z=(z^(z>>30))*UINT64_C(0xbf58476d1ce4e5b9);z=(z^(z>>27))*UINT64_C(0x94d049bb133111eb);return z^(z>>31);}
static uint64_t bounded(uint64_t n){uint64_t r,limit=(uint64_t)(-n)%n;do{r=random64();}while(r<limit);return r%n;}
static unsigned distance(const uint8_t*a,const uint8_t*b){unsigned d=0;for(int j=0;j<4;j++){uint64_t x,y;memcpy(&x,a+8*j,8);memcpy(&y,b+8*j,8);d+=(unsigned)__builtin_popcountll(x^y);}return d;}
static uint32_t new_node(const uint8_t*center,uint32_t parent){
 if(nn==cap){uint32_t next=cap?cap*2:1024;if(next<cap)die("too many nodes");void*p=realloc(nodes,(size_t)next*sizeof(Node));if(!p)die("out of memory");nodes=p;memset(nodes+cap,0,(next-cap)*sizeof(Node));cap=next;}
 uint32_t id=nn++;if(center)memcpy(nodes[id].center,center,32);nodes[id].parent=parent;return id;
}
static unsigned nearest(const uint8_t *d,uint8_t c[KMAX][32],unsigned k){unsigned at=0,best=257;for(unsigned j=0;j<k;j++){unsigned x=distance(d,c[j]);if(x<best){best=x;at=j;}}return at;}
static void grow(uint32_t node,uint32_t*ids,uint32_t n,unsigned level){
 if(level==depth){nodes[node].word=words++;return;}
 uint8_t centers[KMAX][32];uint32_t *min=memory(n,sizeof(uint32_t));uint8_t *assignment=memory(n,1);memset(assignment,255,n);
 memcpy(centers[0],data[ids[bounded(n)]].bits,32);unsigned k=1;for(uint32_t i=0;i<n;i++)min[i]=257;
 while(k<branch){
  uint64_t sum=0;
  for(uint32_t i=0;i<n;i++){unsigned d=distance(data[ids[i]].bits,centers[k-1]);if(d<min[i])min[i]=d;sum+=(uint64_t)min[i]*min[i];}
  if(!sum)break;
  uint64_t pick=bounded(sum);uint32_t chosen=0;
  for(;chosen<n;chosen++){uint64_t weight=(uint64_t)min[chosen]*min[chosen];if(pick<weight)break;pick-=weight;}
  if(chosen==n)die("seeding error");
  memcpy(centers[k++],data[ids[chosen]].bits,32);
 }
 free(min);
 uint32_t counts[KMAX];
 for(unsigned iter=0;iter<steps;iter++){
  uint32_t ones[KMAX][256]={{0}};memset(counts,0,sizeof(counts));unsigned changed=0;
  for(uint32_t i=0;i<n;i++){
   const uint8_t*d=data[ids[i]].bits;unsigned c=nearest(d,centers,k);changed+=assignment[i]!=c;assignment[i]=(uint8_t)c;counts[c]++;
   for(unsigned byte=0;byte<32;byte++)for(unsigned bit=0;bit<8;bit++)ones[c][8*byte+bit]+=(d[byte]>>bit)&1;
  }
  if(!changed)break;
  for(unsigned c=0;c<k;c++)if(counts[c]){
   memset(centers[c],0,32);for(unsigned bit=0;bit<256;bit++)if(ones[c][bit]>counts[c]/2)centers[c][bit/8]|=(uint8_t)(1u<<(bit%8));
  }
 }
 /* Final memberships must correspond to the centers actually exported. */
 memset(counts,0,sizeof(counts));for(uint32_t i=0;i<n;i++){assignment[i]=(uint8_t)nearest(data[ids[i]].bits,centers,k);counts[assignment[i]]++;}
 uint32_t offsets[KMAX],cursor[KMAX],*ordered=memory(n,sizeof(uint32_t));unsigned active=0;uint32_t offset=0;
 for(unsigned c=0;c<k;c++){offsets[c]=cursor[c]=offset;offset+=counts[c];if(counts[c])active++;}
 for(uint32_t i=0;i<n;i++)ordered[cursor[assignment[i]]++]=ids[i];
 free(assignment);
 for(unsigned c=0;c<k;c++)if(counts[c]){
  uint32_t child=new_node(centers[c],node);nodes[node].child[nodes[node].count++]=child;
  /* Single-center clusters cannot split, but the root still needs a block. */
  if(counts[c]==1||active==1||level+1==depth)nodes[child].word=words++;
  else grow(child,ordered+offsets[c],counts[c],level+1);
 }
 free(ordered);
}
static uint32_t word_of(const uint8_t*bits){uint32_t n=0;while(nodes[n].count){unsigned best=257;uint32_t chosen=0;for(unsigned j=0;j<nodes[n].count;j++){uint32_t c=nodes[n].child[j];unsigned d=distance(bits,nodes[c].center);if(d<best){best=d;chosen=c;}}n=chosen;}return nodes[n].word;}
static uint32_t u32(const uint8_t*p){return (uint32_t)p[0]|(uint32_t)p[1]<<8|(uint32_t)p[2]<<16|(uint32_t)p[3]<<24;}
static void put(uint8_t*p,uint64_t v,unsigned n){for(unsigned i=0;i<n;i++)p[i]=(uint8_t)(v>>(8*i));}
static void write_bytes(FILE*f,const void*p,size_t n){if(fwrite(p,1,n,f)!=n)die("write error");}
int main(int argc,char**argv){
 if(argc!=8)die("usage: train descriptors output.fbow k depth iterations seed stats.json");
 branch=(unsigned)strtoul(argv[3],NULL,10);depth=(unsigned)strtoul(argv[4],NULL,10);steps=(unsigned)strtoul(argv[5],NULL,10);rng=strtoull(argv[6],NULL,10);uint64_t seed=rng;
 if(branch<2||branch>KMAX||!depth||depth>7||!steps||steps>100)die("invalid parameters");
 FILE*f=fopen(argv[1],"rb");if(!f)die("input open failed");uint8_t header[16];if(fread(header,1,16,f)!=16||memcmp(header,"SVORBD01",8))die("invalid descriptor header");
 uint32_t docs=u32(header+8),n=u32(header+12);if(!n||!docs||n>100000000||docs>n)die("invalid descriptor counts");
 data=memory(n,sizeof(Record));uint32_t*ids=memory(n,sizeof(uint32_t));uint32_t previous=0;
 for(uint32_t i=0;i<n;i++){uint8_t raw[36];if(fread(raw,1,36,f)!=36)die("truncated descriptors");data[i].doc=u32(raw);memcpy(data[i].bits,raw+4,32);if(data[i].doc>=docs||(i&&data[i].doc<previous))die("documents must be ordered");previous=data[i].doc;ids[i]=i;}
 if(fgetc(f)!=EOF||ferror(f))die("trailing descriptor data");
 fclose(f);
 new_node(NULL,0);grow(0,ids,n,0);free(ids);fprintf(stderr,"tree: %u nodes, %u words\n",nn,words);
 uint32_t*df=memory(words,sizeof(uint32_t)),*last=memory(words,sizeof(uint32_t));for(uint32_t w=0;w<words;w++)last[w]=UINT32_MAX;
 for(uint32_t i=0;i<n;i++){uint32_t w=word_of(data[i].bits);if(last[w]!=data[i].doc){df[w]++;last[w]=data[i].doc;}}
 uint32_t blocks=0;for(uint32_t i=0;i<nn;i++)if(nodes[i].count)nodes[i].block=blocks++;
 const uint32_t alignment=32,feature_offset=32,child_offset=32+branch*32,block_size=(32+branch*40+31)&~31u;
 uint8_t h[128]={0};put(h,55824124,8);memcpy(h+8,"ORB",3);put(h+60,alignment,4);put(h+64,blocks,4);put(h+72,32,8);put(h+80,block_size,8);put(h+88,feature_offset,8);put(h+96,child_offset,8);put(h+104,(uint64_t)blocks*block_size,8);put(h+112,0,4);put(h+116,32,4);put(h+120,branch,4);
 f=fopen(argv[2],"wb");if(!f)die("output open failed");write_bytes(f,h,sizeof(h));uint8_t *block=memory(block_size,1);unsigned zero_df=0;
 for(uint32_t i=0;i<nn;i++)if(nodes[i].count){
  memset(block,0,block_size);put(block,nodes[i].count,2);put(block+4,nodes[nodes[i].parent].block,4);int all_leaf=1;
  for(unsigned j=0;j<nodes[i].count;j++){
   Node*c=&nodes[nodes[i].child[j]];memcpy(block+feature_offset+j*32,c->center,32);
   if(c->count){put(block+child_offset+j*8,c->block,4);all_leaf=0;}
   else {uint32_t w=c->word;put(block+child_offset+j*8,UINT32_C(0x80000000)|w,4);float weight=(float)(log(((double)docs+1)/(df[w]+1))+1);uint32_t bits;memcpy(&bits,&weight,4);put(block+child_offset+j*8+4,bits,4);zero_df+=df[w]==0;}
  }
  put(block+2,all_leaf,2);write_bytes(f,block,block_size);
 }
 if(fclose(f))die("output close failed");
 f=fopen(argv[7],"w");if(!f)die("stats open failed");fprintf(f,"{\"documents\":%u,\"descriptors\":%u,\"nodes\":%u,\"words\":%u,\"blocks\":%u,\"zero_df_words\":%u,\"k\":%u,\"depth\":%u,\"iterations\":%u,\"seed\":%llu,\"idf\":\"log((documents+1)/(df+1))+1\"}\n",docs,n,nn,words,blocks,zero_df,branch,depth,steps,(unsigned long long)seed);if(fclose(f))die("stats close failed");
 free(block);free(last);free(df);free(nodes);free(data);return 0;
}
