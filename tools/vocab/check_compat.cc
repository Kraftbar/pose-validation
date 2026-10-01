// SPDX-License-Identifier: MIT
#include <fbow/vocabulary.h>
extern "C" {
#include "../../stella_port/c/sv_bow.h"
}
#include <fstream>
#include <iostream>
#include <vector>
#include <cstring>
#include <cmath>
#include <cstdint>
static void read(std::istream&f,void*p,size_t n){if(!f.read((char*)p,n))throw std::runtime_error("truncated input");}
int main(int argc,char**argv){try{
 if(argc!=3)throw std::runtime_error("vocabulary descriptors required");
 fbow::Vocabulary vocab;vocab.readFromFile(argv[1]);std::ifstream vf(argv[1],std::ios::binary);std::vector<uint8_t>blob((std::istreambuf_iterator<char>(vf)),{});sv_bow_vocab cvocab;
 if(!vocab.isValid()||sv_bow_load_memory(blob.data(),blob.size(),&cvocab))throw std::runtime_error("invalid vocabulary");
 std::ifstream f(argv[2],std::ios::binary);char magic[8];uint32_t docs,n;read(f,magic,8);read(f,&docs,4);read(f,&n,4);if(memcmp(magic,"SVORBD01",8))throw std::runtime_error("bad descriptors");
 std::vector<std::vector<uint8_t>> images(docs);
 for(unsigned i=0;i<n;i++){uint32_t d;uint8_t bits[32];read(f,&d,4);read(f,bits,32);if(d>=docs)throw std::runtime_error("bad document");images[d].insert(images[d].end(),bits,bits+32);}
 size_t total=0,bad=0;fbow::BoWVector prev;sv_bow_vector cprev={};
 for(unsigned d=0;d<docs;d++){
  auto&desc=images[d];cv::Mat descriptors(desc.size()/32,32,CV_8U,desc.data());fbow::BoWVector bow;fbow::BoWFeatVector feat;vocab.transform(descriptors,4,bow,feat);
  sv_bow_vector cb={};sv_bow_feat_vector cf={};if(sv_bow_transform(&cvocab,desc.data(),desc.size()/32,4,&cb,&cf))throw std::runtime_error("C transform failed");
  if(bow.size()!=cb.count||feat.size()!=cf.count)throw std::runtime_error("size mismatch");
  unsigned j=0;for(auto&e:bow){total+=2;if(e.first!=cb.words[j].word_id||memcmp(&e.second,&cb.words[j].weight,4))bad++;j++;}
  j=0;for(auto&e:feat){total++;if(e.first!=cf.nodes[j].node_id||e.second.size()!=cf.nodes[j].count)throw std::runtime_error("feature node mismatch");for(unsigned k=0;k<e.second.size();k++){total++;if(e.second[k]!=cf.nodes[j].kp_indices[k])bad++;}j++;}
  if(d){double a=fbow::BoWVector::score(prev,bow),b=sv_bow_score(&cprev,&cb);total++;if(memcmp(&a,&b,8))bad++;}
  sv_bow_vector_free(&cprev);cprev=cb;prev=bow;sv_bow_feat_vector_free(&cf);
 }
 sv_bow_vector_free(&cprev);std::cout<<docs<<" documents, native/C FBoW: "<<bad<<"/"<<total<<"\n";return bad?1:0;
 }catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 2;}}
