#ifndef SV_RELOC_TRACE_HPP
#define SV_RELOC_TRACE_HPP
#include "pnp_trace.hpp"
#include <stella_vslam/data/frame.h>
#include <stella_vslam/data/keyframe.h>
#include <stella_vslam/data/landmark.h>
namespace reloc_trace {
template<class V>void ids(const char*name,const V&v){std::vector<double>a;for(auto&p:v)a.push_back(p?double(p->id_):-1.0);emit(name,a.data(),a.size());}
inline void frame(const char*name,const stella_vslam::data::frame&f){
 std::string s(name);scalar((s+"_valid").c_str(),f.pose_is_valid());
 if(f.pose_is_valid())mat((s+"_pose").c_str(),f.get_pose_cw());
 ids((s+"_landmarks").c_str(),f.get_landmarks());scalar((s+"_ref").c_str(),f.ref_keyfrm_?double(f.ref_keyfrm_->id_):-1.0);
}
extern void (*pnp_input)(const stella_vslam::eigen_alloc_vector<stella_vslam::Vec3_t>&,
 const std::vector<int>&,const stella_vslam::eigen_alloc_vector<stella_vslam::Vec3_t>&,const std::vector<float>&);
}
#endif
