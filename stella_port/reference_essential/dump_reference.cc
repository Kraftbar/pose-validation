#include <stella_vslam/data/frame.h>
#include <stella_vslam/data/keyframe.h>
#include <stella_vslam/data/landmark.h>
#include <stella_vslam/camera/perspective.h>
#include <stella_vslam/match/robust.h>
#include <stella_vslam/solve/essential_solver.h>
#include <spdlog/spdlog.h>
#include "essential_solver.h"
#include "trace_hooks.h"
#include <fstream>
#include <stdexcept>
#include <string>
using namespace stella_vslam;
namespace essential_trace { FILE *file=nullptr;sv_essential_5pt_trace minimal{}; }
static data::frame_observation read(std::istream &in,unsigned n,camera::perspective &cam,std::vector<int> &ids){
 data::frame_observation obs{};obs.undist_keypts_.resize(n);obs.descriptors_=cv::Mat(n,32,CV_8UC1);ids.resize(n);
 for(unsigned i=0;i<n;i++){
  std::string x,y,angle,desc;in>>x>>y>>angle>>desc>>ids[i];if(!in || desc.size()!=64)throw std::runtime_error("bad row");
  obs.undist_keypts_[i].pt={std::stof(x),std::stof(y)};obs.undist_keypts_[i].angle=std::stof(angle);
  for(unsigned j=0;j<32;j++)obs.descriptors_.at<unsigned char>(i,j)=std::stoul(desc.substr(2*j,2),nullptr,16);
  obs.bearings_.push_back(cam.convert_point_to_bearing(obs.undist_keypts_[i].pt));
 }
 return obs;
}
int main(int argc,char **argv){try{
 if(argc!=3)throw std::runtime_error("usage: input output");
 std::ifstream in(argv[1]);unsigned frame,n1,n2;if(!(in>>frame>>n1>>n2))throw std::runtime_error("bad header");
 spdlog::set_level(spdlog::level::off);
 camera::perspective cam("leaf",camera::setup_type_t::Monocular,camera::color_order_t::RGB,640,480,30,517.306408,516.469215,318.643040,255.313989,0,0,0,0,0);
 std::vector<int> ids1,ids2;auto obs1=read(in,n1,cam,ids1),obs2=read(in,n2,cam,ids2);
 auto k=data::keyframe::make_keyframe(1,0,Mat44_t::Identity(),&cam,nullptr,obs2,data::bow_vector{},data::bow_feature_vector{});
 for(unsigned i=0;i<n2;i++)if(ids2[i]>=0)k->add_landmark(std::make_shared<data::landmark>(ids2[i],Vec3_t::Zero(),k),i);
 match::robust matcher(.8,true);std::vector<std::pair<int,int>> pairs;matcher.brute_force_match(obs1,k,pairs);
 essential_trace::file=std::fopen(argv[2],"wb");if(!essential_trace::file)throw std::runtime_error("cannot write");
 unsigned n=pairs.size(),iters=1000;
 essential_trace::raw(&n,4);essential_trace::raw(&iters,4);
 for(auto &p:pairs){essential_trace::raw(&p.first,4);essential_trace::raw(&p.second,4);essential_trace::raw(obs1.bearings_[p.first].data(),24);essential_trace::raw(obs2.bearings_[p.second].data(),24);}
 essential_reference::essential_solver observed(obs1.bearings_,obs2.bearings_,pairs,true);observed.find_via_ransac(iters,true);
 unsigned valid=observed.solution_is_valid();float cost=observed.get_best_cost();auto mask=observed.get_inlier_matches();
 essential_trace::raw(&valid,4);essential_trace::raw(&cost,4);if(valid){auto e=observed.get_best_E_21();essential_trace::raw(e.data(),72);}
 for(bool b:mask){unsigned char v=b;essential_trace::raw(&v,1);}std::fclose(essential_trace::file);essential_trace::file=nullptr;
 // Trace patch must not alter public native solver behavior.
 solve::essential_solver native(obs1.bearings_,obs2.bearings_,pairs,true);native.find_via_ransac(iters,true);
 if(valid!=unsigned(native.solution_is_valid()) || cost!=native.get_best_cost() || mask!=native.get_inlier_matches())throw std::runtime_error("trace patch changed result");
 if(valid){auto a=observed.get_best_E_21(),b=native.get_best_E_21();if(std::memcmp(a.data(),b.data(),72))throw std::runtime_error("trace patch changed matrix");}
 data::frame f;f.frm_obs_=obs1;std::vector<std::shared_ptr<data::landmark>> matches;
 unsigned count=matcher.match_frame_and_keyframe(f,k,matches,true),expected=0;
 for(unsigned i=0;i<n;i++)if(mask[i]){expected++;if(!matches[pairs[i].first] || int(matches[pairs[i].first]->id_)!=ids2[pairs[i].second])throw std::runtime_error("native matcher disagrees");}
 if(count!=expected)throw std::runtime_error("native count disagrees");
 std::printf("frame %u: %u correspondences, %u inliers; original solver/matcher agree\n",frame,n,count);
 }catch(const std::exception&e){std::fprintf(stderr,"%s\n",e.what());return 2;}}
