#include <stella_vslam/data/frame.h>
#include <stella_vslam/data/keyframe.h>
#include <stella_vslam/data/common.h>
#include <stella_vslam/camera/perspective.h>
#include <stella_vslam/feature/orb_params.h>
#include <stella_vslam/data/bow_database.h>
#include <stella_vslam/optimize/pose_optimizer_g2o.h>
#include <stella_vslam/module/relocalizer.h>
#include <stella_vslam/data/map_database.h>
#include <fstream>
#include <iostream>
#include <cstring>
// Observer-only access to persisted landmark fields. No implementation copied.
#define private public
#include <stella_vslam/data/landmark.h>
#undef private
#include "relocalizer.h"
#include "reloc_trace.hpp"
#include <spdlog/spdlog.h>
#include "tracking_glue.hpp"
using namespace stella_vslam;
namespace reloc_trace {sv_pnp_trace_fn callback=nullptr;void*context=nullptr;
void (*pnp_input)(const eigen_alloc_vector<Vec3_t>&,const std::vector<int>&,const eigen_alloc_vector<Vec3_t>&,const std::vector<float>&)=nullptr;}
static void read(std::istream&in,void*p,size_t n){if(!in.read((char*)p,n))throw std::runtime_error("truncated map");}
template<class T>T value(std::istream&in){T x;read(in,&x,sizeof x);return x;}
static data::frame_observation observation(std::istream&in,const camera::perspective&cam){
 data::frame_observation o;unsigned n=value<unsigned>(in);o.undist_keypts_.resize(n);o.descriptors_=cv::Mat(n,32,CV_8UC1);
 for(unsigned i=0;i<n;i++){float x=value<float>(in),y=value<float>(in),angle=value<float>(in);int octave=value<int>(in);o.undist_keypts_[i]=cv::KeyPoint(x,y,1,angle,0,octave);read(in,o.descriptors_.ptr(i),32);o.bearings_.push_back(cam.convert_point_to_bearing({x,y}));}
 o.num_grid_cols_=64;o.num_grid_rows_=48;o.keypt_indices_in_cells_=data::assign_keypoints_to_grid(&cam,o.undist_keypts_,64,48);return o;
}
struct World {
 camera::perspective cam{"leaf",camera::setup_type_t::Monocular,camera::color_order_t::RGB,640,480,30,517.306408,516.469215,318.643040,255.313989,0.262383,-0.953104,-0.005358,0.002628,1.163314};
 feature::orb_params orb{"ORB",1.2f,8,20,7};
 std::unique_ptr<data::bow_vocabulary>vocab;std::unique_ptr<data::bow_database>db;
 std::map<unsigned,std::shared_ptr<data::keyframe>>keys;std::vector<std::shared_ptr<data::landmark>>lms;
 data::frame current;
 World(const char*path,const char*vpath){
  vocab.reset(data::bow_vocabulary_util::load(vpath));db.reset(new data::bow_database(vocab.get()));std::ifstream in(path,std::ios::binary);char magic[8];read(in,magic,8);if(memcmp(magic,"SVRELOC1",8))throw std::runtime_error("bad map");
  unsigned id=value<unsigned>(in);double ts=value<double>(in);int ref=value<int>(in);unsigned nk=value<unsigned>(in),nl=value<unsigned>(in);
  current=data::frame(id,ts,&cam,&orb,observation(in,cam),{});current.set_pose_cw(Mat44_t::Identity());current.invalidate_pose();current.compute_bow(vocab.get());
  struct Graph{unsigned id;int parent;std::vector<std::pair<unsigned,unsigned>>covis;std::vector<unsigned>children;};std::vector<Graph>graphs;
  for(unsigned k=0;k<nk;k++){
   unsigned id=value<unsigned>(in);double ts=value<double>(in);Mat44_t pose;read(in,pose.data(),128);auto obs=observation(in,cam);data::bow_vector bow;data::bow_feature_vector feat;data::bow_vocabulary_util::compute_bow(vocab.get(),obs.descriptors_,bow,feat);
   keys[id]=data::keyframe::make_keyframe(id,ts,pose,&cam,&orb,obs,bow,feat);db->add_keyframe(keys[id]);Graph g;g.id=id;
   unsigned nc=value<unsigned>(in);for(unsigned j=0;j<nc;j++){unsigned a=value<unsigned>(in),b=value<unsigned>(in);g.covis.emplace_back(a,b);}
   g.parent=value<int>(in);unsigned nchild=value<unsigned>(in);for(unsigned j=0;j<nchild;j++)g.children.push_back(value<unsigned>(in));graphs.push_back(g);
  }
  for(auto&g:graphs){auto& node=keys.at(g.id)->graph_node_;for(auto&e:g.covis)node->add_connection(keys.at(e.first),e.second);node->update_covisibility_orders();if(g.parent>=0)node->set_spanning_parent(keys.at(g.parent));else{auto root=keys.at(g.id);node->set_spanning_root(root);}for(auto id:g.children)node->add_spanning_child(keys.at(id));}
  for(unsigned i=0;i<nl;i++){
   unsigned id=value<unsigned>(in);Vec3_t p,normal;read(in,p.data(),24);read(in,normal.data(),24);float min=value<float>(in),max=value<float>(in);cv::Mat desc(1,32,CV_8UC1);read(in,desc.data,32);int ref=value<int>(in);unsigned no=value<unsigned>(in);
   auto lm=std::make_shared<data::landmark>(id,p,keys.at(ref));for(unsigned j=0;j<no;j++){unsigned k=value<unsigned>(in),idx=value<unsigned>(in);lm->add_observation(keys.at(k),idx);keys.at(k)->add_landmark(lm,idx);}
   lm->mean_normal_=normal;lm->min_valid_dist_=min;lm->max_valid_dist_=max;lm->has_valid_prediction_parameters_=true;lm->descriptor_=desc;lm->has_representative_descriptor_=true;lms.push_back(lm);
  }
  current.ref_keyfrm_=keys.at(ref);if(in.peek()!=EOF)throw std::runtime_error("trailing map bytes");
 }
};
static std::string prefix;static unsigned npnp=0;
static void pnp_input(const eigen_alloc_vector<Vec3_t>&b,const std::vector<int>&oct,const eigen_alloc_vector<Vec3_t>&p,const std::vector<float>&scales){
 std::ofstream f(prefix+".pnp"+std::to_string(npnp++)+".input",std::ios::binary);unsigned n=b.size(),levels=scales.size();f.write((char*)&n,4);f.write((char*)&levels,4);f.write((char*)scales.data(),4*levels);
 for(unsigned i=0;i<n;i++){f.write((char*)b[i].data(),24);f.write((char*)p[i].data(),24);f.write((char*)&oct[i],4);}if(!f)throw std::runtime_error("cannot save PnP input");
}
static void write_trace(void*u,const char*name,const double*p,unsigned n){FILE*f=(FILE*)u;unsigned len=strlen(name);if(fwrite(&len,4,1,f)!=1 || fwrite(name,1,len,f)!=len || fwrite(&n,4,1,f)!=1 || fwrite(p,8,n,f)!=n)throw std::runtime_error("cannot write trace");}
int main(int argc,char**argv){try{
 if(argc!=5)throw std::runtime_error("usage: map vocabulary output mode");spdlog::set_level(spdlog::level::off);World w(argv[1],argv[2]);prefix=argv[3];int mode=std::stoi(argv[4]);
 auto opt=std::make_shared<optimize::pose_optimizer_g2o>();unsigned min_valid=mode==3?10000:50;bool neighbors=mode!=4;
 reloc_reference::relocalizer rel(opt,.75,.9,.8,20,min_valid,true,neighbors);
 module::relocalizer original(opt,.75,.9,.8,20,min_valid,true,neighbors);
 auto before=w.current;if(mode==2 || mode==7)w.db->clear();FILE*f=fopen((prefix+".trace").c_str(),"wb");if(!f)throw std::runtime_error("cannot open trace");
 reloc_trace::callback=write_trace;reloc_trace::context=f;reloc_trace::pnp_input=pnp_input;
 std::vector<std::shared_ptr<data::keyframe>>candidate{w.current.ref_keyfrm_};
 TrackingBranch tracking{w.current,w.db.get(),w.vocab.get(),rel};
 bool ok=(mode==6 || mode==7)?tracking.run():(mode==1 || mode==5)?rel.reloc_by_candidates(w.current,candidate,mode==5):rel.relocalize(w.db.get(),w.current);
 reloc_trace::scalar("result",ok);reloc_trace::frame("result",w.current);if(mode==6 || mode==7){reloc_trace::scalar("tracking_id",tracking.last_reloc_frm_id_);reloc_trace::scalar("tracking_timestamp",tracking.last_reloc_frm_timestamp_);}fclose(f);reloc_trace::callback=nullptr;reloc_trace::pnp_input=nullptr;
 if(mode!=5){bool native=mode==1?original.reloc_by_candidates(before,candidate):original.relocalize(w.db.get(),before);if(native!=ok || before.pose_is_valid()!=w.current.pose_is_valid() || before.get_landmarks()!=w.current.get_landmarks())throw std::runtime_error("observer changed public result");if(before.pose_is_valid()){auto a=before.get_pose_cw(),b=w.current.get_pose_cw();if(memcmp(a.data(),b.data(),128))throw std::runtime_error("observer changed pose");}}
 std::cout<<w.current.id_<<" mode="<<mode<<" result="<<ok<<" pnp="<<npnp<<"\n";
 }catch(const std::exception&e){std::cerr<<e.what()<<"\n";return 2;}}
