// Real installed landmark::compute_descriptor plus an observation-only
// distance/median trace using upstream's descriptor-distance helper.
#include <stella_vslam/data/keyframe.h>
#include <stella_vslam/data/landmark.h>
#include <stella_vslam/data/map_database.h>
#include <stella_vslam/data/graph_node.h>
#include <stella_vslam/match/base.h>
#include <spdlog/spdlog.h>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
using namespace stella_vslam;
using namespace stella_vslam::data;

static std::string hex(const cv::Mat &d) {
    std::ostringstream s;s<<std::hex<<std::setfill('0');
    for(unsigned i=0;i<32;++i)s<<std::setw(2)<<unsigned(d.ptr<uint8_t>()[i]);
    return s.str();
}
int main(int argc,char **argv) {
    try {
        if(argc!=3)throw std::runtime_error("usage: dump_descriptor commands expected");
        std::ifstream in(argv[1]);std::ofstream out(argv[2]);
        if(!in || !out)throw std::runtime_error("cannot open files");
        spdlog::set_level(spdlog::level::off);
        unsigned qid,n;char tag;size_t cases=0;
        while(in>>tag){
            if(tag!='Q' || !(in>>qid>>n) || !n)throw std::runtime_error("bad case");
            frame_observation empty{};
            auto parent=keyframe::make_keyframe(0,0,Mat44_t::Identity(),nullptr,nullptr,empty,bow_vector{},bow_feature_vector{});
            std::map<unsigned,std::pair<std::shared_ptr<keyframe>,unsigned>> frames;
            std::vector<std::shared_ptr<keyframe>> insertion;
            map_database db(15);
            for(unsigned i=0;i<n;++i){
                unsigned id,erased;std::string s;
                if(!(in>>id>>erased>>s) || s.size()!=64 || erased>1 || frames.count(id))throw std::runtime_error("bad observation");
                frame_observation obs{};obs.undist_keypts_.resize(1);
                obs.descriptors_=cv::Mat(1,32,CV_8UC1);
                for(unsigned j=0;j<32;++j)obs.descriptors_.at<uint8_t>(0,j)=std::stoul(s.substr(2*j,2),nullptr,16);
                auto k=keyframe::make_keyframe(id,0,Mat44_t::Identity(),nullptr,nullptr,obs,bow_vector{},bow_feature_vector{});
                if(erased){
                    // Erase before adding the landmark observation, so the
                    // actual pending-erasure keyframe stays in that snapshot.
                    k->graph_node_->set_spanning_parent(parent);
                    k->prepare_for_erasing(&db,nullptr);
                    if(!k->will_be_erased())throw std::runtime_error("erasure setup failed");
                }
                frames[id]={k,i};insertion.push_back(k);
            }
            auto lm=std::make_shared<landmark>(0,Vec3_t::Zero(),insertion[0]);
            for(auto &k:insertion)lm->add_observation(k,0);
            if(lm->has_representative_descriptor())throw std::runtime_error("premature descriptor");
            lm->compute_descriptor(); // The real method is the output oracle.
            if(!lm->has_representative_descriptor())throw std::runtime_error("missing descriptor");
            auto actual=lm->get_descriptor();
            std::vector<cv::Mat> descriptors;
            std::vector<unsigned> ids,indices,medians(n,65535);
            for(auto &obs:lm->get_observations()){
                auto k=obs.first.lock();
                if(k->will_be_erased())continue;
                descriptors.push_back(k->frm_obs_.descriptors_.row(obs.second));
                ids.push_back(k->id_);indices.push_back(frames.at(k->id_).second);
            }
            unsigned best=256,best_i=0;
            for(unsigned i=0;i<descriptors.size();++i){
                std::vector<unsigned> row;
                for(auto &d:descriptors)row.push_back(match::compute_descriptor_distance(descriptors[i],d));
                std::sort(row.begin(),row.end());
                unsigned m=row[(row.size()-1)/2];medians[indices[i]]=m;
                if(m<best){best=m;best_i=i;}
            }
            if(hex(actual)!=hex(descriptors.at(best_i)))throw std::runtime_error("trace differs from real method");
            out<<"Q "<<qid<<' '<<indices.at(best_i)<<' '<<ids.at(best_i)<<' '<<best<<' '<<hex(actual)<<' '<<n;
            for(auto m:medians)out<<' '<<m;
            out<<'\n';++cases;
        }
        if(!cases || !out)throw std::runtime_error("empty/failed output");
        std::cout<<cases<<" cases\n";
    } catch(const std::exception &e){std::cerr<<e.what()<<'\n';return 2;}
}
