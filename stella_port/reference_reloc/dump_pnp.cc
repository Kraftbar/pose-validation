#include <stella_vslam/solve/pnp_solver.h>
#include "pnp_solver.h"
#include "pnp_trace.hpp"
#include <fstream>
#include <iostream>
#include <cstring>
using namespace stella_vslam;
namespace reloc_trace {sv_pnp_trace_fn callback=nullptr;void*context=nullptr;}
static void read(std::istream&f,void*p,size_t n){if(!f.read((char*)p,n))throw std::runtime_error("truncated input");}
static void write(void*u,const char*name,const double*p,unsigned n){FILE*f=(FILE*)u;unsigned len=strlen(name);if(fwrite(&len,4,1,f)!=1||fwrite(name,1,len,f)!=len||fwrite(&n,4,1,f)!=1||fwrite(p,8,n,f)!=n)throw std::runtime_error("write failed");}
int main(int argc,char**argv){try{
 if(argc!=4)throw std::runtime_error("usage: input trace recompute");std::ifstream f(argv[1],std::ios::binary);unsigned n,levels;read(f,&n,4);read(f,&levels,4);if(n>100000||levels>16)throw std::runtime_error("invalid size");std::vector<float>scales(levels);read(f,scales.data(),4*levels);
 eigen_alloc_vector<Vec3_t>b(n),p(n);std::vector<int>oct(n);for(unsigned i=0;i<n;i++){read(f,b[i].data(),24);read(f,p[i].data(),24);read(f,&oct[i],4);}if(f.peek()!=EOF)throw std::runtime_error("extra input");
 FILE*out=fopen(argv[2],"wb");if(!out)throw std::runtime_error("cannot write");reloc_trace::callback=write;reloc_trace::context=out;bool recompute=std::stoi(argv[3]);
 reloc_reference::pnp_solver solver(b,oct,p,scales,10,true,10);solver.find_via_ransac(30,recompute);reloc_trace::scalar("final_valid",solver.solution_is_valid());std::vector<double>mask;for(bool v:solver.get_inlier_flags())mask.push_back(v);reloc_trace::emit("final_mask",mask.data(),mask.size());
 if(solver.solution_is_valid()){reloc_trace::mat("final_rotation",solver.get_best_rotation());reloc_trace::mat("final_translation",solver.get_best_translation());}fclose(out);reloc_trace::callback=nullptr;
 bool degenerate=n>=4;for(unsigned i=1;i<n;i++)degenerate=degenerate && p[i]==p[0];
 if(degenerate){std::cout<<"identical-point fixture: guarded reference only (upstream reads uninitialized SVD vectors)\n";return 0;}
 solve::pnp_solver native(b,oct,p,scales,10,true,10);native.find_via_ransac(30,recompute);
 if(native.solution_is_valid()!=solver.solution_is_valid()||native.get_inlier_flags()!=solver.get_inlier_flags())throw std::runtime_error("guard changed public result");if(native.solution_is_valid()){auto a=native.get_best_cam_pose(),b=solver.get_best_cam_pose();if(memcmp(a.data(),b.data(),128))throw std::runtime_error("guard changed pose");}
 std::cout<<n<<" correspondences, recompute="<<recompute<<", valid="<<solver.solution_is_valid()<<"\n";
 }catch(const std::exception&e){std::cerr<<e.what()<<"\n";return 2;}}
