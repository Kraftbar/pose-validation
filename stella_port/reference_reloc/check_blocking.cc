#include <Eigen/Core>
#include <cstdio>
int main(){std::ptrdiff_t a,b,c;Eigen::internal::manage_caching_sizes(Eigen::GetAction,&a,&b,&c);printf("cache %td %td %td mr %d nr %d\n",a,b,c,Eigen::internal::gebp_traits<double,double>::mr,Eigen::internal::gebp_traits<double,double>::nr);for(long rows:{100,200,400,500,600,800,1000,1600,2400}){long k=rows,m=12,n=12;Eigen::internal::computeProductBlockingSizes<double,double>(k,m,n);printf("rows %ld k %ld\n",rows,k);}}
