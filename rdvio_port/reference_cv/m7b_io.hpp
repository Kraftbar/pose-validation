// SPDX-License-Identifier: MIT
#pragma once
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <stdexcept>
inline FILE *m7b_out=nullptr;
inline void wr(const void *p,size_t n) {if(n&&fwrite(p,1,n,m7b_out)!=n)throw std::runtime_error("write");}
inline void rec(const char *s,const void *p,size_t n) {char name[32]={0};std::strncpy(name,s,31);uint32_t z=(uint32_t)n;wr(name,32);wr(&z,4);wr(p,n);}
inline void mat(const char *s,const cv::Mat &m) {cv::Mat c=m.isContinuous()?m:m.clone();rec(s,c.data,c.total()*c.elemSize());}
