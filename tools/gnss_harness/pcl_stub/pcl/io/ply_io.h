#pragma once
#include <fstream>
#include <string>
#include "../point_types.h"
namespace pcl { namespace io {
template <class C> int savePLYFileASCII(const std::string& f, const C& c) {
  std::ofstream o(f);
  o << "ply\nformat ascii 1.0\nelement vertex " << c.points.size() << "\nproperty float x\nproperty float y\nproperty float z\nend_header\n";
  for (auto& p : c.points) o << p.x << ' ' << p.y << ' ' << p.z << '\n';
  return 0;
} } }
