// Minimal PCL stand-in (own code): only what OKVIS2-X uses for debug PLY dumps. No PCL needed.
#pragma once
#include <vector>
#include <cstdint>
namespace pcl {
struct PointXYZ { float x, y, z; };
template <class P> struct PointCloud {
  uint32_t width = 0, height = 0; bool is_dense = false;
  std::vector<P> points;
  void resize(size_t n) { points.resize(n); }
  typename std::vector<P>::iterator begin() { return points.begin(); }
  typename std::vector<P>::iterator end() { return points.end(); }
};
}
