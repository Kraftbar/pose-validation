// Test shadow of okvis_frontend/include/opengv/relative_pose/FrameRelativeAdapter.hpp (BSD-3-Clause, OKVIS2): the same class name,
// accessors and identity camera offsets / rotations over plain vectors (see FrameNoncentralAbsoluteAdapter.hpp).
#ifndef OK_SHADOW_FRAMERELATIVEADAPTER_HPP_
#define OK_SHADOW_FRAMERELATIVEADAPTER_HPP_
#include <stdlib.h>
#include <vector>
#include <opengv/types.hpp>
#include <opengv/relative_pose/RelativeAdapterBase.hpp>
namespace opengv {
namespace relative_pose {
class FrameRelativeAdapter : public RelativeAdapterBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  virtual ~FrameRelativeAdapter() {}
  virtual opengv::bearingVector_t getBearingVector1(size_t index) const { return bearingVectors1_[index]; }
  virtual opengv::bearingVector_t getBearingVector2(size_t index) const { return bearingVectors2_[index]; }
  virtual opengv::translation_t getCamOffset1(size_t) const { return Eigen::Vector3d::Zero(); }
  virtual opengv::rotation_t getCamRotation1(size_t) const { return Eigen::Matrix3d::Identity(); }
  virtual opengv::translation_t getCamOffset2(size_t) const { return Eigen::Vector3d::Zero(); }
  virtual opengv::rotation_t getCamRotation2(size_t) const { return Eigen::Matrix3d::Identity(); }
  virtual size_t getNumberCorrespondences() const { return bearingVectors1_.size(); }
  virtual double getWeight(size_t) const { return 1.0; }
  double getSigmaAngle1(size_t index) { return sigmaAngles1_[index]; }
  double getSigmaAngle2(size_t index) { return sigmaAngles2_[index]; }
  opengv::bearingVectors_t bearingVectors1_, bearingVectors2_;
  std::vector<double> sigmaAngles1_, sigmaAngles2_;
};
}
}
#endif
