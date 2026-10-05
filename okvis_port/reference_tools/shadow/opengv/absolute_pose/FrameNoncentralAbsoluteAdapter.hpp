// Test shadow of okvis_frontend/include/opengv/absolute_pose/FrameNoncentralAbsoluteAdapter.hpp (BSD-3-Clause, OKVIS2): the
// same class name and accessors over plain vectors, so that the UNMODIFIED OKVIS2 FrameAbsolutePoseSacProblem header and the real
// OpenGV library can be driven with random data without the OKVIS2 estimator (see okvis_port/reference_tools/okvis_opengv_test.cc).
#ifndef OK_SHADOW_FRAMENONCENTRALABSOLUTEADAPTER_HPP_
#define OK_SHADOW_FRAMENONCENTRALABSOLUTEADAPTER_HPP_
#include <stdlib.h>
#include <vector>
#include <opengv/types.hpp>
#include <opengv/absolute_pose/AbsoluteAdapterBase.hpp>
namespace opengv {
namespace absolute_pose {
class FrameNoncentralAbsoluteAdapter : public AbsoluteAdapterBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  virtual ~FrameNoncentralAbsoluteAdapter() override = default;
  virtual opengv::bearingVector_t getBearingVector(size_t index) const override final { return bearingVectors_[index]; }
  virtual opengv::translation_t getCamOffset(size_t index) const override final { return camOffsets_[index]; }
  virtual opengv::rotation_t getCamRotation(size_t index) const override final { return camRotations_[index]; }
  virtual opengv::point_t getPoint(size_t index) const override final { return points_[index]; }
  virtual size_t getNumberCorrespondences() const override final { return points_.size(); }
  virtual double getWeight(size_t) const override final { return 1.0; }
  double getSigmaAngle(size_t index) { return sigmaAngles_[index]; }
  // test data (per correspondence)
  opengv::bearingVectors_t bearingVectors_;
  opengv::points_t points_;
  opengv::translations_t camOffsets_;
  opengv::rotations_t camRotations_;
  std::vector<double> sigmaAngles_;
};
}
}
#endif
