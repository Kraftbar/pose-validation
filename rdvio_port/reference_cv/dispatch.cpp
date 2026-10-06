// SPDX-License-Identifier: MIT
#include <opencv2/core.hpp>
#include <iostream>
int main() {
 cv::setNumThreads(1);
 std::cout << cv::getBuildInformation() << "\nFEATURES " << cv::getCPUFeaturesLine() << "\n";
 for(int i: {CV_CPU_SSE2,CV_CPU_SSE3,CV_CPU_SSSE3,CV_CPU_SSE4_1,CV_CPU_SSE4_2,CV_CPU_AVX,CV_CPU_AVX2,CV_CPU_FMA3,CV_CPU_AVX_512F})
  std::cout << cv::getHardwareFeatureName(i) << " " << cv::checkHardwareSupport(i) << "\n";
}
