#include "check/check.h"
#include "common/fixed_ratio_sample.h"

#include "logger/my_logger.h"

namespace wlby{

FixedRatioSampler::FixedRatioSampler(const double ratio) : ratio_(ratio) {
  CHECK_GE(ratio, 0.);
  if(FLOAT_IS_ZERO(ratio)) {
    log__save("CHECK_Error", kLogLevel_Error, kLogTarget_Stdout | kLogTarget_Filesystem,"FixedRatioSampler: ratio is zero! ");
  }
  CHECK_LE(ratio, 1.);
}

FixedRatioSampler::~FixedRatioSampler() {}

bool FixedRatioSampler::Pulse() {
  ++num_pulses_;
  if (static_cast<double>(num_samples_) / num_pulses_ < ratio_) {
    ++num_samples_;
    return true;
  }
  return false;
}



}