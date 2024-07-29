#ifndef _FIXED_RATIO_SAMPLER_
#define _FIXED_RATIO_SAMPLER_

/*******************************************************************************
* @brief 采样频率类
* @author jazzey
* @date 2023-8-9
********************************************************************************/

#include <string>
#include "common/common_type_define.h"

namespace wlby {

    class FixedRatioSampler {
    public:
        explicit FixedRatioSampler(double ratio);

        ~FixedRatioSampler();

        FixedRatioSampler(const FixedRatioSampler &) = delete;

        FixedRatioSampler &operator=(const FixedRatioSampler &) = delete;

        bool Pulse();

    private:
        const double ratio_;

        int64 num_pulses_ = 0;
        int64 num_samples_ = 0;
    };


}

#endif