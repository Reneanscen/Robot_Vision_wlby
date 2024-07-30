//
// Created by xu on 24-4-28.
//

#ifndef V4L2_STREAM_MIX_HELPER_H
#define V4L2_STREAM_MIX_HELPER_H

#include <cstdint>

enum ColorFormat {
    YUV_I420 = 0,
    YUV_NV21,
    YUV_NV12,
    BGR,
    RGB,
    BGRA,
    RGBA,
    ABGR,
    ARGB,
};

struct V4L2Buffer {
    uint32_t width, height;
    uint8_t *data[3];
    uint32_t stride[3];
    ColorFormat color;
};

V4L2Buffer create420Buffer(int width, int height);

#endif //V4L2_STREAM_MIX_HELPER_H
