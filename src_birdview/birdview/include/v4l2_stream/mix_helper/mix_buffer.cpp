//
// Created by xu on 24-4-28.
//
#include "mix_helper.h"

// 创建I420格式的yuvBuffer
V4L2Buffer create420Buffer(int width, int height) {
    V4L2Buffer src{};
    src.width = width;
    src.height = height;
    auto *yuvdata = new uint8_t[width * height * 3 / 2];
    src.stride[0] = src.width;
    src.stride[1] = src.width >> 1;
    src.stride[2] = src.width >> 1;
    uint32_t dst_y_size = src.width * src.height;
    uint32_t dst_u_size = src.stride[1] * (src.height >> 1);
    src.data[0] = yuvdata;
    src.data[1] = src.data[0] + dst_y_size;
    src.data[2] = src.data[1] + dst_u_size;
    src.color = ColorFormat::YUV_I420;
    return src;
}
