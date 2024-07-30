//
// Created by xu on 24-4-28.
//

#ifndef V4L2_STREAM_V4L2_CAPTURE_H
#define V4L2_STREAM_V4L2_CAPTURE_H
#pragma once

#include "v4l2_stream/logger/logger.h"
#include <string>
#include <linux/videodev2.h>

struct resolution
{
    unsigned int width = 640;
    unsigned int height = 480;
    resolution() {};
    resolution(int w,int h): width(w),height(h) {};
};

typedef struct buffer
{
    void *data = nullptr;
    unsigned int length = 0;
} Buffer;

class V4L2Capture
{
public:
    explicit V4L2Capture(resolution s);
    V4L2Capture(resolution s,std::string deviceName);
    V4L2Capture();
    ~V4L2Capture();

    int init(int format);
    int camera_capture_start();
    void set_resolution(int width,int height);
    void set_device(std::string name);
    void set_frame_rate(int fps);
    resolution get_resolution() { return resolution_; };
    Buffer get_one_frame();
    void clear_one_frame();

private:
    resolution resolution_;
    std::string deviceName_ = "/dev/video0";
    int camera_fd_ = -1;
    int buffNum_ = 4;    // 申请的帧缓冲个数
    Buffer* buffers_ = nullptr;
    struct v4l2_buffer buf;
    int fps_ = 30;

private:
    int camera_open();
    int camera_ioctl(int request, void *arg);
    int camera_query_cap();
    int camera_set_video_fmt(int format);
    int camera_request_buffer();
    int camera_buffer_release(int i);
    int camera_set_fps();
};

void yuyv2yuv420(unsigned char *yuyv,unsigned char *yuv420,int width,int height);

#endif //V4L2_STREAM_V4L2_CAPTURE_H
