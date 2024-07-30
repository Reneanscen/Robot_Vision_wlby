//
// Created by xu on 24-4-28.
//
//  https://blog.csdn.net/qq_41813395/article/details/114559840

#include <errno.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include "v4l2_capture.h"

V4L2Capture::V4L2Capture(resolution s) : resolution_(s){};

V4L2Capture::V4L2Capture(){};

V4L2Capture::V4L2Capture(resolution s, std::string deviceName)
        : resolution_(s), deviceName_(deviceName) {
    printf("it is %s \n", deviceName_.c_str());
};

V4L2Capture::~V4L2Capture() {
    camera_buffer_release(buffNum_);
    close(camera_fd_);
}

void V4L2Capture::set_resolution(int width, int height) {
    resolution_.width = width;
    resolution_.height = height;
}

void V4L2Capture::set_device(std::string name) { deviceName_ = std::move(name); }

void V4L2Capture::set_frame_rate(int fps) {
    if (fps < 0 || fps > 100) return;
    fps_ = fps;
}

int V4L2Capture::init(int format) {
    int ret = 0;
    ret = camera_open();
    if(ret){
        printf("[init] camera_open error!");
        return ret;
    }
    ret = camera_query_cap();
    if(ret){
        printf("[init] camera_query_cap error!");
        return ret;
    }
    ret = camera_set_video_fmt(format);
    if(ret){
        printf("[init] camera_set_video_fmt error!");
        return ret;
    }
    ret = camera_set_fps();
    if(ret){
        printf("[init] camera_set_fps error!");
        return ret;
    }

    ret = camera_request_buffer();
    if(ret){
        printf("[init] camera_request_buffer error!");
        return ret;
    }
    return ret;
}

int V4L2Capture::camera_capture_start() {
    int i = 0;
    struct v4l2_buffer buf;
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    memset(&buf, 0, sizeof(buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    for (i = 0; i < buffNum_; ++i) {
        buf.index = i;
        if (camera_ioctl(VIDIOC_QBUF, &buf) < 0) {
            printf("VIDIOC_QBUF error:%d %s\n", errno, strerror(errno));
            return -1;
        }
    }

    if (camera_ioctl(VIDIOC_STREAMON, &type) < 0) {
        printf("VIDIOC_STREAMON error:%d %s\n", errno, strerror(errno));
        return -1;
    }

    return 0;
}

int V4L2Capture::camera_set_fps() {
    v4l2_streamparm setfps;
    memset(&setfps, 0, sizeof(setfps));
    setfps.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    // //	SHOULD_BE_OK(ioctl(dev, VIDIOC_G_PARM, &setfps));
    camera_ioctl(VIDIOC_G_PARM, &setfps);
    setfps.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    setfps.parm.capture.timeperframe.numerator = 1;
    setfps.parm.capture.timeperframe.denominator = fps_;
    camera_ioctl(VIDIOC_G_PARM, &setfps);

    // SHOULD_BE_OK(v4l2_ioctl(dev, VIDIOC_S_PARM, &setfps))
    return camera_ioctl(VIDIOC_S_PARM, &setfps);
}

Buffer V4L2Capture::get_one_frame() {
    Buffer frame;
    memset(&buf, 0, sizeof(buf));
    // printf("the size is %d \n",sizeof(buf),buf.length );
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    memset(buffers_[buf.index].data, 0, buf.length);

    fd_set rset;
    FD_ZERO(&rset);
    FD_SET(camera_fd_, &rset);
    struct timeval tv;
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    int retsel = select(camera_fd_ + 1, &rset, NULL, NULL, &tv);
    if (retsel <= 0) {
        printf("%s:select failed,retsel=%d,dev=%s,errno=%d,errstr=%s\n", __func__,
               retsel, deviceName_.c_str(), errno, strerror(errno));
        return frame;
    }

    if (camera_ioctl(VIDIOC_DQBUF, &buf) < 0) {
        printf("err num is %d \n", errno);
        switch (errno) {
            case EAGAIN:
                printf("camera_read_frame EAGAIN\n");
                return frame;
                break;
            case EIO:
                /* 忽略 */
            default:
                return frame;
                break;
        }
    }

    // cam->deal_frame(cam->buffers[buf.index].start, buf.length);
    frame.data = buffers_[buf.index].data;
    frame.length = buf.length;
    return frame;
    // if(camera_ioctl(VIDIOC_QBUF, &buf) < 0)
    // {
    //     printf("camera_read_frame VIDIOC_QBUF error:%d %s\n", errno,
    //     strerror(errno)); return ERROR;
    // }

    // return 0;
}

void V4L2Capture::clear_one_frame() {
    if (camera_ioctl(VIDIOC_QBUF, &buf) < 0) {
        printf("camera_read_frame VIDIOC_QBUF error:%d %s\n", errno,
               strerror(errno));
    }
}

int V4L2Capture::camera_open() {
    struct stat devStat;

    memset(&devStat, 0, sizeof(devStat));
    if (stat(deviceName_.c_str(), &devStat) < 0) {
        printf("get device[%s] info failed: %d, %s\n", deviceName_.c_str(), errno,
               strerror(errno));
        return -1;
    }

    if (!S_ISCHR(devStat.st_mode)) {
        printf("%s is not char device!\n", deviceName_.c_str());
        return -1;
    }

    // O_NONBLOCK,需阻塞方式打开，使用非阻塞会在取缓冲帧时一直返回EAGAIN
    camera_fd_ = open(deviceName_.c_str(), O_RDWR, 0);
    if (camera_fd_ < 0) {
        printf("cannot open %s:%d, %s\n", deviceName_.c_str(), errno,
               strerror(errno));
        return -1;
    }

    return 0;
}

int V4L2Capture::camera_ioctl(int request, void *arg) {
    int ret = -1;
    do {
        ret = ioctl(camera_fd_, request, arg);
    } while (ret < 0 && EINTR == errno);

    return ret;
}

int V4L2Capture::camera_query_cap() {
    struct v4l2_capability cap;

    memset(&cap, 0, sizeof(cap));
    if (camera_ioctl(VIDIOC_QUERYCAP, &cap) < 0) {
        printf("VIDIOC_QUERYCAP error: %d %s\n", errno, strerror(errno));
        return -1;
    }

    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
        printf("device does not support capture!!!\n");
        return -1;
    }

    if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
        printf("device does not support streaming!!!\n");
        return -1;
    }

    printf("VIDIOC_QUERYCAP\n");
    printf("driver:%s\n", cap.driver);
    printf("card:%s\n", cap.card);
    printf("bus info:%s\n", cap.bus_info);
    printf("version:%u\n", cap.version);
    printf("capabilities:%x\n", cap.capabilities);

    printf(
            "【**********************所有支持的格式：****************************"
            "】\n");

    struct v4l2_fmtdesc dis_fmtdesc;

    dis_fmtdesc.index = 0;

    dis_fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    printf("Support format:\n");

    while (ioctl(camera_fd_, VIDIOC_ENUM_FMT, &dis_fmtdesc) != -1)

    {
        printf("\t%d.%s  %d\n", dis_fmtdesc.index + 1, dis_fmtdesc.description,
               dis_fmtdesc.type);

        dis_fmtdesc.index++;
    }

    printf("\n");

    return 0;
}

int V4L2Capture::camera_set_video_fmt(int format) {
    struct v4l2_format fmt;

    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = resolution_.width;
    fmt.fmt.pix.height = resolution_.height;

    if (format == 0) {
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
        log__save("pixelformat", kLogLevel_Info, kLogTarget_Stdout | kLogTarget_Filesystem, "V4L2_PIX_FMT_YUYV");
    }
    if (format == 1) {
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
        log__save("pixelformat", kLogLevel_Info, kLogTarget_Stdout | kLogTarget_Filesystem, "V4L2_PIX_FMT_MJPEG");
    }
    if (format == 2) {
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24;
        log__save("pixelformat", kLogLevel_Info, kLogTarget_Stdout | kLogTarget_Filesystem, "V4L2_PIX_FMT_RGB24");
    }

    if (camera_ioctl(VIDIOC_S_FMT, &fmt) < 0) {
        printf("VIDIOC_S_FMT error:%d %s\n", errno, strerror(errno));
        return -1;
    }

    // VIDIOC_S_FMT 有可能会改变分辨率,会限制为最大分辨率
    printf("VIDIOC_S_FMT:%d x %d   %d\n", fmt.fmt.pix.width, fmt.fmt.pix.height, fmt.fmt.pix.pixelformat);
    resolution_.width = fmt.fmt.pix.width;
    resolution_.height = fmt.fmt.pix.height;

    return 0;
}

int V4L2Capture::camera_request_buffer() {
    struct v4l2_requestbuffers req;
    struct v4l2_buffer v4l2Buf;
    int i = 0;

    memset(&req, 0, sizeof(req));
    req.count = 4;  // 内核空间内存，申请4个帧缓冲空间
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;  // 使用mmap

    if (camera_ioctl(VIDIOC_REQBUFS, &req) < 0) {
        printf("VIDIOC_REQBUFS error:%d %s\n", errno, strerror(errno));
        return -1;
    } else {
        printf("VIDIOC_REQBUFS:count = %d\n", req.count);
        buffNum_ = req.count;
    }

    memset(&v4l2Buf, 0, sizeof(v4l2Buf));
    v4l2Buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    v4l2Buf.memory = V4L2_MEMORY_MMAP;

    buffers_ = (Buffer *)calloc(req.count, sizeof(*(buffers_)));
    if (!buffers_) {
        printf("calloc failed,Out of memory\n");
        return -1;
    }

    for (i = 0; i < req.count; ++i) {
        v4l2Buf.index = i;
        if (camera_ioctl(VIDIOC_QUERYBUF, &v4l2Buf) < 0) {
            printf("VIDIOC_QUERYBUF [%d] error: %d %s\n", i, errno, strerror(errno));
            return -1;
        }
        buffers_[i].length = v4l2Buf.length;
        buffers_[i].data = mmap(NULL, v4l2Buf.length, PROT_READ | PROT_WRITE,
                                MAP_SHARED, camera_fd_, v4l2Buf.m.offset);
        // printf("it is %d \n",v4l2Buf.length);
        if (MAP_FAILED == buffers_[i].data) {
            printf("buffer[%d] mmap fafiled!", i);
            if (i > 0) {
                camera_buffer_release(i);
            }
            return -1;
        }
    }

    return 0;
}

int V4L2Capture::camera_buffer_release(int num) {
    int i = 0;

    if (NULL == buffers_) {
        return 0;
    }

    for (i = 0; i < num; ++i) {
        munmap(buffers_[i].data, buffers_[i].length);
    }

    free(buffers_);
    buffers_ = NULL;
    return 0;
}

void yuyv2yuv420(unsigned char *yuyv, unsigned char *yuv420, int width, int height) {
    int pixs = width * height;
    unsigned char *y = yuv420;
    unsigned char *u = yuv420 + pixs;
    unsigned char *v = yuv420 + pixs + (pixs >> 2);
    unsigned char *start = yuyv;

    /*处理Y分量*/
    for (int j = 0; j < pixs * 2; j = j + 2) {
        *y++ = *(start + j);
    }

    /**处理UV分量**/
    start = yuyv;
    for (int h = 0; h < height; h += 2)  // 隔行, 我选择保留偶数行
    {
        for (int w = h * width * 2 + 1; w < width * 2 * (h + 1);
             w += 4)  // YUYV单行中每四个字节含有一对UV分量
        {
            *u++ = *(start + w);
            *v++ = *(start + w + 2);
        }
    }
}
