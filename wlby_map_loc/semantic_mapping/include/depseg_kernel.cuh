#ifndef _DEPSEG_KERNEL_CUH_
#define _DEPSEG_KERNEL_CUH_

#include <string>
#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/rgbd.hpp>


#include <cuda_runtime.h>
#include <device_launch_parameters.h>


__global__ void ComputeNormalKernel(int Height,int Width,cv::cuda::PtrStep<u_char> depth_map,cv::cuda::PtrStep<u_char> normals);

__global__ void ComputeMinConvexityMapKernel(int Height,int Width,float *padded_mat, float *padded_mat_normal,cv::cuda::PtrStep<u_char> depth_map, cv::cuda::PtrStep<u_char> normals, cv::cuda::PtrStep<u_char> minConvexityMap,float mask_threshold, float threshold);

__device__ float& getInputRefFromLIJ(float* input, int l,int i,int j,int rows,int cols);

__device__ void getPaddingMatByCenterPoint_cuda(float *padded_mat,int rows,int cols,int i,int j,float* output,int size);

__device__ void conv_gpu(float* a1, float* a2, float* output, int size);

__device__ float getSum_gpu(float* a,int size);

extern "C" {
    void ComputeNormal_gpu(int Height,int Width,cv::cuda::PtrStep<u_char> depth_map,cv::cuda::PtrStep<u_char> normals);
    void ComputeMinConvexityMap_gpu(int Height,int Width,float *padded_mat, float *padded_mat_normal ,cv::cuda::PtrStep<u_char> depth_map,cv::cuda::PtrStep<u_char> normals,cv::cuda::PtrStep<u_char> minConvexityMap,float mask_threshold, float threshold);
    void ComputeOutPut_gpu(int Height,int Width,cv::cuda::PtrStep<u_char> output_labels,cv::cuda::PtrStep<u_char> output,cv::cuda::PtrStep<u_char> edge_map_8u,cv::cuda::PtrStep<u_char> depth_map);
}
#endif
