#include "depseg_kernel.cuh"

constexpr float float_nan = std::numeric_limits<float>::quiet_NaN();

// GPU计算图像法向量核函数
__global__ void ComputeNormalKernel(int Height, int Width, cv::cuda::PtrStep<u_char> depth_map, cv::cuda::PtrStep<u_char> normals)
{

	const int x = blockIdx.x * blockDim.x + threadIdx.x;
	const int y = blockIdx.y * blockDim.y + threadIdx.y;

	float *Img_y = (float *)(depth_map + y * depth_map.step);

	float *dest_y = (float *)(normals + y * normals.step);

	if (x < Width && y < Height)
	{

		float b = Img_y[3 * x];
		float g = Img_y[3 * x + 1];
		float r = Img_y[3 * x + 2];

		//==========nan点剔除
		Cv64suf ieee754;
		ieee754.f = b;
		bool flag_b = ((unsigned)(ieee754.u >> 32) & 0x7fffffff) + ((unsigned)ieee754.u != 0) > 0x7ff00000;

		ieee754.f = g;
		bool flag_g = ((unsigned)(ieee754.u >> 32) & 0x7fffffff) + ((unsigned)ieee754.u != 0) > 0x7ff00000;

		ieee754.f = r;
		bool flag_r = ((unsigned)(ieee754.u >> 32) & 0x7fffffff) + ((unsigned)ieee754.u != 0) > 0x7ff00000;

		if (flag_b || flag_g || flag_r || r == 0.0)
		{
			dest_y[3 * x] = float_nan;
			dest_y[3 * x + 1] = float_nan;
			dest_y[3 * x + 2] = float_nan;
		}
		else
		{

			//===========================找出邻居：===============
			const float max_distance = 0.05 * r;
			float mean[3] = {0.0f, 0.0f, 0.0f};
			const int window_size = 13;
			float neighborhood[3][window_size * window_size];
			u_int neighborhood_size = 0;

			// 当前的像素值 b g r
			for (u_int y_idx = 0; y_idx < window_size; ++y_idx)
			{
				const int y_filter_idx = y + y_idx - window_size / 2u;
				if (y_filter_idx < 0 || y_filter_idx >= Height)
				{
					continue;
				}
				for (u_int x_idx = 0; x_idx < window_size; ++x_idx)
				{
					const int x_filter_idx = x + x_idx - window_size / 2u;
					if (x_filter_idx < 0 || x_filter_idx >= Width)
					{
						continue;
					}

					// 根据坐标 x_filter_idx ， y_filter_idx 获取点坐标(filter_Img_b filter_Img_g filter_Img_r)
					float *filter_Img_Y = (float *)(depth_map + y_filter_idx * depth_map.step);
					float filter_point[3];
					float filter_Img_b = filter_Img_Y[3 * x_filter_idx];
					float filter_Img_g = filter_Img_Y[3 * x_filter_idx + 1];
					float filter_Img_r = filter_Img_Y[3 * x_filter_idx + 2];
					filter_point[0] = filter_Img_b;
					filter_point[1] = filter_Img_g;
					filter_point[2] = filter_Img_r;

					// Compute Euclidean distance between filter_point and mid_point.
					const float difference_b = b - filter_Img_b;
					const float difference_g = g - filter_Img_g;
					const float difference_r = r - filter_Img_r;

					const float euclidean_dist = sqrtf(difference_b * difference_b + difference_g * difference_g + difference_r * difference_r);

					if (euclidean_dist < max_distance)
					{
						// Add the filter_point to neighborhood set.
						for (u_int coordinate = 0; coordinate < 3; ++coordinate)
						{
							neighborhood[coordinate][neighborhood_size] = filter_point[coordinate];
						}
						++neighborhood_size;
						mean[0] += filter_point[0];
						mean[1] += filter_point[1];
						mean[2] += filter_point[2];
					}
				}
			}

			mean[0] /= static_cast<float>(neighborhood_size);
			mean[1] /= static_cast<float>(neighborhood_size);
			mean[2] /= static_cast<float>(neighborhood_size);

			//===========================当邻居数量 > 1  ==========

			if (neighborhood_size > 1)
			{

				float covariance[3][3] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
				// 当邻居点 > 1计算协方差矩阵
				for (u_int i = 0; i < neighborhood_size; ++i)
				{
					float point[3];
					point[0] = neighborhood[0][i] - mean[0];
					point[1] = neighborhood[1][i] - mean[1];
					point[2] = neighborhood[2][i] - mean[2];

					covariance[0][0] += point[0] * point[0];
					covariance[0][1] += point[0] * point[1];
					covariance[0][2] += point[0] * point[2];
					covariance[1][1] += point[1] * point[1];
					covariance[1][2] += point[1] * point[2];
					covariance[2][2] += point[2] * point[2];
				}
				// Assign the symmetric elements of the covariance matrix.
				covariance[1][0] = covariance[0][1];
				covariance[2][0] = covariance[0][2];
				covariance[2][1] = covariance[1][2];

				//======================求特征向量===========================
				float eigenVectors[9];
				bool if_success = false;
				float eigenValue[3];
				{
					float val[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

					int sk = 0;
					for (int s = 0; s < 3; s++)
					{
						for (int k = 0; k < 3; k++)
						{
							val[sk] = covariance[s][k];
							++sk;
						}
					}

					int m_nNumColumns = 3;
					double eps = 0.000001;
					int nMaxIt = 60;
					int i, j, p, q, u, w, t, s, l;
					double fm, cn, sn, omega, lx, ly, d;

					l = 1;
					for (i = 0; i <= m_nNumColumns - 1; i++)
					{
						eigenVectors[i * m_nNumColumns + i] = 1.0;
						for (j = 0; j <= m_nNumColumns - 1; j++)
						{
							if (i != j)
								eigenVectors[i * m_nNumColumns + j] = 0.0;
						}
					}

					while (true)
					{
						fm = 0.0;
						for (i = 1; i <= m_nNumColumns - 1; i++)
						{
							for (j = 0; j <= i - 1; j++)
							{
								d = fabs(val[i * m_nNumColumns + j]);
								if ((i != j) && (d > fm))
								{
									fm = d;
									p = i;
									q = j;
								}
							}
						}

						if (fm < eps)
						{
							if_success = true;
							for (i = 0; i < m_nNumColumns; ++i)
								eigenValue[i] = val[i + i * m_nNumColumns];
							break;
						}

						if (l > nMaxIt)
						{
							if_success = false;
							break;
						}

						l = l + 1;
						u = p * m_nNumColumns + q;
						w = p * m_nNumColumns + p;
						t = q * m_nNumColumns + p;
						s = q * m_nNumColumns + q;
						lx = -val[u];
						ly = (val[s] - val[w]) / 2.0;
						omega = lx / sqrt(lx * lx + ly * ly);

						if (ly < 0.0)
							omega = -omega;

						sn = 1.0 + sqrt(1.0 - omega * omega);
						sn = omega / sqrt(2.0 * sn);
						cn = sqrt(1.0 - sn * sn);
						fm = val[w];
						val[w] = fm * cn * cn + val[s] * sn * sn + val[u] * omega;
						val[s] = fm * sn * sn + val[s] * cn * cn - val[u] * omega;
						val[u] = 0.0;
						val[t] = 0.0;

						for (j = 0; j <= m_nNumColumns - 1; j++)
						{
							if ((j != p) && (j != q))
							{
								u = p * m_nNumColumns + j;
								w = q * m_nNumColumns + j;
								fm = val[u];
								val[u] = fm * cn + val[w] * sn;
								val[w] = -fm * sn + val[w] * cn;
							}
						}

						for (i = 0; i <= m_nNumColumns - 1; i++)
						{
							if ((i != p) && (i != q))
							{
								u = i * m_nNumColumns + p;
								w = i * m_nNumColumns + q;
								fm = val[u];
								val[u] = fm * cn + val[w] * sn;
								val[w] = -fm * sn + val[w] * cn;
							}
						}

						for (i = 0; i <= m_nNumColumns - 1; i++)
						{
							u = i * m_nNumColumns + p;
							w = i * m_nNumColumns + q;
							fm = eigenVectors[u];
							eigenVectors[u] = fm * cn + eigenVectors[w] * sn;
							eigenVectors[w] = -fm * sn + eigenVectors[w] * cn;
						}
					}

					for (i = 0; i < m_nNumColumns; ++i)
						eigenValue[i] = val[i + i * m_nNumColumns];
				}

				if (if_success)
				{

					float eigenVectors_real[3];
					if (eigenValue[0] < eigenValue[1] && eigenValue[0] < eigenValue[2])
					{

						eigenVectors_real[0] = eigenVectors[0];
						eigenVectors_real[1] = eigenVectors[3];
						eigenVectors_real[2] = eigenVectors[6];
					}
					else if (eigenValue[1] < eigenValue[0] && eigenValue[1] < eigenValue[2])
					{

						eigenVectors_real[0] = eigenVectors[1];
						eigenVectors_real[1] = eigenVectors[4];
						eigenVectors_real[2] = eigenVectors[7];
					}
					else
					{

						eigenVectors_real[0] = eigenVectors[2];
						eigenVectors_real[1] = eigenVectors[5];
						eigenVectors_real[2] = eigenVectors[8];
					}

					dest_y[3 * x] = eigenVectors_real[0];
					dest_y[3 * x + 1] = eigenVectors_real[1];
					dest_y[3 * x + 2] = eigenVectors_real[2];

					if (dest_y[3 * x + 2] > 0.0f)
					{
						dest_y[3 * x] = -eigenVectors_real[0];
						dest_y[3 * x + 1] = -eigenVectors_real[1];
						dest_y[3 * x + 2] = -eigenVectors_real[2];
					}
				}
				else
				{
					dest_y[3 * x] = float_nan;
					dest_y[3 * x + 1] = float_nan;
					dest_y[3 * x + 2] = float_nan;
				}

				//=========================================================
			}
			else
			{
				dest_y[3 * x] = float_nan;
				dest_y[3 * x + 1] = float_nan;
				dest_y[3 * x + 2] = float_nan;
			}
		}
	}
}

__device__ float& getInputRefFromLIJ(float* input, int l,int i,int j,int rows,int cols){
  	return input[l*rows*cols+i*cols+j];
}

__device__ void getPaddingMatByCenterPoint_cuda(float *padded_mat,int rows,int cols,int i,int j,float* output,int size){
    
	for(int k = 0; k < 3; k++)
	{
		for (int s = 0; s < size; s++)
		{
			for (int t = 0; t < size; t++)
			{
				*(output + k*size*size + s * size + t) = *(padded_mat + k*rows*cols + cols * (s + i) + t + j);
			}
		}
	}
}

__device__ void conv_gpu(float* a1, float* a2, float* output, int size){

		for (int s = 0; s < size; s++)
		{
			for (int t = 0; t < size; t++)
			{
				*(output + s * size + t) = (*(a1 + s * size + t)) * (*(a2 + s * size + t));
			}
		}
}

__device__ float getSum_gpu(float* a,int size){
  
  float sum = 0.0f;
  for (int s = 0; s < size; s++)
  {
    for (int t = 0; t < size; t++)
    {
      sum += *(a + s*size + t);
    }
  }
  return sum;
}

__global__ void ComputeMinConvexityMapKernel(int Height,int Width,float *padded_mat, float *padded_mat_normal, cv::cuda::PtrStep<u_char> depth_map, cv::cuda::PtrStep<u_char> normals, cv::cuda::PtrStep<u_char> minConvexityMap,float mask_threshold, float threshold){
	const int x = blockIdx.x * blockDim.x + threadIdx.x;
	const int y = blockIdx.y * blockDim.y + threadIdx.y;

	const int kernel_size = 5;
	int n_kernels = kernel_size * kernel_size - 1;

	int padding_rows = Height + kernel_size - 1;
  	int padding_cols = Width + kernel_size - 1;

	float* depth_map_ptr = (float *)(depth_map + y*depth_map.step);
	float* normals_ptr = (float *)(normals + y*normals.step);
	float* minConvexityMap_ptr = (float *)(minConvexityMap + y*minConvexityMap.step);
	
	if(x < Width && y < Height){


		for (int i = 0; i < kernel_size * kernel_size; i += 1) {
			
			if (i == n_kernels / 2) {
      			continue;
    		}
			
			float difference_kernel[kernel_size][kernel_size] = { 0.0f };
			int kernel_row = i / kernel_size;
			int kernel_col = i % kernel_size;
			difference_kernel[kernel_row][kernel_col] = 1.0f;
			difference_kernel[2][2] = -1.0f;

			float paddedmat_by_center[3][kernel_size][kernel_size] = { 0.0f };
			float conv_result[3][kernel_size][kernel_size] = { 0.0f };
	
			float sum[3] = {0.0f};
			getPaddingMatByCenterPoint_cuda(padded_mat,padding_rows,padding_cols,y,x,paddedmat_by_center[0][0],kernel_size);


			float vector_projection = 0.0f;
			for(int k = 0; k < 3; k ++){
				conv_gpu(difference_kernel[0],paddedmat_by_center[k][0],conv_result[k][0],kernel_size);
				sum[k] = getSum_gpu(conv_result[k][0],kernel_size);
				
				//实现mul函数
				float difference_times_normals[3];
				difference_times_normals[k] = -normals_ptr[3 * x + k] * sum[k];
				vector_projection += difference_times_normals[k];
				
				
			}

			float convexity_mask = 0.0f;
			float concavity_mask = 1.0f;
			if(vector_projection > mask_threshold){
				convexity_mask = 1.0f;
			}
			if(vector_projection > mask_threshold){
				concavity_mask = 0.0f;
			}

			float normal_kernel[kernel_size][kernel_size]={0.0f};
			kernel_row = i / kernel_size;
			kernel_col = i % kernel_size;
			normal_kernel[kernel_row][kernel_col] = 1.0f;

			float paddedmat_by_center_normal[3][kernel_size][kernel_size] = { 0.0f };
			float conv_result_normal[3][kernel_size][kernel_size] = { 0.0f };

			float sum2[3] = {0.0f};
			getPaddingMatByCenterPoint_cuda(padded_mat_normal,padding_rows,padding_cols,y,x,paddedmat_by_center_normal[0][0],kernel_size);


			float normal_vector_projection = 0.0f;
			for (int k = 0; k < 3; k++)
			{
				conv_gpu(normal_kernel[0], paddedmat_by_center_normal[k][0], conv_result_normal[k][0], kernel_size);
				sum2[k] = getSum_gpu(conv_result_normal[k][0], kernel_size);

				if(sum2[k] != sum2[k]){
					sum2[k] = normals_ptr[3 * x + k];
				}

				float normal_times_filtered_normal[3]={0.0f};
				normal_times_filtered_normal[k] = normals_ptr[3 * x + k] * sum2[k];

				if(normal_times_filtered_normal[k] != normal_times_filtered_normal[k]){
					normal_times_filtered_normal[k] = sum2[k];
				}

				normal_vector_projection += normal_times_filtered_normal[k];
			}

			normal_vector_projection *= concavity_mask;
			
			float convexity_map = normal_vector_projection + convexity_mask;

			minConvexityMap_ptr[x] = minConvexityMap_ptr[x] < convexity_map ? minConvexityMap_ptr[x] : convexity_map;

			minConvexityMap_ptr[x] = minConvexityMap_ptr[x] > threshold ? 1.0f : 0.0f;

		}

	}

}

__global__ void ComputeOutPut(int Height,int Width,cv::cuda::PtrStep<u_char> output_labels,cv::cuda::PtrStep<u_char> output,cv::cuda::PtrStep<u_char> edge_map_8u,cv::cuda::PtrStep<u_char> depth_map){

	const int x = blockIdx.x * blockDim.x + threadIdx.x;
	const int y = blockIdx.y * blockDim.y + threadIdx.y;

	int* output_labels_ptr = (int *)(output_labels + y*output_labels.step);
	int* output_ptr = (int *)(output + y*output.step);
	uchar* edge_map_8u_ptr = (uchar *)(edge_map_8u + y*edge_map_8u.step);
	float* depth_map_ptr = (float *)(depth_map + y*depth_map.step);


	if(x < Width && y < Height){

		int label = output_labels_ptr[x];
		bool is_edge_point = edge_map_8u_ptr[x] == 0 && depth_map_ptr[x] > 0.0f;
		if (is_edge_point)
      	{
			// We assign edgepoints by default to -1.
			label = -1;
			// 获取孔洞点的坐标
			float edge_point[3] = {0.0f};
			edge_point[0] = depth_map_ptr[3*x];
			edge_point[1] = depth_map_ptr[3*x + 1];
			edge_point[2] = depth_map_ptr[3*x + 2];

			constexpr double kMinNearestNeighborDistance = 0.05;
			double min_dist = kMinNearestNeighborDistance;
			constexpr int kFilterSizeHalfFloored = 4u;

			for (int i = -kFilterSizeHalfFloored; i <= kFilterSizeHalfFloored; ++i)
			{
				if (static_cast<int>(x) + i < 0)
				{
					continue;
				}
				if (static_cast<int>(x) + i >= Width)
				{
					break;
				}
				for (int j = -kFilterSizeHalfFloored; j <= kFilterSizeHalfFloored; ++j)
				{
					if (static_cast<int>(y) + j < 0 || (i == 0 && j == 0))
					{
						continue;
					}
					if (static_cast<int>(y) + j >= Height)
					{
						break;
					}

					float filter_point[3]={ 0.0f };
					float* depth_map_ptr_tmp = (float *)(depth_map + (y + j)*depth_map.step);
					filter_point[0] = depth_map_ptr_tmp[3*(x + i)];
					filter_point[1] = depth_map_ptr_tmp[3*(x + i) + 1];
					filter_point[2] = depth_map_ptr_tmp[3*(x + i) + 2];

					//const double dist = cv::norm(edge_point - filter_point);
					const double dist = sqrt(pow(edge_point[0]-filter_point[0],2) + pow(edge_point[1]-filter_point[1],2) + pow(edge_point[2]-filter_point[2],2));

					if (dist >= min_dist)
					{
						continue;
					}

					uchar edge_map_8u_value = 0u;
					uchar* edge_map_8u_ptr_tmp = (uchar*)(edge_map_8u + (y + j)*edge_map_8u.step);
					edge_map_8u_value = edge_map_8u_ptr_tmp[x + i];

					float depth_image_value = 0.0f;
					float* depth_image_ptr_tmp = (float*)(depth_map + (y + j)*depth_map.step);
					depth_image_value = depth_image_ptr_tmp[x + i];


					const bool filter_point_is_edge_point = edge_map_8u_value == 0u && depth_image_value > 0.0f;
					if (!filter_point_is_edge_point)
					{
						int *output_labels_ptr_tmp = (int *)(output_labels + (y + j)*output_labels.step);
						const int label_tmp = output_labels_ptr_tmp[x + i];
						if (label_tmp < 0)
						{
						continue;
						}
						min_dist = dist;
						label = label_tmp;
						output_labels_ptr[x]= label;
					}
				}
			
			}

			if (label > 0)
			{
				output_ptr[x] = label;
			}
		}
	}

}


extern "C"
{

	void ComputeNormal_gpu(int Height, int Width, cv::cuda::PtrStep<u_char> depth_map, cv::cuda::PtrStep<u_char> normals)
	{

		int Nthreads = 256;
		dim3 Block(Nthreads, 1);
		dim3 Grid(cv::divUp(Width, Block.x), cv::divUp(Height, Block.y));
		ComputeNormalKernel<<<Grid, Block>>>(Height, Width, depth_map, normals);
	}

	void ComputeMinConvexityMap_gpu(int Height,int Width,float *padded_mat,float *padded_mat_normal,cv::cuda::PtrStep<u_char> depth_map,cv::cuda::PtrStep<u_char> normals,cv::cuda::PtrStep<u_char> minConvexityMap,float mask_threshold, float threshold){
		int Nthreads = 256;
		dim3 Block(Nthreads,1);
		dim3 Grid(cv::divUp(Width,Block.x),cv::divUp(Height,Block.y));
		ComputeMinConvexityMapKernel<<<Grid,Block>>>(Height,Width,padded_mat,padded_mat_normal,depth_map,normals,minConvexityMap,mask_threshold,threshold);
	}

	void ComputeOutPut_gpu(int Height,int Width,cv::cuda::PtrStep<u_char> output_labels,cv::cuda::PtrStep<u_char> output,cv::cuda::PtrStep<u_char> edge_map_8u,cv::cuda::PtrStep<u_char> depth_map){
		int Nthreads = 256;
		dim3 Block(Nthreads,1);
		dim3 Grid(cv::divUp(Width,Block.x),cv::divUp(Height,Block.y));
		ComputeOutPut<<<Grid,Block>>>(Height,Width,output_labels,output,edge_map_8u,depth_map);
	}
}