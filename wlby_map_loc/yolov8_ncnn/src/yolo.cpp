//
// Created by jazzey on 2023/11/14.
//
#include "yolo.h"

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define MAX_STRIDE 32

namespace wlby::yolo {

    static void slice(const ncnn::Mat &in, ncnn::Mat &out, int start, int end, int axis) {
        ncnn::Option opt;
        opt.num_threads = 4;
        opt.use_fp16_storage = false;
        opt.use_packing_layout = false;

        ncnn::Layer *op = ncnn::create_layer("Crop");

        // set param
        ncnn::ParamDict pd;

        ncnn::Mat axes = ncnn::Mat(1);
        axes.fill(axis);
        ncnn::Mat ends = ncnn::Mat(1);
        ends.fill(end);
        ncnn::Mat starts = ncnn::Mat(1);
        starts.fill(start);
        pd.set(9, starts);// start
        pd.set(10, ends);// end
        pd.set(11, axes);//axes

        op->load_param(pd);

        op->create_pipeline(opt);

        // forward
        op->forward(in, out, opt);

        op->destroy_pipeline(opt);

        delete op;
    }

    static void interp(const ncnn::Mat &in, const float &scale, const int &out_w, const int &out_h, ncnn::Mat &out) {
        ncnn::Option opt;
        opt.num_threads = 4;
        opt.use_fp16_storage = false;
        opt.use_packing_layout = false;

        ncnn::Layer *op = ncnn::create_layer("Interp");

        // set param
        ncnn::ParamDict pd;
        pd.set(0, 2);// resize_type
        pd.set(1, scale);// height_scale
        pd.set(2, scale);// width_scale
        pd.set(3, out_h);// height
        pd.set(4, out_w);// width

        op->load_param(pd);

        op->create_pipeline(opt);

        // forward
        op->forward(in, out, opt);

        op->destroy_pipeline(opt);

        delete op;
    }

    static void reshape(const ncnn::Mat &in, ncnn::Mat &out, int c, int h, int w, int d) {
        ncnn::Option opt;
        opt.num_threads = 4;
        opt.use_fp16_storage = false;
        opt.use_packing_layout = false;

        ncnn::Layer *op = ncnn::create_layer("Reshape");

        // set param
        ncnn::ParamDict pd;

        pd.set(0, w);// start
        pd.set(1, h);// end
        if (d > 0)
            pd.set(11, d);//axes
        pd.set(2, c);//axes
        op->load_param(pd);

        op->create_pipeline(opt);

        // forward
        op->forward(in, out, opt);

        op->destroy_pipeline(opt);

        delete op;
    }

    static void sigmoid(ncnn::Mat &bottom) {
        ncnn::Option opt;
        opt.num_threads = 4;
        opt.use_fp16_storage = false;
        opt.use_packing_layout = false;

        ncnn::Layer *op = ncnn::create_layer("Sigmoid");

        op->create_pipeline(opt);

        // forward

        op->forward_inplace(bottom, opt);
        op->destroy_pipeline(opt);

        delete op;
    }

    static float fast_exp(float x) {
        union {
            uint32_t i;
            float f;
        } v{};
        v.i = (1 << 23) * (1.4426950409 * x + 126.93490512f);
        return v.f;
    }

    static float sigmoid(float x) {
        return 1.0f / (1.0f + fast_exp(-x));
    }

    static void matmul(const std::vector<ncnn::Mat> &bottom_blobs, ncnn::Mat &top_blob) {
        ncnn::Option opt;
        opt.num_threads = 2;
        opt.use_fp16_storage = false;
        opt.use_packing_layout = false;

        ncnn::Layer *op = ncnn::create_layer("MatMul");

        // set param
        ncnn::ParamDict pd;
        pd.set(0, 0);// axis

        op->load_param(pd);

        op->create_pipeline(opt);
        std::vector<ncnn::Mat> top_blobs(1);
        op->forward(bottom_blobs, top_blobs, opt);
        top_blob = top_blobs[0];

        op->destroy_pipeline(opt);

        delete op;
    }

    static float intersection_area(const Object &a, const Object &b) {
        cv::Rect_<float> inter = a.rect & b.rect;
        return inter.area();
    }

    static void qsort_descent_inplace(std::vector<Object> &faceobjects, int left, int right) {
        int i = left;
        int j = right;
        float p = faceobjects[(left + right) / 2].prob;

        while (i <= j) {
            while (faceobjects[i].prob > p)
                i++;

            while (faceobjects[j].prob < p)
                j--;

            if (i <= j) {
                // swap
                std::swap(faceobjects[i], faceobjects[j]);

                i++;
                j--;
            }
        }

        //     #pragma omp parallel sections
        {
            //         #pragma omp section
            {
                if (left < j) qsort_descent_inplace(faceobjects, left, j);
            }
            //         #pragma omp section
            {
                if (i < right) qsort_descent_inplace(faceobjects, i, right);
            }
        }
    }

    static void qsort_descent_inplace(std::vector<Object> &faceobjects) {
        if (faceobjects.empty())
            return;

        qsort_descent_inplace(faceobjects, 0, faceobjects.size() - 1);
    }

    static void nms_sorted_bboxes(const std::vector<Object> &faceobjects, std::vector<int> &picked, float nms_threshold) {
        picked.clear();

        const int n = faceobjects.size();

        std::vector<float> areas(n);
        for (int i = 0; i < n; i++) {
            areas[i] = faceobjects[i].rect.width * faceobjects[i].rect.height;
        }

        for (int i = 0; i < n; i++) {
            const Object &a = faceobjects[i];

            int keep = 1;
            for (int j = 0; j < (int) picked.size(); j++) {
                const Object &b = faceobjects[picked[j]];

                // intersection over union
                float inter_area = intersection_area(a, b);
                float union_area = areas[i] + areas[picked[j]] - inter_area;
                // float IoU = inter_area / union_area
                if (inter_area / union_area > nms_threshold)
                    keep = 0;
            }

            if (keep)
                picked.push_back(i);
        }
    }

    static void generate_proposals(std::vector<GridAndStride> grid_strides, const ncnn::Mat &pred, float prob_threshold, std::vector<Object> &objects) {
        const int num_points = grid_strides.size();
        const int num_class = Class_Nums;
        const int reg_max_1 = 16;

        for (int i = 0; i < num_points; i++) {
            const float *scores = pred.row(i) + 4 * reg_max_1;

            // find label with max score
            int label = -1;
            float score = -FLT_MAX;
            for (int k = 0; k < num_class; k++) {
                float confidence = scores[k];
                if (confidence > score) {
                    label = k;
                    score = confidence;
                }
            }

            float box_prob = sigmoid(score);

            if (box_prob >= prob_threshold) {
                ncnn::Mat bbox_pred(reg_max_1, 4, (void *) pred.row(i));
                {
                    ncnn::Layer *softmax = ncnn::create_layer("Softmax");

                    ncnn::ParamDict pd;
                    pd.set(0, 1); // axis
                    pd.set(1, 1);
                    softmax->load_param(pd);

                    ncnn::Option opt;
                    opt.num_threads = 1;
                    opt.use_packing_layout = false;

                    softmax->create_pipeline(opt);

                    softmax->forward_inplace(bbox_pred, opt);

                    softmax->destroy_pipeline(opt);

                    delete softmax;
                }

                float pred_ltrb[4];
                for (int k = 0; k < 4; k++) {
                    float dis = 0.f;
                    const float *dis_after_sm = bbox_pred.row(k);
                    for (int l = 0; l < reg_max_1; l++) {
                        dis += l * dis_after_sm[l];
                    }

                    pred_ltrb[k] = dis * grid_strides[i].stride;
                }

                float pb_cx = (grid_strides[i].grid0 + 0.5f) * grid_strides[i].stride;
                float pb_cy = (grid_strides[i].grid1 + 0.5f) * grid_strides[i].stride;

                float x0 = pb_cx - pred_ltrb[0];
                float y0 = pb_cy - pred_ltrb[1];
                float x1 = pb_cx + pred_ltrb[2];
                float y1 = pb_cy + pred_ltrb[3];
                
                if (y1 <= y0 || x1 <= x0)
            	 {
                   continue;
                }

                Object obj;
                obj.rect.x = x0;
                obj.rect.y = y0;
                obj.rect.width = x1 - x0;
                obj.rect.height = y1 - y0;
                obj.label_int = label;
                obj.prob = box_prob;
                obj.mask_feat.resize(32);
                std::copy(pred.row(i) + 64 + num_class, pred.row(i) + 64 + num_class + 32, obj.mask_feat.begin());
                objects.push_back(obj);
            }

        }
    }

    static void generate_grids_and_stride(const int target_w, const int target_h, std::vector<int> &strides, std::vector<GridAndStride> &grid_strides) {
        for (int i = 0; i < (int) strides.size(); i++) {
            int stride = strides[i];
            int num_grid_w = target_w / stride;
            int num_grid_h = target_h / stride;
            for (int g1 = 0; g1 < num_grid_h; g1++) {
                for (int g0 = 0; g0 < num_grid_w; g0++) {
                    GridAndStride gs;
                    gs.grid0 = g0;
                    gs.grid1 = g1;
                    gs.stride = stride;
                    grid_strides.push_back(gs);
                }
            }
        }
    }

    static void decode_mask(const ncnn::Mat &mask_feat, const int &img_w, const int &img_h,
                            const ncnn::Mat &mask_proto, const ncnn::Mat &in_pad, const int &wpad, const int &hpad,
                            ncnn::Mat &mask_pred_result) {
        ncnn::Mat masks;
        matmul(std::vector<ncnn::Mat>{mask_feat, mask_proto}, masks);
        sigmoid(masks);
        reshape(masks, masks, masks.h, in_pad.h / 4, in_pad.w / 4, 0);
        slice(masks, mask_pred_result, (wpad / 2) / 4, (in_pad.w - wpad / 2) / 4, 2);
        slice(mask_pred_result, mask_pred_result, (hpad / 2) / 4, (in_pad.h - hpad / 2) / 4, 1);
        interp(mask_pred_result, 4.0, img_w, img_h, mask_pred_result);

    }


    int Yolo::detect(const cv::Mat& rgb, std::vector<Object>& objects, float prob_threshold, float nms_threshold){

        int width  = rgb.cols;
        int height = rgb.rows;

        //将图像的大小调整为 target_size_的倍数
        int w = width;
        int h = height;
        float scale = 1.f;
        if (w > h) {
            scale = (float) target_size_ / w;
            w = target_size_;
            h = h * scale;
        }
        else {
            scale = (float) target_size_ / h;
            h = target_size_;
            w = w * scale;
        }
        ncnn::Mat in = ncnn::Mat::from_pixels_resize(rgb.data, ncnn::Mat::PIXEL_RGB2BGR, width, height, w, h);

        //对调整的mat进行填充
        int wpad = (w + MAX_STRIDE - 1) / MAX_STRIDE * MAX_STRIDE - w;
        int hpad = (h + MAX_STRIDE - 1) / MAX_STRIDE * MAX_STRIDE - h;
        ncnn::Mat in_pad;
        /*
            参数in是原始输入图像。
            参数in_pad是填充后的输出图像。
            hpad / 2和hpad - hpad / 2分别表示垂直方向上填充的上方和下方的像素数。
            wpad / 2和wpad - wpad / 2分别表示水平方向上填充的左侧和右侧的像素数。
            ncnn::BORDER_CONSTANT表示使用常数值填充边界。
            0.f是填充的常数值，即边界填充的像素值为0。
         */
        ncnn::copy_make_border(in, in_pad, hpad / 2, hpad - hpad / 2, wpad / 2, wpad - wpad / 2, ncnn::BORDER_CONSTANT,0.f);

        //进行均值归一化
        //参数 0 表示归一化操作针对所有通道进行，即对图像的每个像素点的每个通道分别进行均值归一化。
        //参数 norm_vals_ 是一个归一化参数，用于减去图像的均值并除以标准差。这些参数通常是在模型训练过程中计算得到的
        in_pad.substract_mean_normalize(0, norm_vals_);


        //创建了一个名为 ex 的 Extractor 对象，该对象用于从 YOLO 模型中提取特征和输出。
        //使用 ex.input 函数将经过预处理的图像 in_pad 作为输入，"images" 是输入的名称，这个名称通常在模型定义时指定。
        //调用 ex.extract 函数，从 YOLO 模型中提取名为 "output0" 的输出到 out 变量中。这个输出通常是目标检测模型对图像进行处理后得到的检测结果。
        //接着，调用 ex.extract 函数，从 YOLO 模型中提取名为 "output1" 的输出到 mask_proto 变量中。这个输出可能是一些辅助信息或者辅助预测结果，具体取决于 YOLO 模型的设计。
        ncnn::Extractor ex = yolo_.create_extractor();
        ex.input("images", in_pad);
        ncnn::Mat out;
        ex.extract("output0", out);
        ncnn::Mat mask_proto;
        ex.extract("output1", mask_proto);


        std::vector<int> strides = {8, 16, 32}; // might have stride=64
        std::vector<GridAndStride> grid_strides;
        generate_grids_and_stride(in_pad.w, in_pad.h, strides, grid_strides);


        std::vector<Object> proposals;
        std::vector<Object> objects8;
        generate_proposals(grid_strides, out, prob_threshold, objects8);
        proposals.insert(proposals.end(), objects8.begin(), objects8.end());


        // sort all proposals by score from highest to lowest
        qsort_descent_inplace(proposals);

        // apply nms with nms_threshold
        std::vector<int> picked;
        nms_sorted_bboxes(proposals, picked, nms_threshold);

        int count = picked.size();

        ncnn::Mat mask_feat = ncnn::Mat(32, count, sizeof(float));
        for (int i = 0; i < count; i++) {
            float *mask_feat_ptr = mask_feat.row(i);
            std::memcpy(mask_feat_ptr, proposals[picked[i]].mask_feat.data(), sizeof(float) * proposals[picked[i]].mask_feat.size());
        }


        ncnn::Mat mask_pred_result;
        decode_mask(mask_feat, width, height, mask_proto, in_pad, wpad, hpad, mask_pred_result);

        objects.resize(count);
        for (int i = 0; i < count; i++) {
            objects[i] = proposals[picked[i]];

            // adjust offset to original unpadded
            float x0 = (objects[i].rect.x - (wpad / 2)) / scale;
            float y0 = (objects[i].rect.y - (hpad / 2)) / scale;
            float x1 = (objects[i].rect.x + objects[i].rect.width - (wpad / 2)) / scale;
            float y1 = (objects[i].rect.y + objects[i].rect.height - (hpad / 2)) / scale;

            // clip
            x0 = std::max(std::min(x0, (float) (width - 1)), 0.f);
            y0 = std::max(std::min(y0, (float) (height - 1)), 0.f);
            x1 = std::max(std::min(x1, (float) (width - 1)), 0.f);
            y1 = std::max(std::min(y1, (float) (height - 1)), 0.f);

            objects[i].rect.x = x0;
            objects[i].rect.y = y0;
            objects[i].rect.width = x1 - x0;
            objects[i].rect.height = y1 - y0;
            objects[i].label_str = Class_names[objects[i].label_int];

            objects[i].mask = cv::Mat::zeros(height, width, CV_32FC1);
            cv::Mat mask = cv::Mat(height, width, CV_32FC1, (float *) mask_pred_result.channel(i));
            mask(objects[i].rect).copyTo(objects[i].mask(objects[i].rect));
        }
        return 0;
    }

    int Yolo::draw(cv::Mat &rgb, const std::vector<Object> &objects) {

        int color_index = 0;
        for (size_t i = 0; i < objects.size(); i++) {
            const Object &obj = objects[i];
            const unsigned char *color = colors[color_index % Class_Nums];
            color_index++;

            cv::Scalar cc(color[0], color[1], color[2]);

            for (int y = 0; y < rgb.rows; y++) {
                uchar *image_ptr = rgb.ptr(y);
                const float *mask_ptr = obj.mask.ptr<float>(y);
                for (int x = 0; x < rgb.cols; x++) {
                    if (mask_ptr[x] >= 0.5) {
                        image_ptr[0] = cv::saturate_cast<uchar>(image_ptr[0] * 0.5 + color[2] * 0.5);
                        image_ptr[1] = cv::saturate_cast<uchar>(image_ptr[1] * 0.5 + color[1] * 0.5);
                        image_ptr[2] = cv::saturate_cast<uchar>(image_ptr[2] * 0.5 + color[0] * 0.5);
                    }
                    image_ptr += 3;
                }
            }
            cv::rectangle(rgb, obj.rect, cc, 2);

            char text[256];
            sprintf(text, "%s %.1f%%", Class_names[obj.label_int].c_str(), obj.prob * 100);

            int baseLine = 0;
            cv::Size label_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);

            int x = obj.rect.x;
            int y = obj.rect.y - label_size.height - baseLine;
            if (y < 0)
                y = 0;
            if (x + label_size.width > rgb.cols)
                x = rgb.cols - label_size.width;

            cv::rectangle(rgb, cv::Rect(cv::Point(x, y), cv::Size(label_size.width, label_size.height + baseLine)),
                          cv::Scalar(255, 255, 255), -1);

            cv::putText(rgb, text, cv::Point(x, y + label_size.height),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
        }
        return 0;
    }

}
