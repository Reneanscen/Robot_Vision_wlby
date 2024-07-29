//
// Created by jazzey on 2023/11/14.
//

#include "yolov8_ncnn_node.h"

namespace wlby::yolo{

    void YoloV8NcnnNode::dataHandleLoop(){

        while(1){
            SleepUtils::Sleep(200);

            sensor_msgs::msg::Image::ConstSharedPtr cur_rgb_image_ptr;
            getRGBImagePtr(cur_rgb_image_ptr);

            if(cur_rgb_image_ptr == nullptr){
                continue;
            }

            double cur_rgb_image_msg_time = TimeUtils::RosTimeToAbsTime(cur_rgb_image_ptr->header.stamp);

            if (last_rgb_image_msg_time_ > 0 && TWO_FLOAT_EQUAL(last_rgb_image_msg_time_, cur_rgb_image_msg_time)) {
                continue;
            }
            last_rgb_image_msg_time_ = cur_rgb_image_msg_time;

            cv_bridge::CvImagePtr cv_rgb_image(new cv_bridge::CvImage);
            cv_rgb_image = cv_bridge::toCvCopy(cur_rgb_image_ptr, cur_rgb_image_ptr->encoding);

            int height = cv_rgb_image->image.rows;
            int width = cv_rgb_image->image.cols;
            cv::Mat out(height, width, CV_8UC3, cv::Scalar(0.0));

            if (cur_rgb_image_ptr->encoding == sensor_msgs::image_encodings::RGB8)
            {
                cv::cvtColor(cv_rgb_image->image, out, CV_RGB2BGR);
            }
            cv::Mat image = out;

            auto start_time = TimeUtils::GetThreadCpuTimeSeconds();
            std::vector<Object> boxes_and_masks;
            yolo_ptr_->detect(image, boxes_and_masks);
            yolo_ptr_->draw(image, boxes_and_masks);
            auto end_time = TimeUtils::GetThreadCpuTimeSeconds();
            auto elapsed_time = end_time - start_time;
            //std::cout << "detected " << boxes_and_masks.size() << "  objects .. cost time " << elapsed_time << "s" << std::endl;

            cv::imshow("detected result", image);
            cv::waitKey(1);

            //赋值头
            segment_msg::msg::Result result_msg;
            result_msg.header = cur_rgb_image_ptr->header;
            for(const Object& bm : boxes_and_masks){
                sensor_msgs::msg::RegionOfInterest roi;
                roi.x_offset = bm.rect.x;
                roi.y_offset = bm.rect.y;
                roi.width = bm.rect.width;
                roi.height = bm.rect.height;
                result_msg.boxes.push_back(roi);

                result_msg.class_ids.push_back(bm.label_int);
                result_msg.class_names.push_back(bm.label_str);
                result_msg.scores.push_back(bm.prob);

                cv::Mat mask = bm.mask;
                cv::threshold(mask, mask, 0.5, 1.0, cv::THRESH_BINARY);
                mask = 255*mask;
                mask.convertTo(mask, CV_8UC1);

                sensor_msgs::msg::Image seg_mask;
                seg_mask.header = cur_rgb_image_ptr->header;
                seg_mask.height = image.rows;
                seg_mask.width = image.cols;
                seg_mask.encoding = "mono8";
                seg_mask.is_bigendian = false;
                seg_mask.step = seg_mask.width;
                seg_mask.data.assign(mask.datastart, mask.dataend);
                result_msg.masks.push_back(seg_mask);

            }
            pub_result_->publish(result_msg);
        }
    }


}