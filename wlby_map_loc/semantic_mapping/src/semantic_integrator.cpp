//
// Created by jazzey on 2023/12/18.
//

#include "semantic_integrator.h"

namespace wlby::semantic_integrator{

    //输入三维点云输出占据栅格的索引
    uint64 SemanticIntegrator::GetOccpuiedVoxels(const Localization::Pos3D &point)
    {
        Localization::Pos3Di vox;
        vox_pos_convert_tools_.pos2Vox(point, vox);
        if (!vox_pos_convert_tools_.voxInRange(vox))
        {
            return -1;
        }
        uint64 idx = vox_pos_convert_tools_.vox2Idx(vox);
        return idx;
    }

    //进来的是一帧数据分割的点云
    void SemanticIntegrator::integrateSegments(std::vector<std::shared_ptr<depth_segmentation::SegmentWithIdx>>& segment_with_idx_vec,
                           pcl::PointCloud<pcl::PointXYZRGB>::Ptr& semantic_cloud_ptr,
                           pcl::PointCloud<pcl::PointXYZRGB>::Ptr& instance_cloud_ptr) {

        int index = 0;
        //将一帧内所有分割出来的点云进行遍历，融合语义标签和实例标签
        for (std::shared_ptr<depth_segmentation::SegmentWithIdx> &segment_ptr: segment_with_idx_vec) {

            std::string class_name = yolo::Class_names[segment_ptr->semantic_label_];
            log__save("SemanticMapping",kLogLevel_Info, kLogTarget_Filesystem,"======= idx %d Get Object %s confidence %f =======", index, class_name.c_str(), segment_ptr->score_);
            ++index;

            if(!setPassSemanticObject(segment_ptr->semantic_label_,Parameter::SemanticIntegratorParams::semantic_label_vec)){
                continue;
            }

            if(segment_ptr->score_ < Parameter::SemanticIntegratorParams::score_limit){
                continue;
            }

            // 计算当前segment点云占据的格子
            for (const Localization::Pos3D& pos : segment_ptr->points_) {
                uint64 occ_idx = GetOccpuiedVoxels(pos);
                if (occ_idx >= 0) {
                    segment_ptr->idx_set.emplace(occ_idx);
                }
            }

            int limit_nums = Parameter::SemanticIntegratorParams::nums_limit;
            double fusion_ratio = Parameter::SemanticIntegratorParams::fusion_ratio_limit;

            //特殊处理
            if(class_name == "water_dispenser_face"){
                limit_nums = 100;
                //fusion_ratio = 0.25;
            }



            if (segment_ptr->idx_set.size() < limit_nums) {
                log__save("SemanticMapping",kLogLevel_Info, kLogTarget_Filesystem,"Pass Object %s, idx_set %d", class_name.c_str(),segment_ptr->idx_set.size());
                continue;
            }

            std::vector<uint8> max_semantic_vec;
            std::vector<int> max_semantic_count_vec;
            getSemanticLabelAndCount(max_semantic_vec, max_semantic_count_vec);
            for(int i = 0; i < max_semantic_vec.size(); i++){
                std::string class_name = yolo::Class_names[max_semantic_vec[i]];
                log__save("SemanticMapping",kLogLevel_Info, kLogTarget_Filesystem,"Last Segment: %d Object: %s count: %d",i, class_name.c_str(),max_semantic_count_vec[i]);
            }

            //与所有的当前更新的segment进行融合判断
            bool if_is_new_class = true;

            std::set<int> merged_segment_idx_vec;
            for (int i = 0; i < last_segment_ptr_vec_.size(); ++i) {
                IntegrateSegmentStruct last_iss = last_segment_ptr_vec_[i];

                bool if_first_fusion = false;
                bool if_second_fusion = false;

                //1. 当前的segment与之前的segment进行融合,当重复的格子大于一个数量，则合并
                int overlap_num = getOverlapNumOfTwoSet(last_iss.integrated_idx, segment_ptr->idx_set);
                int last_size = last_iss.integrated_idx.size();
                double ratio = overlap_num * 1.0 / last_size;
                double ratio2 = overlap_num * 1.0 / segment_ptr->idx_set.size();
                if (ratio > fusion_ratio || ratio2 > fusion_ratio) {
                    // 这是需要融合的融合类
                    if_first_fusion = true;
                    if_is_new_class = false;
                    merged_segment_idx_vec.emplace(i);
                }
                log__save("SemanticMapping",kLogLevel_Info, kLogTarget_Filesystem,"Fusion %d Ratio %f ratio2 %f, overlap_num %d, last_size %d",i,ratio,ratio2,overlap_num,last_size);

                //获取最大可能的语义类别
                uint8_t max_semantic = 0;
                int max_semantic_count = -1;
                for (const auto &it: last_iss.semantic_label_map_) {
                    if (it.second > max_semantic_count) {
                        max_semantic = it.first;
                        max_semantic_count = it.second;
                    }
                }
                //当是同一类物体的时候，才进行包围盒相交率检测
                if(max_semantic != segment_ptr->semantic_label_){
                    continue;
                }

                //2. 当前的2D包围盒与历史2D包围盒相交率>0.5，则合并
                Eigen::AlignedBox2i last_box;
                for(const auto& item: last_iss.integrated_idx){
                    Localization::Pos3Di vox = vox_pos_convert_tools_.idx2Vox(item);
                    Eigen::Vector2i vec2i(vox._ix,vox._iy);
                    last_box.extend(vec2i);
                }
                Eigen::AlignedBox2i current_box;
                for(const auto& item: segment_ptr->idx_set){
                    Localization::Pos3Di vox = vox_pos_convert_tools_.idx2Vox(item);
                    Eigen::Vector2i vec2i(vox._ix,vox._iy);
                    current_box.extend(vec2i);
                }
                Eigen::AlignedBox2i intersection = last_box.intersection(current_box);
                if (!intersection.isEmpty()) {
                    int intersection_area = intersection.sizes().x() * intersection.sizes().y();
                    int area_last = last_box.sizes().x() * last_box.sizes().y();
                    int area_current = current_box.sizes().x() * current_box.sizes().y();
                    double box_ratio1 = 1.0 * intersection_area / area_last;
                    double box_ratio2 = 1.0 * intersection_area / area_current;
                    if(box_ratio1 > Parameter::SemanticIntegratorParams::obbbox_fusion_ratio_limit || box_ratio2 > Parameter::SemanticIntegratorParams::obbbox_fusion_ratio_limit){
                        if_second_fusion = true;
                        if_is_new_class = false;
                        merged_segment_idx_vec.emplace(i);
                    }
                    log__save("SemanticMapping",kLogLevel_Info, kLogTarget_Filesystem,"Obbbox Fusion %d ratio %f ratio2 %f, area_last %d area_current %d intersection_area %d",i,box_ratio1,box_ratio2,area_last,area_current,intersection_area);
                }


                log__save("SemanticMapping",kLogLevel_Info, kLogTarget_Filesystem,"Same Semantic Fusion %d  check %d - %d!",i,if_first_fusion,if_second_fusion);




            }

            if (if_is_new_class) {
                //之前的last_segment_ptr_vec没有与当前segment重合的部分，是一个新的类
                IntegrateSegmentStruct iss;
                IntegrateSegmentSem(iss, segment_ptr->instance_label_, segment_ptr->semantic_label_, 1);
                IntegrateSegmentVox(iss, segment_ptr->idx_set);
                last_segment_ptr_vec_.push_back(iss);

                log__save("SemanticMapping",kLogLevel_Info, kLogTarget_Filesystem,"New Class!, Last Segment Vec Size: %d",last_segment_ptr_vec_.size());


            }
            else {

                log__save("SemanticMapping",kLogLevel_Info, kLogTarget_Filesystem,"Not New Class!, Merge Segment Vec Size: %d",merged_segment_idx_vec.size());

                //要融合为一个类的标号合集：merged_segment_idx_vec
                //将 0 号作为最终的类合集
                CHECK_GT(merged_segment_idx_vec.size(), 0);

                int idx = *(merged_segment_idx_vec.begin());
                for (auto it = merged_segment_idx_vec.begin(); it != merged_segment_idx_vec.end(); ++it) {

                    if(it == merged_segment_idx_vec.begin()){
                        continue;
                    }

                    int cur_idx = *it;

                    //将 1 ~ N的实例标签合并segment放到0号segment中
                    last_segment_ptr_vec_[idx].instance_label_map_.insert(
                            last_segment_ptr_vec_[cur_idx].instance_label_map_.begin(),
                            last_segment_ptr_vec_[cur_idx].instance_label_map_.end()
                    );

                    //将 1 ~ N的语义合并segment放到0号segment中
                    for (const auto &it: last_segment_ptr_vec_[cur_idx].semantic_label_map_) {
                        last_segment_ptr_vec_[idx].semantic_label_map_[it.first] += it.second;
                    }
                    IntegrateSegmentVox(last_segment_ptr_vec_[idx], last_segment_ptr_vec_[cur_idx].integrated_idx);
                }
                IntegrateSegmentSem(last_segment_ptr_vec_[idx], segment_ptr->instance_label_, segment_ptr->semantic_label_, 1);
                IntegrateSegmentVox(last_segment_ptr_vec_[idx], segment_ptr->idx_set);

                // 标号的删除
                for (auto it = merged_segment_idx_vec.begin(); it != merged_segment_idx_vec.end(); ++it) {
                    if (it != merged_segment_idx_vec.begin()) {
                        last_segment_ptr_vec_[*it].can_delete = true;
                    }
                }

                for (auto it = last_segment_ptr_vec_.begin(); it != last_segment_ptr_vec_.end();) {
                    if (it->can_delete) {
                        it = last_segment_ptr_vec_.erase(it);
                    } else {
                        it++;
                    }
                }

            }

        }

        point_vec_.clear();

        resetInstanceLabel();

        {
            std::lock_guard<std::mutex> lock(data_mutx);
            for (int i = 0; i < last_segment_ptr_vec_.size(); ++i) {

                // 先选择语义，最多的观测次数
                IntegrateSegmentStruct iss = last_segment_ptr_vec_[i];

                uint8_t max_semantic = 0;
                int max_semantic_count = -1;
                for (const auto &it: iss.semantic_label_map_) {
                    if (it.second > max_semantic_count) {
                        max_semantic = it.first;
                        max_semantic_count = it.second;
                    }
                }

                if (max_semantic_count < Parameter::SemanticIntegratorParams::obs_pass_limit) {
                    std::string class_name = yolo::Class_names[max_semantic];
                    log__save("SemanticMapping", kLogLevel_Info, kLogTarget_Filesystem,
                              "Pass Object %s, semantic_count %d", class_name.c_str(), max_semantic_count);
                    continue;
                }


                uint32_t max_instance = getFreshInstanceLabel();

                srand(max_semantic * 7);
                int r = (rand() % 255);
                int g = (rand() % 255);
                int b = (rand() % 255);

                srand(max_instance * 11);
                int i_r = (rand() % 255);
                int i_g = (rand() % 255);
                int i_b = (rand() % 255);

                PointCloudWithSemanticLabel pws;

                pcl::PointCloud<pcl::PointXYZRGB>::Ptr instance_cloud_ptr2(new pcl::PointCloud<pcl::PointXYZRGB>());
                for (const uint64 &idx: iss.integrated_idx) {

                    Localization::Pos3Di vox = vox_pos_convert_tools_.idx2Vox(idx);
                    Localization::Pos3D pos;
                    vox_pos_convert_tools_.vox2Pos(vox, pos);
                    pcl::PointXYZRGB pt;
                    pt.x = pos._xyz[0];
                    pt.y = pos._xyz[1];
                    pt.z = pos._xyz[2];
                    pt.r = r;
                    pt.g = g;
                    pt.b = b;
                    semantic_cloud_ptr->push_back(pt);

                    pcl::PointXYZRGB i_pt;
                    i_pt.x = pos._xyz[0];
                    i_pt.y = pos._xyz[1];
                    i_pt.z = pos._xyz[2];
                    i_pt.r = i_r;
                    i_pt.g = i_g;
                    i_pt.b = i_b;
                    instance_cloud_ptr->push_back(i_pt);

                    instance_cloud_ptr2->push_back(i_pt);
                }

                // 创建 RadiusOutlierRemoval 对象
                pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outlier_removal;
                outlier_removal.setRadiusSearch(0.1);  // 设置搜索半径
                outlier_removal.setMinNeighborsInRadius(5);  // 设置最小邻居数
                outlier_removal.setInputCloud(semantic_cloud_ptr);  // 设置输入点云
                outlier_removal.filter(*semantic_cloud_ptr);

                outlier_removal.setInputCloud(instance_cloud_ptr2);  // 设置输入点云
                outlier_removal.filter(*instance_cloud_ptr2);

                outlier_removal.setInputCloud(instance_cloud_ptr);  // 设置输入点云
                outlier_removal.filter(*instance_cloud_ptr);

                pws.instance_cloud_ptr = instance_cloud_ptr2;
                pws.semantic_label = max_semantic;
                pws.instance_label = max_instance;

                point_vec_.push_back(pws);
            }
        }

        log__save("SemanticMapping",kLogLevel_Info, kLogTarget_Filesystem,"\n");

    }


    void SemanticIntegrator::getOBBBox(std::vector<OBBBox>& obbbox_vec){

        std::lock_guard<std::mutex> lock(data_mutx);
        for(const auto& it : point_vec_){

            pcl::MomentOfInertiaEstimation<pcl::PointXYZRGB> moi;
            moi.setInputCloud(it.instance_cloud_ptr);
            moi.compute();
            /*pcl::PointXYZRGB OBB_min_pt, OBB_max_pt, center_pos;
            Eigen::Matrix3f OBB_matrix_r;
            moi.getOBB(OBB_min_pt, OBB_max_pt, center_pos, OBB_matrix_r);

            Eigen::Vector3f center_position(center_pos.x, center_pos.y, 0);

            Eigen::Vector3f p00(OBB_min_pt.x, OBB_min_pt.y, 0);
            Eigen::Vector3f p10(OBB_max_pt.x, OBB_min_pt.y, 0);
            Eigen::Vector3f p11(OBB_max_pt.x, OBB_max_pt.y, 0);
            Eigen::Vector3f p01(OBB_min_pt.x, OBB_max_pt.y, 0);
            p00 = OBB_matrix_r * p00 + center_position;
            p10 = OBB_matrix_r * p10 + center_position;
            p11 = OBB_matrix_r * p11 + center_position;
            p01 = OBB_matrix_r * p01 + center_position;*/

            pcl::PointXYZRGB min_point_AABB;
            pcl::PointXYZRGB max_point_AABB;
            moi.getAABB(min_point_AABB,max_point_AABB);

            if(max_point_AABB.x > max_.x()){
                max_point_AABB.x = max_.x() - 0.05;
            }
            if(max_point_AABB.y > max_.y()){
                max_point_AABB.y = max_.y() - 0.05;
            }
            if(min_point_AABB.x < min_.x()){
                min_point_AABB.x = min_.x() + 0.05;
            }
            if(min_point_AABB.y < min_.y()){
                min_point_AABB.y = min_.y() + 0.05;
            }


            Localization::Pos2D pos00(min_point_AABB.x,min_point_AABB.y);
            Localization::Pos2D pos10(max_point_AABB.x,min_point_AABB.y);
            Localization::Pos2D pos11(max_point_AABB.x,max_point_AABB.y);
            Localization::Pos2D pos01(min_point_AABB.x,max_point_AABB.y);

            OBBBox obbbox;
            obbbox.obb_box.push_back(pos00);
            obbbox.obb_box.push_back(pos10);
            obbbox.obb_box.push_back(pos11);
            obbbox.obb_box.push_back(pos01);
            obbbox.obb_box.push_back(pos00);

            obbbox.semantic_label = it.semantic_label;
            obbbox.instance_label = it.instance_label;
            obbbox.r = it.instance_cloud_ptr->points[0].r / 255.0;
            obbbox.g = it.instance_cloud_ptr->points[0].g / 255.0;
            obbbox.b = it.instance_cloud_ptr->points[0].b / 255.0;
            obbbox.instance_cloud_ptr = it.instance_cloud_ptr;
            obbbox_vec.push_back(obbbox);
        }

    }

}