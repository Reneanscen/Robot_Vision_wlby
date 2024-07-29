//
// Created by jazzey on 2023/11/21.
//

#ifndef SEMANTIC_MAPPING_SEMANTICINTEGRATOR_H
#define SEMANTIC_MAPPING_SEMANTICINTEGRATOR_H

#include <memory>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "variables.h"
#include "common/common_type_define.h"
#include "depth_segmentation.h"
#include "vox_pos_convert_tools.h"

#include "logger/my_logger.h"

//语义标签、实例标签融合类
namespace wlby::semantic_integrator{

    struct IntegrateSegmentStruct{

        //语义标签，观测的次数
        std::map<uint8_t, int> semantic_label_map_;

        //实例标签
        std::set<uint32_t> instance_label_map_;

        //占据的格子标号
        std::set<uint64> integrated_idx;

        bool can_delete = false;
    };

    struct PointCloudWithSemanticLabel{
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr instance_cloud_ptr;
        uint8 semantic_label = 0;
        int instance_label = -1;
    };

    struct OBBBox{
        std::vector<Localization::Pos2D> obb_box;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr instance_cloud_ptr;
        uint8 semantic_label;
        int instance_label;
        float r = 1.0;
        float g = 1.0;
        float b = 1.0;
    };


    class SemanticIntegrator{
    public:
        explicit SemanticIntegrator(const VoxPosConvertTools& vox_pos_convert_tools):vox_pos_convert_tools_(vox_pos_convert_tools){


        }

        uint64 GetOccpuiedVoxels(const Localization::Pos3D &point);

        // 更新segments的实例、语义、对应voxel信息
        static void IntegrateSegmentSem(IntegrateSegmentStruct &iss, uint32_t instance_label, uint8_t semantic_label, int obs)
        {
            iss.instance_label_map_.insert(instance_label);
            iss.semantic_label_map_[semantic_label] += obs;
        }

        static void IntegrateSegmentVox(IntegrateSegmentStruct &iss, const std::set<uint64> &idx_set)
        {
            iss.integrated_idx.insert(idx_set.begin(), idx_set.end());
        }

        static int getOverlapNumOfTwoSet(const std::set<uint64> &set1, const std::set<uint64> &set2)
        {
            std::set<uint64> intersection;
            // 在intersection中找到相同的元素
            std::set_intersection(set1.begin(), set1.end(), set2.begin(), set2.end(), std::inserter(intersection, intersection.begin()));
            return intersection.size();
        }

        void resetInstanceLabel(){
            instance_label_ = 1;
        }

        uint32_t getFreshInstanceLabel(){
            return instance_label_++;
        }

        static bool setPassSemanticObject(uint8 semantic_label,const std::vector<std::string>& semantic_labels_str_vec){

            for(const auto& str: semantic_labels_str_vec){
                auto it = yolo::Classname_semanticlabel_map.find(str);
                if(it != yolo::Classname_semanticlabel_map.end() && semantic_label == it->second){
                    return true;
                }
            }
            return false;
        }

        //进来的是一帧数据分割的点云
        void integrateSegments(std::vector<std::shared_ptr<depth_segmentation::SegmentWithIdx>>& segment_with_idx_vec,
                               pcl::PointCloud<pcl::PointXYZRGB>::Ptr& semantic_cloud_ptr,
                               pcl::PointCloud<pcl::PointXYZRGB>::Ptr& instance_cloud_ptr);

        void getOBBBox(std::vector<OBBBox>& obbbox_vec);

        void setMaxMin(const Eigen::Vector2f& max,const Eigen::Vector2f& min){
            max_ = max;
            min_ = min;
        }

        void getSemanticLabelAndCount(std::vector<uint8>& max_semantic_vec,std::vector<int>& max_semantic_count_vec){
            for (int i = 0; i < last_segment_ptr_vec_.size(); ++i) {

                // 先选择语义，最多的观测次数
                IntegrateSegmentStruct iss = last_segment_ptr_vec_[i];

                uint8 max_semantic = 255;
                int max_semantic_count = -1;
                for (const auto &it: iss.semantic_label_map_) {
                    if (it.second > max_semantic_count) {
                        max_semantic = it.first;
                        max_semantic_count = it.second;
                    }
                }

                max_semantic_vec.push_back(max_semantic);
                max_semantic_count_vec.push_back(max_semantic_count);

            }

        }

    private:

        VoxPosConvertTools vox_pos_convert_tools_;

        std::vector<IntegrateSegmentStruct> last_segment_ptr_vec_;

        uint32_t instance_label_ = 1;

        std::vector<PointCloudWithSemanticLabel> point_vec_;

        std::mutex data_mutx;

        Eigen::Vector2f max_;    // cartographer地图坐标系左上角为坐标系的坐标的最大值
        Eigen::Vector2f min_;    // cartographer地图坐标系左上角为坐标系的坐标的最小值
    };


}


#endif //SEMANTIC_MAPPING_SEMANTICINTEGRATOR_H
