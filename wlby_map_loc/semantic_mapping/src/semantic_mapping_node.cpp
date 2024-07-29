//
// Created by jazzey on 2023/11/14.
//

#include "semantic_mapping_node.h"

#include <set>

namespace wlby{

    void SemanticMappingNode::markerVecPublish(){

        std::lock_guard<std::mutex> lock(marker_mutex_);
        if(!marker_msg_vec_.empty()){
            for(const auto& marker:marker_msg_vec_){
                SleepUtils::Sleep(10);
                marker_publisher_->publish(*marker);
            }
        }
    }

    void SemanticMappingNode::createMarkerText(const std::vector<semantic_integrator::OBBBox>& obbbox_vec, std::vector<std::shared_ptr<visualization_msgs::msg::Marker>>& marker_msg_vec) {

        for(int i = 0; i < obbbox_vec.size(); ++i){
            // 创建Marker消息
            auto markerMsg3 = std::make_shared<visualization_msgs::msg::Marker>();
            markerMsg3->header.frame_id = "map";  // 设置坐标系
            markerMsg3->action =  visualization_msgs::msg::Marker::ADD;
            markerMsg3->type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            markerMsg3->id = i + obbbox_vec.size();
            markerMsg3->pose.position.x = obbbox_vec[i].obb_box[0]._x;  // 设置文字位置
            markerMsg3->pose.position.y = obbbox_vec[i].obb_box[0]._y;
            markerMsg3->pose.position.z = 0.03;
            markerMsg3->pose.orientation.w = 1.0;
            markerMsg3->scale.x = 0.1;  // 设置文字大小
            markerMsg3->scale.y = 0.1;  // 设置文字大小
            markerMsg3->scale.z = 0.1;  // 设置文字大小
            markerMsg3->color.r = obbbox_vec[i].r; // 线颜色为红色
            markerMsg3->color.g = obbbox_vec[i].g;
            markerMsg3->color.b = obbbox_vec[i].b;
            markerMsg3->color.a = 1.0; // 不透明度为1.0
            markerMsg3->text = yolo::Class_names[obbbox_vec[i].semantic_label];  // 设置要显示的文字内容
            marker_msg_vec.push_back(markerMsg3);
        }
    }

    void SemanticMappingNode::createMarkerLineStripMsg(const std::vector<semantic_integrator::OBBBox>& obbbox_vec, std::vector<std::shared_ptr<visualization_msgs::msg::Marker>>& marker_msg_vec){
        // 创建第一个 Marker 消息

        for(int i = 0; i < obbbox_vec.size(); ++i){
            auto markerMsg1 = std::make_shared<visualization_msgs::msg::Marker>();
            // 设置第一个 Marker 的属性
            markerMsg1->header.frame_id = "map";
            markerMsg1->id = i;
            markerMsg1->type = visualization_msgs::msg::Marker::LINE_STRIP;
            markerMsg1->action = visualization_msgs::msg::Marker::ADD;
            markerMsg1->scale.x = 0.02; // 线宽度

            markerMsg1->color.r = obbbox_vec[i].r; // 线颜色为红色
            markerMsg1->color.g = obbbox_vec[i].g;
            markerMsg1->color.b = obbbox_vec[i].b;
            markerMsg1->color.a = 1.0; // 不透明度为1.0

            for(const auto& pos: obbbox_vec[i].obb_box){

                geometry_msgs::msg::Point point;
                point.x = pos._x;
                point.y = pos._y;
                point.z = 0.03;
                markerMsg1->points.push_back(point);
            }

            marker_msg_vec.push_back(markerMsg1);

        }




    }

    int SemanticMappingNode::readNavMap(const std::string& file_path){

        std::ifstream inputFile(file_path);

        bool read_success = false;
        if (inputFile.is_open()) {

            // 文件成功打开
            std::string line;

            while (std::getline(inputFile, line)) {
                // 处理每一行数据
                std::vector<std::string> result;
                SplitString(line, " ", result);
                if (result.size() == 2 && result[0] == "Resolution:") {
                    resolution_ = std::stod(result[1]);
                } else if (result.size() == 3 && result[0] == "Max:") {
                    max_.x() = std::stof(result[1]);
                    max_.y() = std::stof(result[2]);
                } else if (result.size() == 2 && result[0] == "CellNumsX:") {
                    num_x_cells_ = std::stoi(result[1]);
                } else if (result.size() == 2 && result[0] == "CellNumsY:") {
                    num_y_cells_ = std::stoi(result[1]);
                    read_success = true;
                    break;
                }
            }

            min_.x() =  max_.x() - num_y_cells_ * resolution_;
            min_.y() =  max_.y() - num_x_cells_ * resolution_;

            inputFile.close();

            if(read_success){
                return 0;
            }else{
                return -1;
            }
        }
        else{
            inputFile.close();
            return -1;
        }

    }

    int SemanticMappingNode::changeMapWithOccpuiedValue(const std::string& file_path,const std::vector<semantic_integrator::OBBBox>& obb_box_vec){

        std::ofstream file(file_path, std::ios_base::app);
        if (file.is_open()) {

            file << "Semantic Mapping Data Begin" << std::endl;
            for(const auto& item:obb_box_vec){

                file << "instance label: " << (int)item.instance_label<<std::endl;

                std::set<std::pair<int, int>> pairs_set;
                for(const auto& xyz_rgb_point : *item.instance_cloud_ptr){

                    Eigen::Vector2f point(xyz_rgb_point.x,xyz_rgb_point.y);
                    Eigen::Array2i cell_index = GetCellIndex(point);
                    if(Contains(cell_index)){
                        pairs_set.insert(std::make_pair(cell_index.x(),cell_index.y()));
                    }
                }
                for(const auto& it:pairs_set){
                    file<<it.first<<" "<<it.second<<std::endl;
                }
            }

            file << "Semantic Mapping Data End" << std::endl;
            file.close();

            return 0;
        }else{


            file.close();
            return -1;
        }
    }

    int SemanticMappingNode::saveSematicMap(const std::string& file_path,const std::vector<semantic_integrator::OBBBox>& obbbox_vec){

        std::ofstream outputFile(file_path);
        if (outputFile.is_open()) {
            outputFile << "Semantic Map Data" << std::endl;

            for(const auto& box : obbbox_vec){

                outputFile << "semantic label " << (int)box.semantic_label <<std::endl;
                outputFile << "instance label " << (int)box.instance_label <<std::endl;
                outputFile << "min pos " << box.obb_box[0]._x << " " <<box.obb_box[0]._y << std::endl;
                outputFile << "max pos " << box.obb_box[2]._x << " " <<box.obb_box[2]._y << std::endl;
                outputFile << "color " << box.r << " " << box.g << " " << box.b <<std::endl;
            }

            outputFile << "Finish" << std::endl;
            outputFile.close();
            return 0;
        }else{
            outputFile.close();
            return -1;
        }

    }

    void SemanticMappingNode::control_callback(control_msg::srv::Control::Request::SharedPtr request, control_msg::srv::Control::Response::SharedPtr response){

        //接受到语义地图的存储命令
        if(request->req == 1) {

            std::lock_guard<std::mutex> lock(marker_mutex_);

            log__save("MappingRequest", kLogLevel_Info, kLogTarget_Stdout | kLogTarget_Filesystem, "Get Save Semantic Map Request!");
            std::vector<semantic_integrator::OBBBox> obbbox_vec;
            semantic_integrator_->getOBBBox(obbbox_vec);

            log__save("MappingRequest", kLogLevel_Info, kLogTarget_Stdout | kLogTarget_Filesystem, "Get Obbbox Size %d",obbbox_vec.size());

            marker_msg_vec_.clear();

            std::cout<<"1..........."<<std::endl;
            createMarkerLineStripMsg(obbbox_vec,marker_msg_vec_);
            std::cout<<"2..........."<<std::endl;
            createMarkerText(obbbox_vec,marker_msg_vec_);
            std::cout<<"3..........."<<std::endl;

            saveSematicMap(Parameter::CommonParams::save_path,obbbox_vec);
            std::cout<<"4..........."<<std::endl;
            changeMapWithOccpuiedValue(Parameter::CommonParams::save_path,obbbox_vec);
            std::cout<<"5..........."<<std::endl;

            if (response->res == 0) {
                log__save("MappingRequest", kLogLevel_Info, kLogTarget_Stdout | kLogTarget_Filesystem, "Save Map Success Save");
            } else if (response->res == -1) {
                log__save("MappingRequest", kLogLevel_Info, kLogTarget_Stdout | kLogTarget_Filesystem, "Save Map Fail!");
            }
        }
    }

    void SemanticMappingNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &depth_camera_info_msg) {

        if (camera_info_ready_) {
            return;
        }

        sensor_msgs::msg::CameraInfo depth_info = *depth_camera_info_msg;
        Eigen::Vector2d depth_image_size(depth_info.width, depth_info.height);

        log__save("ParamRecord", kLogLevel_Info, kLogTarget_Stdout | kLogTarget_Filesystem, "depth image width %d , height %d",depth_info.width,depth_info.height);

        cv::Mat K_depth = cv::Mat::eye(3, 3, CV_32FC1);

        K_depth.at<float>(0, 0) = static_cast<float>(depth_info.k[0]);
        K_depth.at<float>(0, 2) = static_cast<float>(depth_info.k[2]);
        K_depth.at<float>(1, 1) = static_cast<float>(depth_info.k[4]);
        K_depth.at<float>(1, 2) = static_cast<float>(depth_info.k[5]);
        K_depth.at<float>(2, 2) = static_cast<float>(depth_info.k[8]);

        log__save("ParamRecord", kLogLevel_Info, kLogTarget_Stdout | kLogTarget_Filesystem, "%f %f %f", K_depth.at<float>(0, 0),K_depth.at<float>(0, 1),K_depth.at<float>(0, 2));
        log__save("ParamRecord", kLogLevel_Info, kLogTarget_Stdout | kLogTarget_Filesystem, "%f %f %f", K_depth.at<float>(1, 0),K_depth.at<float>(1, 1),K_depth.at<float>(1, 2));
        log__save("ParamRecord", kLogLevel_Info, kLogTarget_Stdout | kLogTarget_Filesystem, "%f %f %f", K_depth.at<float>(2, 0),K_depth.at<float>(2, 1),K_depth.at<float>(2, 2));

        depth_camera_.setCameraMatrix(K_depth);
        depth_camera_.setHeight(depth_info.height);
        depth_camera_.setWidth(depth_info.width);

        depth_segmenter_ = std::make_shared<depth_segmentation::DepthSegmenter>(depth_camera_);

        camera_info_ready_ = true;
    }

    void SemanticMappingNode::depthMatFromRosMsg(const sensor_msgs::msg::Image::ConstSharedPtr &depth_msg, cv::Mat *rescaled_depth) {

        CHECK_NOT_NULL(rescaled_depth);

        cv_bridge::CvImagePtr cv_depth_image(new cv_bridge::CvImage);
        //============ 处理depth_msg信息， 赋值cv_depth_image 和 rescaled_depth
        // TYPE_16UC1 需要单独处理，将深度值/1000, 0值变成nan
        if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {

            cv_depth_image = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);

            *rescaled_depth = cv::Mat::zeros(cv_depth_image->image.size(), CV_32FC1);
            /*
              如果输入的图像是CV_16UC1类型的（比如Kinect的），图像被转换为浮点数，除以1000得到以米为单位的深度。
              除以1000得到一个以米为单位的深度
              0的值被转换为std::numeric_limits<float>::quiet_NaN()
              否则，图像将被简单地转换为浮点数,类型为CV_32FC1
            */
            cv::rgbd::rescaleDepth(cv_depth_image->image, CV_32FC1, *rescaled_depth);
        }
        else if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
            //若类型是TYPE_32FC1, 则直接使用其深度值
            cv_depth_image = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
            *rescaled_depth = cv_depth_image->image;
        }
        else {
            //LOG(FATAL) << "Unknown depth image encoding.";
        }
        /*
          通过使用"!="运算符将 "*rescaled_depth "与自身进行比较，
          任何NaN值都将在产生的 "nan_mask "矩阵中被识别为 "真"，
          而所有其他值都是 "假"。这对于过滤无效数据或进一步处理深度图像很有用。
          setTo在为”真“，设置值kZeroValue = 0，将rescaled_depth的nan值填充为0
        */
        constexpr double kZeroValue = 0.0;
        cv::Mat nan_mask = *rescaled_depth != *rescaled_depth;
        rescaled_depth->setTo(kZeroValue, nan_mask);

        cv::medianBlur(*rescaled_depth, *rescaled_depth, 1);
    }

    void SemanticMappingNode::PoseStampedFromRosMsg(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& pose_stamped_ptr, Localization::Pose2D& pose){

        double quat[4] = { pose_stamped_ptr->pose.orientation.w , pose_stamped_ptr->pose.orientation.x , pose_stamped_ptr->pose.orientation.y, pose_stamped_ptr->pose.orientation.z};
        Localization::Orient3D orient(quat);
        orient.uniformByQuat();
        pose = Localization::Pose2D(pose_stamped_ptr->pose.position.x,pose_stamped_ptr->pose.position.y,orient._eulerDeg[0]);
    }

    void SemanticMappingNode::semanticInstanceSegmentationFromRosMsg(const segment_msg::msg::Result::ConstSharedPtr &segmentation_msg,
                                                depth_segmentation::SemanticInstanceSegmentation *semantic_instance_segmentation) {
        semantic_instance_segmentation->masks.reserve(segmentation_msg->masks.size());
        semantic_instance_segmentation->labels.reserve(segmentation_msg->masks.size());
        semantic_instance_segmentation->scores.reserve(segmentation_msg->masks.size());
        for (size_t i = 0; i < segmentation_msg->masks.size(); ++i) {

            // MONO8: 它表示图像数据是单通道、每个像素点占用 8 位（1 字节）存储的灰度图像。
            cv_bridge::CvImagePtr cv_mask_image = cv_bridge::toCvCopy(segmentation_msg->masks[i], sensor_msgs::image_encodings::MONO8);
            semantic_instance_segmentation->masks.push_back(cv_mask_image->image.clone());
            semantic_instance_segmentation->labels.push_back(segmentation_msg->class_ids[i]);
            semantic_instance_segmentation->scores.push_back(segmentation_msg->scores[i]);

        }
    }

    void SemanticMappingNode::computeEdgeMap(cv::Mat &rescaled_depth,cv::Mat *depth_map, cv::Mat *normal_map, cv::Mat *edge_map) {

        //========================  通过归一化深度图，
        // depth_map : 点云数据
        *depth_map = cv::Mat::zeros(depth_camera_.getHeight(), depth_camera_.getWidth(), CV_32FC3);
        depth_segmenter_->computeDepthMap(rescaled_depth, depth_map);

        //=======================  计算法向量
        // normal_map : 法向量图
        *normal_map = cv::Mat::zeros(depth_map->size(), CV_32FC3);
        depth_segmenter_->computeNormalMap(*depth_map, normal_map);

        //=======================  计算连续图
        // 连续图：CV_32FC1 类型 depth_discontinuity_map的值非0即1
        cv::Mat discontinuity_map = cv::Mat::zeros(depth_camera_.getHeight(), depth_camera_.getWidth(), CV_32FC1);
        if (Parameter::DepthDiscontinuityMapParams::use_discontinuity) {
            depth_segmenter_->computeDepthDiscontinuityMap(rescaled_depth, &discontinuity_map);
        }

        //====================== 计算最小凸性图
        cv::Mat convexity_map = cv::Mat::zeros(depth_camera_.getHeight(), depth_camera_.getWidth(), CV_32FC1);
        if (Parameter::MinConvexityMapParams::use_min_convexity) {
            depth_segmenter_->computeMinConvexityMap(*depth_map, *normal_map, &convexity_map);
        }

        //======================== 计算最小边图
        *edge_map = cv::Mat::zeros(depth_camera_.getHeight(), depth_camera_.getWidth(), CV_32FC1);
        depth_segmenter_->computeFinalEdgeMap(convexity_map, discontinuity_map, edge_map);

    }

    void SemanticMappingNode::dataHandleLoop(){
        while(1){

            SleepUtils::Sleep(50);

            sensor_msgs::msg::Image::ConstSharedPtr cur_depth_msg_ptr;
            segment_msg::msg::Result::ConstSharedPtr cur_result_msg_ptr;
            geometry_msgs::msg::PoseStamped::ConstSharedPtr cur_pose_msg_ptr;
            get_depth_segment_posestamped(cur_depth_msg_ptr,cur_result_msg_ptr,cur_pose_msg_ptr);

            if(cur_depth_msg_ptr == nullptr || cur_result_msg_ptr == nullptr ||cur_pose_msg_ptr ==nullptr || !camera_info_ready_){
                continue;
            }

            double cur_depth_time = TimeUtils::RosTimeToAbsTime(cur_depth_msg_ptr->header.stamp);
            double cur_segment_time = TimeUtils::RosTimeToAbsTime(cur_result_msg_ptr->header.stamp);
            double cur_pose_time = TimeUtils::RosTimeToAbsTime(cur_pose_msg_ptr->header.stamp);

            if (last_depth_msg_time_ > 0 && TWO_FLOAT_EQUAL(last_depth_msg_time_, cur_depth_time)) {
                continue;
            }
            if (last_segmentation_msg_time_ > 0 && TWO_FLOAT_EQUAL(last_segmentation_msg_time_, cur_segment_time)) {
                continue;
            }
            if(last_pose_stamped_msg_time_ > 0 && TWO_FLOAT_EQUAL(last_pose_stamped_msg_time_, cur_pose_time)){
                continue;
            }
            last_depth_msg_time_ = cur_depth_time;
            last_segmentation_msg_time_ = cur_segment_time;
            last_pose_stamped_msg_time_ = cur_pose_time;

            // =================  开始处理深度图和分割消息  ======================
            //解析分割消息
            depth_segmentation::SemanticInstanceSegmentation instance_segmentation;
            semanticInstanceSegmentationFromRosMsg(cur_result_msg_ptr, &instance_segmentation);

            //解析深度消息
            cv::Mat rescaled_depth;
            depthMatFromRosMsg(cur_depth_msg_ptr, &rescaled_depth);

            cv::Mat depth_map, normal_map, edge_map;
            auto start_time = TimeUtils::GetThreadCpuTimeSeconds();
            computeEdgeMap(rescaled_depth, &depth_map, &normal_map, &edge_map);
            auto end_time = TimeUtils::GetThreadCpuTimeSeconds();
            auto elapsed_time = end_time - start_time;
            std::cout<< "computeEdgeMap cost time " << elapsed_time <<std::endl;

            cv::Mat label_map(edge_map.size(), CV_32FC1);
            // 将edge_map非nan值的元素拷贝到remove_no_values，再赋值给edge_map
            cv::Mat remove_no_values = cv::Mat::zeros(edge_map.size(), edge_map.type());
            edge_map.copyTo(remove_no_values, rescaled_depth == rescaled_depth);
            edge_map = remove_no_values;

            // 进行结果与深度图融合操作
            std::vector<depth_segmentation::Segment> segments;
            std::vector<cv::Mat> segment_masks;

            start_time = TimeUtils::GetThreadCpuTimeSeconds();
            depth_segmenter_->LabelMap(instance_segmentation, depth_map, edge_map,
                                       normal_map, &label_map, &segment_masks,
                                       &segments);
            end_time = TimeUtils::GetThreadCpuTimeSeconds();
            elapsed_time = end_time - start_time;
            std::cout<< "LabelMap cost time " << elapsed_time <<std::endl;

            Localization::Pose2D odo_pose;
            PoseStampedFromRosMsg(cur_pose_msg_ptr,odo_pose);
            Localization::Pose3D odo_pose_3d = Localization::Pose3D(odo_pose._pos._x,odo_pose._pos._y,0,odo_pose._orient._deg,0,0);

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr semantic_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr instance_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());

            start_time = TimeUtils::GetThreadCpuTimeSeconds();
            Localization::Pose3D pose_mul_calib = odo_pose_3d * calib_odo_to_cam_;
            segmentationObjectReceive(segments, pose_mul_calib, semantic_cloud_ptr, instance_cloud_ptr);
            end_time = TimeUtils::GetThreadCpuTimeSeconds();
            elapsed_time = end_time - start_time;
            std::cout<< "Fusion cost time " << elapsed_time <<std::endl;

            sendPointCloud2(segmented_pointcloud_publisher_, semantic_cloud_ptr, cur_depth_msg_ptr->header);
            sendPointCloud2(instance_pointcloud_publisher_,  instance_cloud_ptr, cur_depth_msg_ptr->header);

        }
    }

    void SemanticMappingNode::sendPointCloud2(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& publisher,
                         const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_ptr,
                         const std_msgs::msg::Header& header){

        std::shared_ptr<sensor_msgs::msg::PointCloud2> pointcloud2_ptr = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(*cloud_ptr, *pointcloud2_ptr);
        pointcloud2_ptr->header = header;
        pointcloud2_ptr->header.frame_id = "map";
        publisher->publish(*pointcloud2_ptr);

    }

    void SemanticMappingNode::segmentationObjectReceive(const std::vector<depth_segmentation::Segment>& segments,
                                   const Localization::Pose3D& global_pose,
                                   pcl::PointCloud<pcl::PointXYZRGB>::Ptr& semantic_cloud_ptr,
                                   pcl::PointCloud<pcl::PointXYZRGB>::Ptr& instance_cloud_ptr){

        std::vector<std::shared_ptr<depth_segmentation::SegmentWithIdx>> segments_vec;
        for(const auto& segment : segments){
            std::shared_ptr<depth_segmentation::SegmentWithIdx> segment_with_idx(new depth_segmentation::SegmentWithIdx(segment,global_pose));
            segments_vec.push_back(segment_with_idx);
        }

        semantic_integrator_->integrateSegments(segments_vec,semantic_cloud_ptr,instance_cloud_ptr);
    }
}