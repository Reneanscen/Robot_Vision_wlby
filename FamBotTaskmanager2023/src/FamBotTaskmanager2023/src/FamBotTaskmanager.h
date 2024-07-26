#pragma once

#include "VisionCommunication.h"

#include "rclcpp/rclcpp.hpp"

#include <thread>

#include "cv_msgs/msg/c_vinstructions.hpp"

class FamBotTaskmanager
{
private:
    bool isTaskRun;
    std::shared_ptr<VisionCommunication> vision_communication_;
    std::thread* pTaskThread;
public:
    FamBotTaskmanager(std::shared_ptr<VisionCommunication> vision_communication);
    ~FamBotTaskmanager();

    void Initial();

	//同步执行(阻塞式)
	void SyncTaskRun();

	//异步执行(非阻塞式)
	void BeginTaskRun();

	//结束执行
	void EndTaskRun();

	//异步执行,等待线程结束
	void Join();



    //饮水机正面任务
    void task_WaterDispenserFront();

    //透明水杯抓取任务
    void task_GraspTransparentCup();

    //接水任务
    void task_WaterFetching();

    //开关门任务
    void task_OpenCloseDoor();

    //收纳任务
    void task_OrganizeStorage();

    //地毯任务
    void task_Carpet();



    //饮水机正面任务结束
    void task_WaterDispenserFront_close();

    //透明水杯抓取任务结束
    void task_GraspTransparentCup_close();

    //接水任务结束
    void task_WaterFetching_close();

    //开关门任务结束
    void task_OpenCloseDoor_close();

    //收纳任务结束
    void task_OrganizeStorage_close();

    //地毯任务结束
    void task_Carpet_close();
};

