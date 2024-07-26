#include "FamBotTaskmanager.h"
#include <iostream>
#include <thread>
#include <chrono>

FamBotTaskmanager::FamBotTaskmanager(std::shared_ptr<VisionCommunication> vision_communication)
    : vision_communication_(vision_communication)
{
}

FamBotTaskmanager::~FamBotTaskmanager()
{
}

void FamBotTaskmanager::Initial(){

}

void FamBotTaskmanager::SyncTaskRun(){
    std::cout<<"------SyncTaskRun"<<std::endl;
    isTaskRun = true;
    try
    {
        while (isTaskRun)
        {
            rclcpp::spin_some(vision_communication_);
            std::cout<<"vision_communication_"<<std::endl;
            cv_msgs::msg::CVinstructions cmdCondition = vision_communication_->GetLatestCmdCondition();
            std::cout<<"cmdCondition:"<<cmdCondition.cmd<<std::endl;
            if(cmdCondition.cmd == 0 || cmdCondition.cmd == -1){
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                std::cout<<"The Caiwei sleep 10ms!"<<std::endl;
                continue;
            }


            if(cmdCondition.cmd == 1){
                task_WaterDispenserFront();
            }

            if(cmdCondition.cmd == 2){
                task_WaterDispenserFront_close();
            }

            if(cmdCondition.cmd == 3){
                task_GraspTransparentCup();
            }

            if(cmdCondition.cmd == 4){
                task_GraspTransparentCup_close();
            }

            if(cmdCondition.cmd == 5){
                task_WaterFetching();
            }

            if(cmdCondition.cmd == 6){
                task_WaterFetching_close();
            }

            if(cmdCondition.cmd == 7){
                task_OpenCloseDoor();
            }

            if(cmdCondition.cmd == 8){
                task_OpenCloseDoor_close();
            }

            if(cmdCondition.cmd == 9){
                task_OrganizeStorage();
            }

            if(cmdCondition.cmd == 10){
                task_OrganizeStorage_close();
            }

            if(cmdCondition.cmd == 11){
                task_Carpet();
            }

            if(cmdCondition.cmd == 12){
                task_Carpet_close();
            }
        }
        
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    
}


void FamBotTaskmanager::BeginTaskRun(){
    if(isTaskRun){
        std::cout<<"Other task is running"<<std::endl;
        return;
    }

    isTaskRun = true;
    if (pTaskThread)
    {
        pTaskThread->detach();
        delete pTaskThread;
    }

    pTaskThread = new std::thread([&]()
    {
        SyncTaskRun();
    }
    );
}

//饮水机正面任务
void FamBotTaskmanager::task_WaterDispenserFront(){
    std::cout<<"饮水机正面任务启动！"<<std::endl;
}

//透明水杯抓取任务
void FamBotTaskmanager::task_GraspTransparentCup(){
    std::cout<<"透明水杯抓取任务启动！"<<std::endl;
}

//接水任务
void FamBotTaskmanager::task_WaterFetching(){
    std::cout<<"接水任务启动！"<<std::endl;
}

//开关门任务
void FamBotTaskmanager::task_OpenCloseDoor(){
    std::cout<<"开关门任务启动！"<<std::endl;
}

//收纳任务
void FamBotTaskmanager::task_OrganizeStorage(){
    std::cout<<"收纳任务启动！"<<std::endl;
}

//地毯任务
void FamBotTaskmanager::task_Carpet(){
    std::cout<<"地毯任务启动！"<<std::endl;
}

//饮水机正面任务结束
void FamBotTaskmanager::task_WaterDispenserFront_close(){
    std::cout<<"饮水机正面任务结束！"<<std::endl;
}

//透明水杯抓取任务结束
void FamBotTaskmanager::task_GraspTransparentCup_close(){
    std::cout<<"透明水杯抓取任务结束！"<<std::endl;
}

//接水任务结束
void FamBotTaskmanager::task_WaterFetching_close(){
    std::cout<<"接水任务结束！"<<std::endl;
}

//开关门任务结束
void FamBotTaskmanager::task_OpenCloseDoor_close(){
    std::cout<<"开关门任务结束！"<<std::endl;
}

//收纳任务结束
void FamBotTaskmanager::task_OrganizeStorage_close(){
    std::cout<<"收纳任务结束！"<<std::endl;
}

//地毯任务结束
void FamBotTaskmanager::task_Carpet_close(){
    std::cout<<"地毯任务结束！"<<std::endl;
}