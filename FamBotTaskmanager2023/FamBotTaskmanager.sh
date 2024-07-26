#!/bin/bash

source /opt/ros/humble/setup.bash
source /home/lz/FamBotTaskmanager2023/install/setup.bash

# 定义要运行的可执行文件路径
EXECUTABLE_PATH="/home/lz/FamBotTaskmanager2023/build/FamBotTaskmanager2023/FamBotTaskmanager"

# 循环启动可执行文件
while true; do
    $EXECUTABLE_PATH
    sleep 2  # 等待一些时间后重新启动
done

