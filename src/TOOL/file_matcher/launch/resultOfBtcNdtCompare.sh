#!/bin/bash
###
 # @Author: Jixuan Lee
 # @Date: 2025-02-27 16:05:10
 # @LastEditors: Jixuan Lee
 # @LastEditTime: 2025-02-27 16:07:16
 # @FilePath: /OnlineLTSlam/src/TOOL/file_matcher/launch/resultOfBtcNdtCompare.sh
 # @Description: 
 # @Logs: 
### 

# 定义目标目录和可执行文件名
TARGET_DIR="/home/jixuanlee/OnlineLTSlam/devel/lib/file_matcher"
EXECUTABLE="result_of_btc_ndt_compare"

# 进入目标目录
cd "$TARGET_DIR"

# 检查是否成功进入目录
if [ $? -eq 0 ]; then
    echo "Entered directory: $TARGET_DIR"
    echo "Starting executable: $EXECUTABLE"
    # 启动可执行文件
    ./"$EXECUTABLE"
else
    echo "Failed to enter directory: $TARGET_DIR"
    exit 1
fi
