#!/bin/bash
# 设置环境变量，使ROS节点能够找到所需的库
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$(rospack find pct_planner)/PCT_planner-main/planner/lib/3rdparty/gtsam-4.1.1/install/lib
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$(rospack find pct_planner)/PCT_planner-main/planner/lib
export PYTHONPATH=$PYTHONPATH:$(rospack find pct_planner)/PCT_planner-main/planner/lib