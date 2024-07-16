#ifndef SAFETY_CHECKER_H
#define SAFETY_CHECKER_H

#include <iostream>

// 包含所有与控制相关的数据
#include "ControlFSMData.h"

/**
 * SafetyChecker处理ControlFSM请求的检查
 */
template<typename T>
class SafetyChecker
{
public:
    SafetyChecker(ControlFSMData<T> *dataIn) : data(dataIn)
    {};
    
    // 预检查以确保控制安全运行
    bool checkSafeOrientation();  // 机器人的方位可以安全控制
    
    // 事后检查以确保控制可以发送给机器人
    bool checkPDesFoot();          // 期望的脚位制不是太远
    bool checkForceFeedForward();  // 期望的前馈力不是太大
    
    // 存储来自ControlFSM的数据
    ControlFSMData<T> *data;

private:
};

#endif  // SAFETY_CHECKER_H