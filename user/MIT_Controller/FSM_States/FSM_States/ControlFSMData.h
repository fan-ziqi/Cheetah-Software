#ifndef CONTROLFSMDATA_H
#define CONTROLFSMDATA_H

#include <ControlParameters/RobotParameters.h>
#include <MIT_UserParameters.h>
#include "Controllers/DesiredStateCommand.h"
#include "Controllers/GaitScheduler.h"
#include "Controllers/LegController.h"
#include "Controllers/StateEstimatorContainer.h"
#include "Dynamics/Quadruped.h"

/**
 *
 */
template<typename T>
struct ControlFSMData
{
    // EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // 机器人相关数据
    Quadruped<T> *_quadruped;
    // 状态估计容器
    StateEstimatorContainer<T> *_stateEstimator;
    // 腿部控制器
    LegController<T> *_legController;
    // 步态计划
    GaitScheduler<T> *_gaitScheduler;
    // 期望状态命令
    DesiredStateCommand<T> *_desiredStateCommand;
    // 控制参数
    RobotControlParameters *controlParameters;
    // 用户配置参数
    MIT_UserParameters *userParameters;
    // 可视化数据
    VisualizationData *visualizationData;
};

template
struct ControlFSMData<double>;
template
struct ControlFSMData<float>;

#endif  // CONTROLFSM_H