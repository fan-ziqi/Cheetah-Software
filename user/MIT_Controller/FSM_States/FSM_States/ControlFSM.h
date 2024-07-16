#ifndef CONTROLFSM_H
#define CONTROLFSM_H

#include <iostream>

// 包含所有与控制相关的数据
#include "ControlFSMData.h"

// 检查机器人状态和安全命令
#include "SafetyChecker.h"

// 一些FSM状态
#include "../FSM_States/FSM_State.h"
#include "../FSM_States/FSM_State_BalanceStand.h"
#include "../FSM_States/FSM_State_ImpedanceControl.h"
#include "../FSM_States/FSM_State_JointPD.h"
#include "../FSM_States/FSM_State_Locomotion.h"
#include "../FSM_States/FSM_State_Passive.h"
#include "../FSM_States/FSM_State_StandUp.h"
#include "../FSM_States/FSM_State_RecoveryStand.h"
#include "../FSM_States/FSM_State_Vision.h"
#include "../FSM_States/FSM_State_BackFlip.h"
#include "../FSM_States/FSM_State_FrontJump.h"

/**
 * 枚举所有操作模式
 */
enum class FSM_OperatingMode
{
    NORMAL, TRANSITIONING, ESTOP, EDAMP
};

/**
 *
 */
template<typename T>
struct FSM_StatesList
{
    FSM_State<T> *invalid;
    FSM_State_Passive<T> *passive;
    FSM_State_JointPD<T> *jointPD;
    FSM_State_ImpedanceControl<T> *impedanceControl;
    FSM_State_StandUp<T> *standUp;
    FSM_State_BalanceStand<T> *balanceStand;
    FSM_State_Locomotion<T> *locomotion;
    FSM_State_RecoveryStand<T> *recoveryStand;
//    FSM_State_Vision<T> *vision;
    FSM_State_BackFlip<T> *backflip;
    FSM_State_FrontJump<T> *frontJump;
};


/**
 *
 */
template<typename T>
struct FSM_ControllerList
{
};


/**
 * ControlFSM从一个更高的级别处理FSM状态
 */
template<typename T>
class ControlFSM
{
public:
    ControlFSM(Quadruped<T> *_quadruped,
               StateEstimatorContainer<T> *_stateEstimator,
               LegController<T> *_legController, GaitScheduler<T> *_gaitScheduler,
               DesiredStateCommand<T> *_desiredStateCommand,
               RobotControlParameters *controlParameters,
               VisualizationData *visualizationData,
               MIT_UserParameters *userParameters);
    
    // 初始化有限状态机ControlFSM
    void initialize();
    
    // 运行FSM逻辑，处理状态转换和正常运行
    void runFSM();
    
    // 这将被删除并放入SafetyChecker类
    FSM_OperatingMode safetyPreCheck();
    FSM_OperatingMode safetyPostCheck();
    
    // G从请求时创建的状态列表中获取下一个FSM_State
    FSM_State<T> *getNextState(FSM_StateName stateName);
    
    // 输出当前状态
    void printInfo(int opt);
    
    // 包含所有控制相关数据
    ControlFSMData<T> data;
    
    // FSM状态信息
    FSM_StatesList<T> statesList;  // 保存所有FSM状态
    FSM_State<T> *currentState;    // 当前FSM状态
    FSM_State<T> *nextState;       // 下一个FSM状态
    FSM_StateName nextStateName;   // 下一个FSM状态名
    
    // 检查所有输入和命令是否安全
    SafetyChecker<T> *safetyChecker;
    
    TransitionData<T> transitionData;

private:
    // FSM的工作模式
    FSM_OperatingMode operatingMode;
    
    // 选择每N次迭代打印信息的频率，模拟时间N*(0.001s)
    int printNum = 10000;
    
    // 跟踪自上次信息打印以来的迭代次数，自增到大于printNum时不输出
    int printIter = 0;
    
    int iter = 0;
    
    //lcm::LCM state_estimator_lcm;
    //state_estimator_lcmt _state_estimator;
};

#endif  // CONTROLFSM_H
