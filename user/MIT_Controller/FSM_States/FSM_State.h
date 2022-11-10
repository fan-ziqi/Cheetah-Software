#ifndef FSM_State_H
#define FSM_State_H

#include <stdio.h>

#include "ControlFSMData.h"
#include "TransitionData.h"
#include "Controllers/GaitScheduler.h"

#include <Controllers/BalanceController/BalanceController.hpp>

// 正常机器人状态
#define K_PASSIVE 0
#define K_STAND_UP 1
#define K_BALANCE_STAND 3
#define K_LOCOMOTION 4
#define K_LOCOMOTION_TEST 5
#define K_RECOVERY_STAND 6
#define K_VISION 8
#define K_BACKFLIP 9
#define K_FRONTJUMP 11

// 特定控制状态
#define K_JOINT_PD 51
#define K_IMPEDANCE_CONTROL 52

#define K_INVALID 100

/**
 * 枚举所有FSM状态，以便我们可以跟踪它们
 */
enum class FSM_StateName
{
    INVALID,
    PASSIVE,
    JOINT_PD,
    IMPEDANCE_CONTROL,
    STAND_UP,
    BALANCE_STAND,
    LOCOMOTION,
    RECOVERY_STAND,
    VISION,
    BACKFLIP,
    FRONTJUMP
};

/**
 * 状态机状态父类
 * 目前看到的状态，大部分参数没有用到
 */
template<typename T>
class FSM_State
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    // 所有状态的通用构造函数
    FSM_State(ControlFSMData<T> *_controlFSMData, FSM_StateName stateNameIn,
              std::string stateStringIn);
    
    // 进入一种状态时要执行的行为
    virtual void onEnter() = 0;// {}
    
    // 运行状态的正常行为
    virtual void run() = 0; //{}
    
    // 管理具体状态的转换
    virtual FSM_StateName checkTransition()
    { return FSM_StateName::INVALID; }
    
    // 运行转换行为，并在完成转换时返回true
    virtual TransitionData<T> transition()
    { return transitionData; }
    
    // 退出状态时要执行的行为
    virtual void onExit() = 0; // {}
    
    // 关节PD控制
    void jointPDControl(int leg, Vec3<T> qDes, Vec3<T> qdDes);
    // 笛卡尔阻抗控制
    void cartesianImpedanceControl(int leg, Vec3<T> pDes, Vec3<T> vDes,
                                   Vec3<double> kp_cartesian,
                                   Vec3<double> kd_cartesian);
    // 脚踏启发式布置（未启用）
    void footstepHeuristicPlacement(int leg);
    
    // 运行控制器
    void runControls();
    void runBalanceController();
    void runWholeBodyController();
    void runConvexModelPredictiveController();
    void runRegularizedPredictiveController();
    
    // 参数安全检测 置真/假
    void turnOnAllSafetyChecks();
    void turnOffAllSafetyChecks();
    
    // 保存所有相关的控制数据
    ControlFSMData<T> *_data;
    
    // FSM状态信息
    FSM_StateName stateName;      // 当前状态的枚举名称
    FSM_StateName nextStateName;  // 下一状态的枚举名称
    std::string stateString;      // 状态名
    
    // 状态转换参数
    T transitionDuration;  // 转换持续时间
    T tStartTransition;    // 转换开始时间
    TransitionData<T> transitionData;
    
    // 预先控制安全检查
    bool checkSafeOrientation = false;  // 检查 roll 和 pitch
    
    // 控制后安全检查
    bool checkPDesFoot = false;          // 不要控制脚走得太远
    bool checkForceFeedForward = false;  // 不要控制很大的力
    bool checkLegSingularity = false;    // 不用腿
    
    // 整个机器人的腿部控制器命令占位符(3x4矩阵)
    Mat34<T> jointFeedForwardTorques;  // 前馈关节力矩
    Mat34<T> jointPositions;           // 关节角度
    Mat34<T> jointVelocities;          // 关节角速度
    Mat34<T> footFeedForwardForces;    // 前馈力
    Mat34<T> footPositions;            // 足端位置
    Mat34<T> footVelocities;           // 足端速度
    
    // 下一步的脚步位置
    Mat34<T> footstepLocations;
    
    // 高级机器人身体控制器
    BalanceController balanceController;
    // ModelPredictiveController cMPC
    // RegularizedPredictiveController RPC

private:
    // 创建直角坐标P增益矩阵
    Mat3<float> kpMat;
    
    // 创建直角坐标D增益矩阵
    Mat3<float> kdMat;
};

#endif  // FSM_State_H
