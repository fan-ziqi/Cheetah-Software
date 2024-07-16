#ifndef FSM_STATE_LOCOMOTION_H
#define FSM_STATE_LOCOMOTION_H

#include <Controllers/convexMPC/ConvexMPCLocomotion.h>
#include "FSM_State.h"

template<typename T>
class WBC_Ctrl;

template<typename T>
class LocomotionCtrlData;

/**
 *
 */
template<typename T>
class FSM_State_Locomotion : public FSM_State<T>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    FSM_State_Locomotion(ControlFSMData<T> *_controlFSMData);
    
    // 进入状态时要执行的行为
    void onEnter();
    
    // 运行状态的正常行为
    void run();
    
    // 检查任何转换触发器
    FSM_StateName checkTransition();
    
    // 管理特定于状态的转换
    TransitionData<T> transition();
    
    // 退出状态时要执行的行为
    void onExit();

private:
    // 跟踪控制迭代
    int iter = 0;
    // MPC控制器
    ConvexMPCLocomotion *cMPCOld;
    // WBC控制器
    WBC_Ctrl<T> *_wbc_ctrl;
    // 移动控制器数据
    LocomotionCtrlData<T> *_wbc_data;
    
    // 通过调用适当的平衡控制器并解析每个站立或摆动腿的结果，计算每个脚的腿部控制器的命令。
    void LocomotionControlStep();
    
    bool locomotionSafe();
    
    // 运动过程中支撑腿的阻抗控制
    void StanceLegImpedanceControl(int leg);
};

#endif  // FSM_STATE_LOCOMOTION_H
