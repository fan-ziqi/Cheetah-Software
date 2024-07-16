#ifndef FSM_STATE_PASSIVE_H
#define FSM_STATE_PASSIVE_H

#include "FSM_State.h"

/**
 *
 */
template<typename T>
class FSM_State_Passive : public FSM_State<T>
{
public:
    FSM_State_Passive(ControlFSMData<T> *_controlFSMData);
    
    // 进入一种状态时的行为
    void onEnter();
    
    // 运行状态的正常行为
    void run();
    
    // 检查任何转换触发器
    FSM_StateName checkTransition();
    
    // 管理特定于状态的转换
    TransitionData<T> transition();
    
    // 退出状态时要执行的行为
    void onExit();
    
    TransitionData<T> testTransition();

private:
    // Keep track of the control iterations
    int iter = 0;
};

#endif  // FSM_STATE_PASSIVE_H
