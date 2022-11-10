/*============================== Passive ==============================*/
/**
 * FSM状态无需控制。意味着一个安全状态，机器人不应该做任何事情，因为所有命令都将设置为0。
 */

#include "FSM_State_Passive.h"

/**
 * FSM状态的构造函数，该构造函数将状态特定信息传递给通用FSM状态构造函数
 *
 * @param _controlFSMData 保存所有相关控制数据
 */
template<typename T>
FSM_State_Passive<T>::FSM_State_Passive(ControlFSMData<T> *_controlFSMData)
        : FSM_State<T>(_controlFSMData, FSM_StateName::PASSIVE, "PASSIVE")
{
    // 啥也不做
    // 设置预控制安全检查
    this->checkSafeOrientation = false;
    
    // 控制后安全检查
    this->checkPDesFoot = false;
    this->checkForceFeedForward = false;
}

template<typename T>
void FSM_State_Passive<T>::onEnter()
{
    // 默认是不转换
    this->nextStateName = this->stateName;
    
    // 重置转换数据
    this->transitionData.zero();
}

/**
 * 调用要在每个控制循环迭代中执行的函数
 */
template<typename T>
void FSM_State_Passive<T>::run()
{
    // 什么都不做，所有的命令都应该以0开头
    testTransition();
}

/**
 * 处理机器人在状态之间的实际转换。
 *
 * @return 转换完成返回true
 */
template<typename T>
TransitionData<T> FSM_State_Passive<T>::testTransition()
{
    this->transitionData.done = true;
    return this->transitionData;
}

/**
 * 管理可以通过用户命令或状态事件触发器转换为哪些状态
 *
 * @return 要转换到的枚举FSM状态名称
 */
template<typename T>
FSM_StateName FSM_State_Passive<T>::checkTransition()
{
    this->nextStateName = this->stateName;
    iter++;
    
    // 切换FSM控制模式
    switch((int) this->_data->controlParameters->control_mode)
    {
        case K_PASSIVE:  // normal c (0)
            // 基于状态的转换的正常操作
            break;
        
        case K_JOINT_PD:
            // 请求切换到关节 PD 控制
            this->nextStateName = FSM_StateName::JOINT_PD;
            break;
        
        case K_STAND_UP:
            this->nextStateName = FSM_StateName::STAND_UP;
            break;
        
        case K_RECOVERY_STAND:
            this->nextStateName = FSM_StateName::RECOVERY_STAND;
            break;
        
        default:
            std::cout << "[CONTROL FSM] Bad Request: Cannot transition from "
                      << K_PASSIVE << " to "
                      << this->_data->controlParameters->control_mode << std::endl;
    }
    
    // 得到下一个状态
    return this->nextStateName;
}

/**
 * 处理机器人在状态之间的实际转换。
 *
 * @return 转换完成返回true
 */
template<typename T>
TransitionData<T> FSM_State_Passive<T>::transition()
{
    // 完成转换
    this->transitionData.done = true;
    
    // 将转换数据返回到 FSM
    return this->transitionData;
}

/**
 * 清理退出状态时的状态信息
 */
template<typename T>
void FSM_State_Passive<T>::onExit()
{
    // 退出时无需清理
}

// template class FSM_State_Passive<double>;
template
class FSM_State_Passive<float>;
