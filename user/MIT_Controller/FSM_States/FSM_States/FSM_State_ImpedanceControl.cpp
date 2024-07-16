/*========================= Impedance Control =========================*/
/**
 * FSM State that allows PD Impedance control in cartesian space for
 * each of the legs.
 */

#include "FSM_State_ImpedanceControl.h"

/**
 * Constructor for the FSM State that passes in state specific info to
 * the generif FSM State constructor.
 *
 * @param _controlFSMData 保存所有相关的控制数据
 */
template<typename T>
FSM_State_ImpedanceControl<T>::FSM_State_ImpedanceControl(
        ControlFSMData<T> *_controlFSMData)
        : FSM_State<T>(_controlFSMData, FSM_StateName::IMPEDANCE_CONTROL,
                       "IMPEDANCE_CONTROL")
{
    // Do nothing here yet
}

template<typename T>
void FSM_State_ImpedanceControl<T>::onEnter()
{
    // Default is to not transition
    this->nextStateName = this->stateName;
    
    // Reset the transition data
    this->transitionData.zero();
}

/**
 * 调用要在每个控制循环迭代上执行的函数
 */
template<typename T>
void FSM_State_ImpedanceControl<T>::run()
{
    // Do nothing, all commands should begin as zeros
}

/**
 * 管理可以通过用户命令或状态事件触发器转换到哪些状态
 *
 * @return 要转换到的FSM枚举名称
 */
template<typename T>
FSM_StateName FSM_State_ImpedanceControl<T>::checkTransition()
{
    // Get the next state
    // 切换FSM控制模式
    switch((int) this->_data->controlParameters->control_mode)
    {
        case K_IMPEDANCE_CONTROL:
            // Normal operation for state based transitions
            break;
        
        case K_BALANCE_STAND:
            // Requested change to balance stand
            this->nextStateName = FSM_StateName::BALANCE_STAND;
            
            // 立即转换
            this->transitionData.tDuration = 0.0;
            
            // 在调度程序中设置下一个步态为
            this->_data->_gaitScheduler->gaitData._nextGait = GaitType::STAND;
            break;
        
        default:
            std::cout << "[CONTROL FSM] Bad Request: Cannot transition from " << 0
                      << " to " << this->_data->controlParameters->control_mode
                      << std::endl;
    }
    
    return this->nextStateName;
}

/**
 * 处理机器人在状态之间的实际转换
 *
 * @return true 如果转换完成
 */
template<typename T>
TransitionData<T> FSM_State_ImpedanceControl<T>::transition()
{
    this->transitionData.done = true;
    
    // 向FSM返回转换数据
    return this->transitionData;
}

/**
 * 清除退出状态时的状态信息
 */
template<typename T>
void FSM_State_ImpedanceControl<T>::onExit()
{
    // Nothing to clean up when exiting
}

// template class FSM_State_ImpedanceControl<double>;
template
class FSM_State_ImpedanceControl<float>;