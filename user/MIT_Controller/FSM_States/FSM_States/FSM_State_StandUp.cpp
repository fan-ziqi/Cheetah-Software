/*============================= Stand Up ==============================*/
/**
 * Transitionary state that is called for the robot to stand up into
 * balance control mode.
 */

#include "FSM_State_StandUp.h"

/**
 * @ * FSM状态的构造函数，它将特定于状态的信息传递给通用的FSM状态构造函数。
 *
 * @param _controlFSMData 保存所有相关的控制数据
 */
template<typename T>
FSM_State_StandUp<T>::FSM_State_StandUp(ControlFSMData<T> *_controlFSMData)
        : FSM_State<T>(_controlFSMData, FSM_StateName::STAND_UP, "STAND_UP"),
          _ini_foot_pos(4)
{
    // Do nothing
    // Set the pre controls safety checks
    this->checkSafeOrientation = false;
    
    // Post control safety checks
    this->checkPDesFoot = false;
    this->checkForceFeedForward = false;
}

template<typename T>
void FSM_State_StandUp<T>::onEnter()
{
    // Default is to not transition
    this->nextStateName = this->stateName;
    
    // Reset the transition data
    this->transitionData.zero();
    
    // Reset iteration counter
    iter = 0;
    
    for(size_t leg(0); leg < 4; ++leg)
    {
        _ini_foot_pos[leg] = this->_data->_legController->datas[leg].p;
    }
}

/**
 * 调用要在每个控制循环迭代上执行的函数
 */
template<typename T>
void FSM_State_StandUp<T>::run()
{
    
    if(this->_data->_quadruped->_robotType == RobotType::MINI_CHEETAH)
    {
        T hMax = 0.25;
        T progress = 2 * iter * this->_data->controlParameters->controller_dt;
        
        if(progress > 1.)
        { progress = 1.; }
        
        for(int i = 0; i < 4; i++)
        {
            this->_data->_legController->commands[i].kpCartesian = Vec3<T>(500, 500, 500).asDiagonal();
            this->_data->_legController->commands[i].kdCartesian = Vec3<T>(8, 8, 8).asDiagonal();
            
            this->_data->_legController->commands[i].pDes = _ini_foot_pos[i];
            this->_data->_legController->commands[i].pDes[2] =
                    progress * (-hMax) + (1. - progress) * _ini_foot_pos[i][2];
        }
    }
}

/**
 * 管理可以通过用户命令或状态事件触发器转换到哪些状态
 *
 * @return 要转换到的FSM枚举名称
 */
template<typename T>
FSM_StateName FSM_State_StandUp<T>::checkTransition()
{
    this->nextStateName = this->stateName;
    iter++;
    
    // 切换FSM控制模式
    switch((int) this->_data->controlParameters->control_mode)
    {
        case K_STAND_UP:
            break;
        case K_BALANCE_STAND:
            this->nextStateName = FSM_StateName::BALANCE_STAND;
            break;
        
        case K_LOCOMOTION:
            this->nextStateName = FSM_StateName::LOCOMOTION;
            break;
        
        case K_VISION:
            this->nextStateName = FSM_StateName::VISION;
            break;
        
        
        case K_PASSIVE:  // normal c
            this->nextStateName = FSM_StateName::PASSIVE;
            break;
        
        default:
            std::cout << "[CONTROL FSM] Bad Request: Cannot transition from "
                      << K_PASSIVE << " to "
                      << this->_data->controlParameters->control_mode << std::endl;
    }
    
    // Get the next state
    return this->nextStateName;
}

/**
 * 处理机器人在状态之间的实际转换
 *
 * @return true 如果转换完成
 */
template<typename T>
TransitionData<T> FSM_State_StandUp<T>::transition()
{
    // Finish Transition
    switch(this->nextStateName)
    {
        case FSM_StateName::PASSIVE:  // normal
            this->transitionData.done = true;
            break;
        
        case FSM_StateName::BALANCE_STAND:
            this->transitionData.done = true;
            break;
        
        case FSM_StateName::LOCOMOTION:
            this->transitionData.done = true;
            break;
        
        case FSM_StateName::VISION:
            this->transitionData.done = true;
            break;
        
        
        default:
            std::cout << "[CONTROL FSM] Something went wrong in transition"
                      << std::endl;
    }
    
    // 向FSM返回转换数据
    return this->transitionData;
}

/**
 * 清除退出状态时的状态信息
 */
template<typename T>
void FSM_State_StandUp<T>::onExit()
{
    // Nothing to clean up when exiting
}

// template class FSM_State_StandUp<double>;
template
class FSM_State_StandUp<float>;
