/*=========================== Balance Stand ===========================*/
/**
 * 强制所有腿着地，使用 QP Balance 控制器进行瞬时平衡控制
 */

#include "FSM_State_BalanceStand.h"
#include <Controllers/WBC_Ctrl/LocomotionCtrl/LocomotionCtrl.hpp>

/**
 * FSM状态的构造函数，它将特定于状态的信息传递给通用的FSM状态构造函数。
 *
 * @param _controlFSMData 保存所有相关的控制数据
 */
template<typename T>
FSM_State_BalanceStand<T>::FSM_State_BalanceStand(
        ControlFSMData<T> *_controlFSMData)
        : FSM_State<T>(_controlFSMData, FSM_StateName::BALANCE_STAND, "BALANCE_STAND")
{
    // 控制前安全检查
    this->turnOnAllSafetyChecks();
    // 关闭Foot pos命令，因为它在WBC中被设置为操作任务
    this->checkPDesFoot = false;
    
    
    // 初始化 GRF 为 0s
    this->footFeedForwardForces = Mat34<T>::Zero();
    
    _wbc_ctrl = new LocomotionCtrl<T>(_controlFSMData->_quadruped->buildModel());
    _wbc_data = new LocomotionCtrlData<T>();
    
    _wbc_ctrl->setFloatingBaseWeight(1000.);
}

template<typename T>
void FSM_State_BalanceStand<T>::onEnter()
{
    // 默认是不转换
    this->nextStateName = this->stateName;
    
    // 重置转换数据
    this->transitionData.zero();
    
    // 始终将步态设置为站立状态
    this->_data->_gaitScheduler->gaitData._nextGait = GaitType::STAND;
    
    _ini_body_pos = (this->_data->_stateEstimator->getResult()).position;
    
    if(_ini_body_pos[2] < 0.2)
    {
        _ini_body_pos[2] = 0.3;
    }
    
    last_height_command = _ini_body_pos[2];
    
    _ini_body_ori_rpy = (this->_data->_stateEstimator->getResult()).rpy;
    _body_weight = this->_data->_quadruped->_bodyMass * 9.81;
}

/**
 * 调用要在每个控制循环迭代上执行的函数
 */
template<typename T>
void FSM_State_BalanceStand<T>::run()
{
    Vec4<T> contactState;
    contactState << 0.5, 0.5, 0.5, 0.5;
    this->_data->_stateEstimator->setContactPhase(contactState);
    BalanceStandStep();
}

/**
 * 管理可以通过用户命令或状态事件触发器转换到哪些状态
 *
 * @return 要转换到的FSM枚举名称
 */
template<typename T>
FSM_StateName FSM_State_BalanceStand<T>::checkTransition()
{
    // Get the next state
    _iter++;
    
    // 切换FSM控制模式
    switch((int) this->_data->controlParameters->control_mode)
    {
        case K_BALANCE_STAND:
            break;
        
        case K_LOCOMOTION:
            // 请求转换到 LOCOMOTION 状态
            this->nextStateName = FSM_StateName::LOCOMOTION;
            
            // 立即转换
            this->transitionDuration = 0.0;
            
            // 在调度程序中设置下一个步态为
            this->_data->_gaitScheduler->gaitData._nextGait = GaitType::TROT;
            break;
        
        case K_PASSIVE:
            this->nextStateName = FSM_StateName::PASSIVE;
            // 立即转换
            this->transitionDuration = 0.0;
            
            break;
        
        case K_VISION:
            this->nextStateName = FSM_StateName::VISION;
            // 立即转换
            this->transitionDuration = 0.0;
            break;
        
        case K_RECOVERY_STAND:
            this->nextStateName = FSM_StateName::RECOVERY_STAND;
            // 立即转换
            this->transitionDuration = 0.0;
            break;
        
        case K_BACKFLIP:
            this->nextStateName = FSM_StateName::BACKFLIP;
            this->transitionDuration = 0.;
            break;
        
        default:
            std::cout << "[CONTROL FSM] Bad Request: Cannot transition from "
                      << K_BALANCE_STAND << " to "
                      << this->_data->controlParameters->control_mode << std::endl;
    }
    
    // 将下一个状态名返回给 FSM
    return this->nextStateName;
}

/**
 * 处理机器人在状态之间的实际转换
 *
 * @return true 如果转换完成
 */
template<typename T>
TransitionData<T> FSM_State_BalanceStand<T>::transition()
{
    // 切换FSM控制模式
    switch(this->nextStateName)
    {
        case FSM_StateName::LOCOMOTION:
            BalanceStandStep();
            
            _iter++;
            if(_iter >= this->transitionDuration * 1000)
            {
                this->transitionData.done = true;
            }
            else
            {
                this->transitionData.done = false;
            }
            
            break;
        
        case FSM_StateName::PASSIVE:
            this->turnOffAllSafetyChecks();
            this->transitionData.done = true;
            break;
        
        case FSM_StateName::RECOVERY_STAND:
            this->transitionData.done = true;
            break;
        
        case FSM_StateName::BACKFLIP:
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
void FSM_State_BalanceStand<T>::onExit()
{
    _iter = 0;
}

/**
 * 为每只脚计算腿部控制器leg controller的命令
 */
template<typename T>
void FSM_State_BalanceStand<T>::BalanceStandStep()
{
    
    _wbc_data->pBody_des = _ini_body_pos;
    _wbc_data->vBody_des.setZero();
    _wbc_data->aBody_des.setZero();
    
    _wbc_data->pBody_RPY_des = _ini_body_ori_rpy;
    if(this->_data->controlParameters->use_rc)
    {
        const rc_control_settings *rc_cmd = this->_data->_desiredStateCommand->rcCommand;
        // 方向
        _wbc_data->pBody_RPY_des[0] = rc_cmd->rpy_des[0] * 1.4;
        _wbc_data->pBody_RPY_des[1] = rc_cmd->rpy_des[1] * 0.46;
        _wbc_data->pBody_RPY_des[2] -= rc_cmd->rpy_des[2];
        
        // 高度
        _wbc_data->pBody_des[2] += 0.12 * rc_cmd->height_variation;
    }
    else
    {
        // 方向
        _wbc_data->pBody_RPY_des[0] = 0.6 * this->_data->_desiredStateCommand->gamepadCommand->leftStickAnalog[0];
        _wbc_data->pBody_RPY_des[1] = 0.6 * this->_data->_desiredStateCommand->gamepadCommand->rightStickAnalog[0];
        _wbc_data->pBody_RPY_des[2] -= this->_data->_desiredStateCommand->gamepadCommand->rightStickAnalog[1];
        
        // 高度
        _wbc_data->pBody_des[2] += 0.12 * this->_data->_desiredStateCommand->gamepadCommand->rightStickAnalog[0];

//        // 周期性上下动
//        _wbc_data->pBody_des[2] = 0.05 * sin(2 * (_iter / 500)) + 0.3;
    
    }
    _wbc_data->vBody_Ori_des.setZero();
    
    for(int i = 0; i < 4; ++i)
    {
        _wbc_data->pFoot_des[i].setZero();
        _wbc_data->vFoot_des[i].setZero();
        _wbc_data->aFoot_des[i].setZero();
        _wbc_data->Fr_des[i].setZero();
        _wbc_data->Fr_des[i][2] = _body_weight / 4.;
        _wbc_data->contact_state[i] = true;
    }
    
    if(this->_data->_desiredStateCommand->trigger_pressed)
    {
        _wbc_data->pBody_des[2] = 0.05;
        
        if(last_height_command - _wbc_data->pBody_des[2] > 0.001)
        {
            _wbc_data->pBody_des[2] = last_height_command - 0.001;
        }
    }
    last_height_command = _wbc_data->pBody_des[2];
    
    _wbc_ctrl->run(_wbc_data, *this->_data);
}

// template class FSM_State_BalanceStand<double>;
template
class FSM_State_BalanceStand<float>;
