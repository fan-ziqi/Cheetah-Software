/*============================= Joint PD ==============================*/
/**
 * FSM State that allows PD control of the joints.
 */

#include "FSM_State_JointPD.h"
#include <Configuration.h>

/**
 * @ * FSM状态的构造函数，它将特定于状态的信息传递给通用的FSM状态构造函数。
 *
 * @param _controlFSMData 保存所有相关的控制数据
 */
template<typename T>
FSM_State_JointPD<T>::FSM_State_JointPD(ControlFSMData<T> *_controlFSMData)
        : FSM_State<T>(_controlFSMData, FSM_StateName::JOINT_PD, "JOINT_PD"),
          _ini_jpos(cheetah::num_act_joint)
{
    // Do nothing here yet
}

template<typename T>
void FSM_State_JointPD<T>::onEnter()
{
    // Default is to not transition
    this->nextStateName = this->stateName;
    
    // Reset the transition data
    this->transitionData.zero();
    
    // Reset counter
    iter = 0;
    
    for(size_t leg(0); leg < 4; ++leg)
    {
        for(size_t jidx(0); jidx < 3; ++jidx)
        {
            _ini_jpos[3 * leg + jidx] = FSM_State<T>::_data->_legController->datas[leg].q[jidx];
        }
    }
    
}

/**
 * 调用要在每个控制循环迭代上执行的函数
 */
template<typename T>
void FSM_State_JointPD<T>::run()
{
    // This is just a test, should be running whatever other code you want
    Vec3<T> qDes;
    qDes << 0, -1.052, 2.63;
    Vec3<T> qdDes;
    qdDes << 0, 0, 0;
    
    static double progress(0.);
    progress += this->_data->controlParameters->controller_dt;
    double movement_duration(3.0);
    double ratio = progress / movement_duration;
    if(ratio > 1.) ratio = 1.;
    
    this->jointPDControl(0, ratio * qDes + (1. - ratio) * _ini_jpos.head(3), qdDes);
    this->jointPDControl(1, ratio * qDes + (1. - ratio) * _ini_jpos.segment(3, 3), qdDes);
    this->jointPDControl(2, ratio * qDes + (1. - ratio) * _ini_jpos.segment(6, 3), qdDes);
    this->jointPDControl(3, ratio * qDes + (1. - ratio) * _ini_jpos.segment(9, 3), qdDes);
}

/**
 * 管理可以通过用户命令或状态事件触发器转换到哪些状态
 *
 * @return 要转换到的FSM枚举名称
 */
template<typename T>
FSM_StateName FSM_State_JointPD<T>::checkTransition()
{
    this->nextStateName = this->stateName;
    
    iter++;
    
    // 切换FSM控制模式
    switch((int) this->_data->controlParameters->control_mode)
    {
        case K_JOINT_PD:
            // Normal operation for state based transitions
            break;
        
        case K_IMPEDANCE_CONTROL:
            // Requested change to impedance control
            this->nextStateName = FSM_StateName::IMPEDANCE_CONTROL;
            
            // Transition time is 1 second
            this->transitionDuration = 1.0;
            break;
        
        case K_STAND_UP:
            // Requested change to impedance control
            this->nextStateName = FSM_StateName::STAND_UP;
            
            // 立即转换
            this->transitionDuration = 0.0;
            break;
        
        case K_BALANCE_STAND:
            // Requested change to balance stand
            this->nextStateName = FSM_StateName::BALANCE_STAND;
            break;
        
        case K_PASSIVE:
            // Requested change to BALANCE_STAND
            this->nextStateName = FSM_StateName::PASSIVE;
            
            // 立即转换
            this->transitionDuration = 0.0;
            
            break;
        
        default:
            std::cout << "[CONTROL FSM] Bad Request: Cannot transition from "
                      << K_JOINT_PD << " to "
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
TransitionData<T> FSM_State_JointPD<T>::transition()
{
    // 切换FSM控制模式
    switch(this->nextStateName)
    {
        case FSM_StateName::IMPEDANCE_CONTROL:
            
            iter++;
            if(iter >= this->transitionDuration * 1000)
            {
                this->transitionData.done = true;
            }
            else
            {
                this->transitionData.done = false;
            }
            break;
        
        case FSM_StateName::STAND_UP:
            this->transitionData.done = true;
            
            break;
        
        case FSM_StateName::BALANCE_STAND:
            this->transitionData.done = true;
            
            break;
        
        case FSM_StateName::PASSIVE:
            this->turnOffAllSafetyChecks();
            
            this->transitionData.done = true;
            
            break;
        
        default:
            std::cout << "[CONTROL FSM] Bad Request: Cannot transition from "
                      << K_JOINT_PD << " to "
                      << this->_data->controlParameters->control_mode << std::endl;
    }
    // Finish transition
    this->transitionData.done = true;
    
    // 向FSM返回转换数据
    return this->transitionData;
}

/**
 * 清除退出状态时的状态信息
 */
template<typename T>
void FSM_State_JointPD<T>::onExit()
{
    // Nothing to clean up when exiting
}

// template class FSM_State_JointPD<double>;
template
class FSM_State_JointPD<float>;
