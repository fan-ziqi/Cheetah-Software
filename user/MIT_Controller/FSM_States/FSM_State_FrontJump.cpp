/*============================= Recovery Stand ==============================*/
/**
 * Transitionary state that is called for the robot to stand up into
 * balance control mode.
 */

#include "FSM_State_FrontJump.h"
#include <Utilities/Utilities_print.h>

/**
 * @ * FSM状态的构造函数，它将特定于状态的信息传递给通用的FSM状态构造函数。
 *
 * @param _controlFSMData 保存所有相关的控制数据
 */
template<typename T>
FSM_State_FrontJump<T>::FSM_State_FrontJump(ControlFSMData<T> *_controlFSMData)
        : FSM_State<T>(_controlFSMData, FSM_StateName::STAND_UP, "STAND_UP")
{
    // Do nothing
    // Set the pre controls safety checks
    this->checkSafeOrientation = false;
    
    // Post control safety checks
    this->checkPDesFoot = false;
    this->checkForceFeedForward = false;
    
    zero_vec3.setZero();
    f_ff << 0.f, 0.f, -25.f;
    
    _data_reader = new DataReader(this->_data->_quadruped->_robotType, FSM_StateName::FRONTJUMP);
    
    front_jump_ctrl_ = new FrontJumpCtrl<T>(_data_reader, this->_data->controlParameters->controller_dt);
    front_jump_ctrl_->SetParameter();
    
}


template<typename T>
void FSM_State_FrontJump<T>::onEnter()
{
    // Default is to not transition
    this->nextStateName = this->stateName;
    
    // Reset the transition data
    this->transitionData.zero();
    
    // Reset iteration counter
    iter = 0;
    _state_iter = 0;
    _count = 0;
    _curr_time = 0;
    _motion_start_iter = 0;
    _b_first_visit = true;
    
    // initial configuration, position
    for(size_t i(0); i < 4; ++i)
    {
        initial_jpos[i] = this->_data->_legController->datas[i].q;
    }
    front_jump_ctrl_->SetParameter();
}

/**
 * 调用要在每个控制循环迭代上执行的函数
 */
template<typename T>
void FSM_State_FrontJump<T>::run()
{

// Command Computation
    if(_b_running)
    {
        if(!_Initialization())
        {
            ComputeCommand();
        }
    }
    else
    {
        _SafeCommand();
    }
    
    ++_count;
    _curr_time += this->_data->controlParameters->controller_dt;
    
}


template<typename T>
bool FSM_State_FrontJump<T>::_Initialization()
{ // do away with this?
    static bool test_initialized(false);
    if(!test_initialized)
    {
        test_initialized = true;
        printf("[Cheetah Test] Test initialization is done\n");
    }
    if(_count < _waiting_count)
    {
        for(int leg = 0; leg < 4; ++leg)
        {
            this->_data->_legController->commands[leg].qDes = initial_jpos[leg];
            for(int jidx = 0; jidx < 3; ++jidx)
            {
                this->_data->_legController->commands[leg].tauFeedForward[jidx] = 0.;
                this->_data->_legController->commands[leg].qdDes[jidx] = 0.;
                this->_data->_legController->commands[leg].kpJoint(jidx, jidx) = 20.;
                this->_data->_legController->commands[leg].kdJoint(jidx, jidx) = 2.;
            }
        }
        return true;
    }
    
    return false;
}

template<typename T>
void FSM_State_FrontJump<T>::ComputeCommand()
{
    if(_b_first_visit)
    {
        front_jump_ctrl_->FirstVisit(_curr_time);
        _b_first_visit = false;
    }
    
    if(this->_data->controlParameters->use_rc)
    {
        if(this->_data->_desiredStateCommand->rcCommand->mode == RC_mode::BACKFLIP_PRE)
        {
            front_jump_ctrl_->OneStep(_curr_time, true, this->_data->_legController->commands);
        }
        else
        {
            front_jump_ctrl_->OneStep(_curr_time, false, this->_data->_legController->commands);
        }
        
    }
    else
    {
        front_jump_ctrl_->OneStep(_curr_time, false, this->_data->_legController->commands);
    }
    
    if(front_jump_ctrl_->EndOfPhase(this->_data->_legController->datas))
    {
        front_jump_ctrl_->LastVisit();
    }
}

template<typename T>
void FSM_State_FrontJump<T>::_SafeCommand()
{
    for(int leg = 0; leg < 4; ++leg)
    {
        for(int jidx = 0; jidx < 3; ++jidx)
        {
            this->_data->_legController->commands[leg].tauFeedForward[jidx] = 0.;
            this->_data->_legController->commands[leg].qDes[jidx] = this->_data->_legController->datas[leg].q[jidx];
            this->_data->_legController->commands[leg].qdDes[jidx] = 0.;
        }
    }
}


template<typename T>
void FSM_State_FrontJump<T>::_SetJPosInterPts(
        const size_t &curr_iter, size_t max_iter, int leg,
        const Vec3<T> &ini, const Vec3<T> &fin)
{
    
    float a(0.f);
    float b(1.f);
    
    // if we're done interpolating
    if(curr_iter <= max_iter)
    {
        b = (float) curr_iter / (float) max_iter;
        a = 1.f - b;
    }
    
    // compute setpoints
    Vec3<T> inter_pos = a * ini + b * fin;
    
    // do control
    this->jointPDControl(leg, inter_pos, zero_vec3);
    
    //if(curr_iter == 0){ 
    //printf("flag:%d, curr iter: %lu, state iter: %llu, motion start iter: %d\n",
    //_flag, curr_iter, _state_iter, _motion_start_iter);
    //printf("inter pos: %f, %f, %f\n", inter_pos[0], inter_pos[1], inter_pos[2]);
    //}
    //if(curr_iter == max_iter){ 
    //printf("flag:%d, curr iter: %lu, state iter: %llu, motion start iter: %d\n",
    //_flag, curr_iter, _state_iter, _motion_start_iter);
    //printf("inter pos: %f, %f, %f\n", inter_pos[0], inter_pos[1], inter_pos[2]);
    //}
}

/**
 * 管理可以通过用户命令或状态事件触发器转换到哪些状态
 *
 * @return 要转换到的FSM枚举名称
 */
template<typename T>
FSM_StateName FSM_State_FrontJump<T>::checkTransition()
{
    this->nextStateName = this->stateName;
    iter++;
    
    // 切换FSM控制模式
    switch((int) this->_data->controlParameters->control_mode)
    {
        case K_FRONTJUMP:
            break;
        
        case K_RECOVERY_STAND:
            this->nextStateName = FSM_StateName::RECOVERY_STAND;
            break;
        
        case K_LOCOMOTION:
            this->nextStateName = FSM_StateName::LOCOMOTION;
            break;
        
        case K_PASSIVE:  // normal c
            this->nextStateName = FSM_StateName::PASSIVE;
            break;
        
        case K_BALANCE_STAND:
            this->nextStateName = FSM_StateName::BALANCE_STAND;
            break;
        
        default:
            std::cout << "[CONTROL FSM] Bad Request: Cannot transition from "
                      << K_FRONTJUMP << " to "
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
TransitionData<T> FSM_State_FrontJump<T>::transition()
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
        
        case FSM_StateName::RECOVERY_STAND:
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
void FSM_State_FrontJump<T>::onExit()
{
    // nothing to clean up?
}

// template class FSM_State_FrontJump<double>;
template
class FSM_State_FrontJump<float>;
