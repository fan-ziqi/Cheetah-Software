/*============================= Recovery Stand ==============================*/
/**
 * Transitionary state that is called for the robot to stand up into
 * balance control mode.
 */

#include "FSM_State_RecoveryStand.h"
#include <Utilities/Utilities_print.h>


/**
 * @ * FSM状态的构造函数，它将特定于状态的信息传递给通用的FSM状态构造函数。
 *
 * @param _controlFSMData 保存所有相关的控制数据
 */
template<typename T>
FSM_State_RecoveryStand<T>::FSM_State_RecoveryStand(ControlFSMData<T> *_controlFSMData)
        : FSM_State<T>(_controlFSMData, FSM_StateName::RECOVERY_STAND, "recovery_stand")
{
    // 设置控制前的安全检查
    this->checkSafeOrientation = false;
    
    // 设置控制后安全检查
    this->checkPDesFoot = false;
    this->checkForceFeedForward = false;
    
    zero_vec3.setZero();
    for ( size_t i( 0 ); i < 4; ++i ) //RobotType _robotType
        if ( this->_data->_quadruped->_robotType == RobotType::MINI_CHEETAH )
            fold_jpos[ i ] << -0.0f, -1.4f, 2.4f;

    T L1           = 0.2;//this->data_->quadruped->_hipLinkLength; //_hipLinkLength, _kneeLinkLength
    T L2           = 0.217;//this->data_->quadruped->_kneeLinkLength;
    T com_offset_x = 0.01;
    T H            = L1 * cos( -0.69 ) + L2 * cos( 0.7 );  // cyberdog2: 0.24, cyberdog: 0.32
    T D            = std::sqrt( H * H + com_offset_x * com_offset_x );
    T alpha2       = 3.141592653589793 - std::acos( ( L1 * L1 + L2 * L2 - D * D ) / ( 2.0 * L1 * L2 ) );
    T alpha1       = std::atan( com_offset_x / H ) - std::acos( ( L1 * L1 + D * D - L2 * L2 ) / ( 2.0 * L1 * D ) );

        // Stand Up
    for ( size_t i( 0 ); i < 4; ++i ) {
        stand_jpos[ i ] << 0.f, alpha1, alpha2;
    }
    _flag = FoldLegs;


}

template<typename T>
void FSM_State_RecoveryStand<T>::onEnter()
{
    // Default is to not transition
    this->nextStateName = this->stateName;
    
    // Reset the transition data
    this->transitionData.zero();
    
    // Reset iteration counter
    iter = 0;
    _state_iter = 0;

    // initial configuration, position
    for(size_t i(0); i < 4; ++i)
    {
        initial_jpos[i] = this->_data->_legController->datas[i].q;
    }
    
    T body_height = this->_data->_stateEstimator->getResult().position[2];
    
    _flag = FoldLegs;
    
    if(!_UpsideDown())
    { // Proper orientation
        if((0.2 < body_height) && (body_height < 0.45))
        {
            printf("[Recovery Balance] body height is %f; Stand Up \n", body_height);
            _flag = StandUp;
        }
        else
        {
            printf("[Recovery Balance] body height is %f; Folding legs \n", body_height);
        }
    }
    else
    {
        printf("[Recovery Balance] UpsideDown (%d) \n", _UpsideDown());
    }
    _motion_start_iter = 0;
}

template<typename T>
bool FSM_State_RecoveryStand<T>::_UpsideDown()
{
//    pretty_print(this->_data->_stateEstimator->getResult().rBody, std::cout, "Rot");
    //if(this->_data->_stateEstimator->getResult().aBody[2] < 0){
    if(this->_data->_stateEstimator->getResult().rBody(2, 2) < 0)
    {
        return true;
    }
    return false;
}

/**
 * 调用要在每个控制循环迭代上执行的函数
 */
template<typename T>
void FSM_State_RecoveryStand<T>::run()
{
    
    switch(_flag)
    {
        case StandUp:
            _StandUp(_state_iter - _motion_start_iter);
            break;
        case FoldLegs:
            _FoldLegs(_state_iter - _motion_start_iter);
            break;
        case RollOver:
            _RollOver(_state_iter - _motion_start_iter);
            break;
    }
    
    ++_state_iter;
}

template<typename T>
void FSM_State_RecoveryStand<T>::_SetJPosInterPts(
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

//    if(curr_iter == 0)
//    {
//        printf("flag:%d, curr iter: %lu, state iter: %llu, motion start iter: %d\n",
//               _flag, curr_iter, _state_iter, _motion_start_iter);
//        printf("inter pos: %f, %f, %f\n", inter_pos[0], inter_pos[1], inter_pos[2]);
//    }
//    if(curr_iter == max_iter)
//    {
//        printf("flag:%d, curr iter: %lu, state iter: %llu, motion start iter: %d\n",
//               _flag, curr_iter, _state_iter, _motion_start_iter);
//        printf("inter pos: %f, %f, %f\n", inter_pos[0], inter_pos[1], inter_pos[2]);
//    }
}

template<typename T>
void FSM_State_RecoveryStand<T>::_RollOver(const int &curr_iter)
{
    if ( curr_iter == 1 )
        std::cout << "Start rollover to RecoveryStand" << std::endl;

    // leg motor limit angle
    // 3: 44.47 -46.9    -201.1  116.8    29.5~142.8
    // 1: 43.57 - 47.0   -251.1~ 72.8     28.5~143.7
    // leg motor limit angle for mini of new motor structrue
    // 1: -0.68~0.68(38.9)    -2.792（-159.9）~1.326(75.9)    0.523(29.96)~2.53(144.9)
    // 3: -0.68~0.68(38.9)    -3.141(-179.9)~0.977(55.9)    0.523(29.96)~2.53(144.9)
    rolling_jpos[ 0 ] << -45 / 57.3, -100 / 57.3, 140 / 57.3;
    rolling_jpos[ 1 ] << 0.0, -110 / 57.3, 140 / 57.3;
    rolling_jpos[ 2 ] << -45 / 57.3, -100 / 57.3, 140 / 57.3;
    rolling_jpos[ 3 ] << 0.0, -110 / 57.3, 140 / 57.3;
    
    if ( curr_iter > 0 && curr_iter <= 200 )
        for ( size_t i( 0 ); i < 4; ++i )
            _SetJPosInterPts( curr_iter - 0, 200, i, initial_jpos[ i ], rolling_jpos[ i ] );
    if ( curr_iter == 200 )
        for ( size_t i( 0 ); i < 4; ++i )
            initial_jpos[ i ] = rolling_jpos[ i ];
    // stage2:
    rolling_jpos[ 0 ] << -45 / 57.3, -100 / 57.3, 140 / 57.3;
    rolling_jpos[ 1 ] << 30 / 57.3, -180 / 57.3, 140 / 57.3;
    rolling_jpos[ 2 ] << -45 / 57.3, -100 / 57.3, 140 / 57.3;
    rolling_jpos[ 3 ] << 30 / 57.3, -180 / 57.3, 140 / 57.3;
    
    if ( curr_iter > 200 && curr_iter <= 500 )
        for ( size_t i( 0 ); i < 4; ++i )
            _SetJPosInterPts( curr_iter - 200, 300, i, initial_jpos[ i ], rolling_jpos[ i ] );
    if ( curr_iter == 500 )
        for ( size_t i( 0 ); i < 4; ++i )
            initial_jpos[ i ] = rolling_jpos[ i ];
    // stage3:
    rolling_jpos[ 0 ] << -45 / 57.3, -160 / 57.3, 120 / 57.3;
    rolling_jpos[ 1 ] << 30 / 57.3, -180 / 57.3, 140 / 57.3;
    rolling_jpos[ 2 ] << -45 / 57.3, -160 / 57.3, 120 / 57.3;
    rolling_jpos[ 3 ] << 30 / 57.3, -180 / 57.3, 140 / 57.3;

    if ( curr_iter > 500 && curr_iter <= 800 ) {
        for ( size_t i( 0 ); i < 4; ++i )
            _SetJPosInterPts( curr_iter - 500, 300, i, initial_jpos[ i ], rolling_jpos[ i ] );
    }
    if ( curr_iter == 800 )
        for ( size_t i( 0 ); i < 4; ++i )
            initial_jpos[ i ] = rolling_jpos[ i ];
    // stage4:
    rolling_jpos[ 0 ] << -45 / 57.3, -160 / 57.3, 120 / 57.3;
    rolling_jpos[ 1 ] << -10 / 57.3, -90 / 57.3, 120 / 57.3;
    rolling_jpos[ 2 ] << -45 / 57.3, -160 / 57.3, 120 / 57.3;
    rolling_jpos[ 3 ] << -10 / 57.3, -90 / 57.3, 120 / 57.3;

    if ( curr_iter > 800 && curr_iter <= 900 ) {
        for ( size_t i( 0 ); i < 4; ++i )
            _SetJPosInterPts( curr_iter - 800, 50, i, initial_jpos[ i ], rolling_jpos[ i ] );
    }
    if ( curr_iter == 900 )
        for ( size_t i( 0 ); i < 4; ++i )
            initial_jpos[ i ] = rolling_jpos[ i ];
    // stage5:
    rolling_jpos[ 0 ] << 42 / 57.3, -150 / 57.3, 140 / 57.3;
    rolling_jpos[ 1 ] << -10 / 57.3, -90 / 57.3, 120 / 57.3;
    rolling_jpos[ 2 ] << 42 / 57.3, -150 / 57.3, 140 / 57.3;
    rolling_jpos[ 3 ] << -10 / 57.3, -90 / 57.3, 120 / 57.3;
    if ( curr_iter > 900 && curr_iter <= 1100 ) {
        for ( size_t i( 0 ); i < 4; ++i ) {
            _SetJPosInterPts( curr_iter - 900, 200, i, initial_jpos[ i ], rolling_jpos[ i ] );
            if ( ( i == 1 || i == 3 ) && this->_data->_quadruped->_robotType != RobotType::CHEETAH_3 ) {
                this->_data->_legController->commands[ i ].kpJoint << 50, 0, 0, 0, 50, 0, 0, 0, 60;
                ;
                this->_data->_legController->commands[ i ].kdJoint << 4, 0, 0, 0, 4, 0, 0, 0, 4;
            }
        }
    }

    if ( curr_iter == 1100 )
        for ( size_t i( 0 ); i < 4; ++i )
            initial_jpos[ i ] = rolling_jpos[ i ];
    // stage6:
    rolling_jpos[ 0 ] << -0 / 57.3, -80 / 57.3, 140 / 57.3;
    rolling_jpos[ 1 ] << 0 / 57.3, -80 / 57.3, 140 / 57.3;
    rolling_jpos[ 2 ] << -0 / 57.3, -80 / 57.3, 140 / 57.3;
    rolling_jpos[ 3 ] << 0 / 57.3, -80 / 57.3, 140 / 57.3;

    if ( curr_iter > 1100 && curr_iter <= 1500 ) {
        for ( size_t i( 0 ); i < 4; ++i )
            _SetJPosInterPts( curr_iter - 1100, 200, i, initial_jpos[ i ], rolling_jpos[ i ] );
    }
    if ( curr_iter == 1500 )
        for ( size_t i( 0 ); i < 4; ++i )
            initial_jpos[ i ] = rolling_jpos[ i ];
    // stage7:
    if ( curr_iter >= 1500 ) {
        

        _flag = FoldLegs;
        for(size_t i(0); i < 4; ++i) initial_jpos[i] = rolling_jpos[i];
        _motion_start_iter = _state_iter + 1;
    }

}

template<typename T>
void FSM_State_RecoveryStand<T>::_StandUp(const int &curr_iter)
{
    T body_height = this->_data->_stateEstimator->getResult().position[2];//0.116
    
    bool something_wrong(false);
    
    if(_UpsideDown() || (body_height < 0.1))
    {
        something_wrong = true;
    }
    
    if((curr_iter > floor(standup_ramp_iter * 0.7)) && something_wrong)
    {
        // 如果由于某种原因，即使在站立动作几乎结束后，身体高度仍然过低
        // (当急停装置处于“其他”状态时，可能会发生这种情况)
        for(size_t i(0); i < 4; ++i)
        {
            initial_jpos[i] = this->_data->_legController->datas[i].q;
        }
        _flag = FoldLegs;
        _motion_start_iter = _state_iter + 1;
        
        printf("[Recovery Balance - Warning] body height is still too low (%f) or UpsideDown (%d); Folding legs \n",
               body_height, _UpsideDown());
    }
    else
    {
        for(size_t leg(0); leg < 4; ++leg)
        {
            _SetJPosInterPts(curr_iter, standup_ramp_iter,
                             leg, initial_jpos[leg], stand_jpos[leg]);
        }
    }
    // 机器人的前馈质量
    Vec4<T> se_contactState(0.5, 0.5, 0.5, 0.5);
    this->_data->_stateEstimator->setContactPhase(se_contactState);
    
}

template<typename T>
void FSM_State_RecoveryStand<T>::_FoldLegs(const int &curr_iter)
{
    for(size_t leg(0); leg < 4; ++leg)
    {
        _SetJPosInterPts(curr_iter, fold_ramp_iter, leg,
                         initial_jpos[leg], fold_jpos[leg]);
    }
    if(curr_iter >= fold_ramp_iter + fold_settle_iter)
    {
        if(_UpsideDown())
        {
            _flag = RollOver;
            for(size_t i(0); i < 4; ++i) initial_jpos[i] = fold_jpos[i];
        }
        else
        {
            _flag = StandUp;
            for(size_t i(0); i < 4; ++i) initial_jpos[i] = fold_jpos[i];
        }
        _motion_start_iter = _state_iter + 1;
    }
}

/**
 * 管理可以通过用户命令或状态事件触发器转换到哪些状态
 *
 * @return 要转换到的FSM枚举名称
 */
template<typename T>
FSM_StateName FSM_State_RecoveryStand<T>::checkTransition()
{
    this->nextStateName = this->stateName;
    iter++;
    
    // 切换FSM控制模式
    switch((int) this->_data->controlParameters->control_mode)
    {
        case K_RECOVERY_STAND:
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
        
        case K_BACKFLIP:
            this->nextStateName = FSM_StateName::BACKFLIP;
            break;
        
        case K_FRONTJUMP:
            this->nextStateName = FSM_StateName::FRONTJUMP;
            break;
        
        case K_VISION:
            this->nextStateName = FSM_StateName::VISION;
            break;
        
        default:
            std::cout << "[CONTROL FSM] Bad Request: Cannot transition from "
                      << K_RECOVERY_STAND << " to "
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
TransitionData<T> FSM_State_RecoveryStand<T>::transition()
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
        
        case FSM_StateName::BACKFLIP:
            this->transitionData.done = true;
            break;
        
        case FSM_StateName::FRONTJUMP:
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
void FSM_State_RecoveryStand<T>::onExit()
{
    // Nothing to clean up when exiting
}

// template class FSM_State_RecoveryStand<double>;
template
class FSM_State_RecoveryStand<float>;
