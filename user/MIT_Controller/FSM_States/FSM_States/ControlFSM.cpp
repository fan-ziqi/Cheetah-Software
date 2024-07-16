/*============================ Control FSM ============================*/
/**
 * 管理机器人控制的有限状态机。
 * 处理对FSM状态函数的调用，并管理所有状态之间的转换。
 */

#include "ControlFSM.h"
#include <rt/rt_rc_interface.h>

/**
 * 控制FSM的构造函数。
 * 传递所有必要的数据并将其存储在结构中。用启动状态和操作模式初始化FSM。
 * 传入参数从robotrunner的给RobotController* _robot_ctrl赋值来
 *
 * @param _quadruped 四足的信息
 * @param _stateEstimator 包含估计状态
 * @param _legController 与腿部控制器的接口
 * @param _gaitScheduler 控制预定的足部接触模式
 * @param _desiredStateCommand 获取所需的COM状态轨迹
 * @param controlParameters 从GUI中传入控制参数
 */
template<typename T>
ControlFSM<T>::ControlFSM(Quadruped<T> *_quadruped,
                          StateEstimatorContainer<T> *_stateEstimator,
                          LegController<T> *_legController,
                          GaitScheduler<T> *_gaitScheduler,
                          DesiredStateCommand<T> *_desiredStateCommand,
                          RobotControlParameters *controlParameters,
                          VisualizationData *visualizationData,
                          MIT_UserParameters *userParameters)
{
    // 将指针添加到ControlFSMData结构体
    data._quadruped = _quadruped;
    data._stateEstimator = _stateEstimator;
    data._legController = _legController;
    data._gaitScheduler = _gaitScheduler;
    data._desiredStateCommand = _desiredStateCommand;
    data.controlParameters = controlParameters;
//    data.visualizationData = visualizationData;
    data.userParameters = userParameters;
    
    // 初始化并将所有FSM状态添加到状态列表中
    statesList.invalid = nullptr;
    statesList.passive = new FSM_State_Passive<T>(&data);
    statesList.jointPD = new FSM_State_JointPD<T>(&data);
    statesList.impedanceControl = new FSM_State_ImpedanceControl<T>(&data);
    statesList.standUp = new FSM_State_StandUp<T>(&data);
    statesList.balanceStand = new FSM_State_BalanceStand<T>(&data);
    statesList.locomotion = new FSM_State_Locomotion<T>(&data);
    statesList.recoveryStand = new FSM_State_RecoveryStand<T>(&data);
//    statesList.vision = new FSM_State_Vision<T>(&data);
    statesList.backflip = new FSM_State_BackFlip<T>(&data);
    statesList.frontJump = new FSM_State_FrontJump<T>(&data);
    
    // 进行安全检查并限制操作
    safetyChecker = new SafetyChecker<T>(&data);
    
    // 使用被动FSM状态初始化FSM
    initialize();
}

/**
 * 使用默认设置初始化控件FSM。应设置为被动状态和正常工作模式。
 */
template<typename T>
void ControlFSM<T>::initialize()
{
    // 用控制数据初始化一个新的FSM状态
    currentState = statesList.passive;
    
    // 输入新的当前状态
    currentState->onEnter();
    
    // 初始化为不处于转换状态
    nextState = currentState;
    
    // 将FSM模式初始化为正常操作
    operatingMode = FSM_OperatingMode::NORMAL;
}

/**
 * 调用每个控制循环迭代。确定机器人是否可以安全地运行控制，
 * 并检查当前状态是否有任何转换。如果一切正常，则运行常规状态行为。
 */
template<typename T>
void ControlFSM<T>::runFSM()
{
    // Publish state estimator data to other computer
    //for(size_t i(0); i<3; ++i){
    //_state_estimator.p[i] = data._stateEstimator->getResult().position[i];
    //_state_estimator.quat[i] = data._stateEstimator->getResult().orientation[i];
    //}
    //_state_estimator.quat[3] = data._stateEstimator->getResult().orientation[3];
    //state_estimator_lcm.publish("state_estimator_ctrl_pc", &_state_estimator);
    
    // 为了安全操作，检查机器人状态
    operatingMode = safetyPreCheck();

// data.controlParameters->control_mode = K_RECOVERY_STAND;
if(iter < 1000)
{
    data.controlParameters->control_mode = K_PASSIVE;
}
else if(iter < 2000)
{
    data.controlParameters->control_mode = K_RECOVERY_STAND;
}
else if(iter < 3000)
{
       data.controlParameters->control_mode = K_BALANCE_STAND;
}
//    else if(iter < 4000)
//    {
//        data.controlParameters->control_mode = K_LOCOMOTION;
//    }
//    else if(iter < 5000)
//    {
//        data.controlParameters->control_mode = K_BALANCE_STAND;
//    }
//    else
//    {
//        data.controlParameters->control_mode = K_BACKFLIP;
//    }

//    // 上下晃
//    int count_temp = 20000;
//    if(iter < 1000)
//    {
//        data.controlParameters->control_mode = K_PASSIVE;
//        // printf("Mark K_PASSIVE=0\n");
//    }
//    else if(iter < 2000)
//    {
//        data.controlParameters->control_mode = K_RECOVERY_STAND;
//    }
//    else if(iter < count_temp)
//    {
//        data.controlParameters->control_mode = K_BALANCE_STAND;
//        // printf("Mark control_mode = K_BALANCE_STAND\n");
//    }
//    else if(iter < count_temp + 3000)
//    {
//        data.controlParameters->control_mode = K_RECOVERY_STAND;
//        // printf("Mark control_mode = K_RECOVERY_STAND\n");
//    }
//    else
//    {
//        printf("done!\n");
//    }
    
    
    // 是否使用遥控器
    if(data.controlParameters->use_rc)
    {
        // 设定控制模式
        int rc_mode = data._desiredStateCommand->rcCommand->mode;
        
        if(rc_mode == RC_mode::RECOVERY_STAND)
        {
            data.controlParameters->control_mode = K_RECOVERY_STAND;
            
        }
        else if(rc_mode == RC_mode::LOCOMOTION)
        {
            data.controlParameters->control_mode = K_LOCOMOTION;
            
        }
        else if(rc_mode == RC_mode::QP_STAND)
        {
            data.controlParameters->control_mode = K_BALANCE_STAND;
            
        }
        else if(rc_mode == RC_mode::VISION)
        {
            data.controlParameters->control_mode = K_VISION;
            
        }
        else if(rc_mode == RC_mode::BACKFLIP || rc_mode == RC_mode::BACKFLIP_PRE)
        {
            data.controlParameters->control_mode = K_BACKFLIP;
        }
        //data.controlParameters->control_mode = K_FRONTJUMP;
        //std::cout<< "control mode: "<<data.controlParameters->control_mode<<std::endl;
    }
    
    
    // 如果操作模式是安全的，则运行机器人控制代码。下面为状态机
    if(operatingMode != FSM_OperatingMode::ESTOP)
    {
        // 如果没有检测到过渡，则运行正常控制
        if(operatingMode == FSM_OperatingMode::NORMAL)
        {
            // 检查当前状态是否有任何转换
            nextStateName = currentState->checkTransition();
            
            // 检测转换指令
            if(nextStateName != currentState->stateName)
            {
                // 将FSM工作模式设置为 TRANSITIONING
                operatingMode = FSM_OperatingMode::TRANSITIONING;
                
                // 按名称获取下一个FSM状态
                nextState = getNextState(nextStateName);
                
                // 输出转换初始化信息
                printInfo(1);
            }
            else
            {
                // 没有状态转换则运行当前状态控制器
                currentState->run();
            }
        }
        
        // 在发生转换时运行转换代码
        if(operatingMode == FSM_OperatingMode::TRANSITIONING)
        {
            // 获得转换数据，进行状态转换操作 有时候是延时切换状态 比如MPClocomotion
            transitionData = currentState->transition();
            
            // 检查机器人状态是否安全运行
            safetyPostCheck();
            
            // 运行状态转换，直到状态转换完成（延时到）
            if(transitionData.done)
            {
                // 干净地退出当前状态
                currentState->onExit();
                
                // 输出最终的转换信息
                printInfo(2);
                
                // 完成状态转换
                currentState = nextState;
                
                // 进入新状态
                currentState->onEnter();
                
                // 操作模式设置为 NORMAL
                operatingMode = FSM_OperatingMode::NORMAL;
            }
        }
        else
        {
            // 非转换状态 会进行检查机器人状态，确保操作安全。即任何状态下进行检查并限制
            safetyPostCheck();
        }
        
    }
    else
    { // if ESTOP 停止的默认状态
        currentState = statesList.passive;
        currentState->onEnter();
        nextStateName = currentState->stateName;
    }
    
    // 输出FSM的当前状态
    printInfo(0);
    
    // 自增迭代计数器
    iter++;
}

/**
 * 为了安全操作状态检查机器人状态。如果处于不安全状态，则在其安全之前不会运行一般控制代码
 *
 * @return 适当的操作模式
 */
template<typename T>
FSM_OperatingMode ControlFSM<T>::safetyPreCheck()
{
    // 如果当前状态需要，检查安全方向
    if(currentState->checkSafeOrientation && data.controlParameters->control_mode != K_RECOVERY_STAND)
    {
        // 姿态角不安全
        if(!safetyChecker->checkSafeOrientation())
        {
            operatingMode = FSM_OperatingMode::ESTOP;
            std::cout << "broken: Orientation Safety Ceck FAIL" << std::endl;
        }
    }
    
    // 默认返回当前操作模式
    return operatingMode;
}

/**
 * 计算控制后，检查机器人状态是否有安全的操作指令。
 * 打印出哪个命令是不安全的。每个状态都有一个选项来开关控制它所关心的命令的检查。
 *
 * 这是EDamp / EStop还是继续?
 * 为清晰起见，应将每个单独的检查拆分成各自的功能
 *
 * @return 适当的操作模式
 */
template<typename T>
FSM_OperatingMode ControlFSM<T>::safetyPostCheck()
{
    // 检查期望脚位置是否安全
    if(currentState->checkPDesFoot)
    {
        safetyChecker->checkPDesFoot();
    }
    
    // 检查期望前馈力是否安全
    if(currentState->checkForceFeedForward)
    {
        safetyChecker->checkForceFeedForward();
    }
    
    // 默认是返回当前的操作模式
    return operatingMode;
}

/**
 * 命令时返回相应的下一个FSM状态
 *
 * @param  stateName 下一个命令枚举状态名称
 * @return 下一个FSM状态
 */
template<typename T>
FSM_State<T> *ControlFSM<T>::getNextState(FSM_StateName stateName)
{
    // 通过枚举状态名称选择正确的FSM状态
    switch(stateName)
    {
        case FSM_StateName::INVALID:
            return statesList.invalid;
        
        case FSM_StateName::PASSIVE:
            return statesList.passive;
        
        case FSM_StateName::JOINT_PD:
            return statesList.jointPD;
        
        case FSM_StateName::IMPEDANCE_CONTROL:
            return statesList.impedanceControl;
        
        case FSM_StateName::STAND_UP:
            return statesList.standUp;
        
        case FSM_StateName::BALANCE_STAND:
            return statesList.balanceStand;
        
        case FSM_StateName::LOCOMOTION:
            return statesList.locomotion;
        
        case FSM_StateName::RECOVERY_STAND:
            return statesList.recoveryStand;

//        case FSM_StateName::VISION:
//            return statesList.vision;
        
        case FSM_StateName::BACKFLIP:
            return statesList.backflip;
        
        case FSM_StateName::FRONTJUMP:
            return statesList.frontJump;
        
        default:
            return statesList.invalid;
    }
}

/**
 * 定期打印控制FSM信息以及重要事件（如转换初始化和终止）
 * 单独的函数不会使实际代码混乱
 *
 * @param opt 常规或事件的打印模式选项
 */
template<typename T>
void ControlFSM<T>::printInfo(int opt)
{
    switch(opt)
    {
        case 0:  // 定期进行正常输出
            // 自增输出迭代器
            printIter++;
            
            // 按控制频率输出
            if(printIter == printNum)
            {
                std::cout << "[CONTROL FSM] Printing FSM Info...\n";
                std::cout
                        << "---------------------------------------------------------\n";
                std::cout << "Iteration: " << iter << "\n";
                if(operatingMode == FSM_OperatingMode::NORMAL)
                {
                    std::cout << "Operating Mode: NORMAL in " << currentState->stateString
                              << "\n";
                    
                }
                else if(operatingMode == FSM_OperatingMode::TRANSITIONING)
                {
                    std::cout << "Operating Mode: TRANSITIONING from "
                              << currentState->stateString << " to "
                              << nextState->stateString << "\n";
                    
                }
                else if(operatingMode == FSM_OperatingMode::ESTOP)
                {
                    std::cout << "Operating Mode: ESTOP\n";
                }
                std::cout << "Gait Type: " << data._gaitScheduler->gaitData.gaitName
                          << "\n";
                std::cout << std::endl;
                
                // Reset iteration counter
                printIter = 0;
            }
            
            // 打印有关机器人状态的机器人信息
            // data._gaitScheduler->printGaitInfo();
            // data._desiredStateCommand->printStateCommandInfo();
            
            break;
        
        case 1:  // 正在初始化FSM状态转换
            std::cout << "[CONTROL FSM] Transition initialized from "
                      << currentState->stateString << " to " << nextState->stateString
                      << "\n"
                      << std::endl;
            
            break;
        
        case 2:  // 完成FSM状态转换
            std::cout << "[CONTROL FSM] Transition finalizing from "
                      << currentState->stateString << " to " << nextState->stateString
                      << "\n"
                      << std::endl;
            
            break;
    }
}

// template class ControlFSM<double>; This should be fixed... need to make
// RobotRunner a template
template
class ControlFSM<float>;
