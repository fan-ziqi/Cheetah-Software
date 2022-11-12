#include "MIT_Controller.hpp"

MIT_Controller::MIT_Controller() : RobotController()
{}

//#define RC_ESTOP
/**
 * Initializes the Control FSM.
 */
void MIT_Controller::initializeController()
{
    // 初始化一个新的GaitScheduler对象
    _gaitScheduler = new GaitScheduler<float>(&userParameters, _controlParameters->controller_dt);
    
    // Initialize a new ContactEstimator object
    //_contactEstimator = new ContactEstimator<double>();
    ////_contactEstimator->initialize();
    
    // 用所有需要的数据初始化控制FSM（数据从硬件接口->robotrunner->this）
    _controlFSM = new ControlFSM<float>(_quadruped, _stateEstimator,
                                        _legController, _gaitScheduler,
                                        _desiredStateCommand, _controlParameters,
                                        _visualizationData, &userParameters);
}

/**
 * Calculate the commands for the leg controllers using the ControlFSM logic.
 */
void MIT_Controller::runController()
{
//    printf("MIT_Controller::runController()\r\n");
    // 查找当前步态计划
    _gaitScheduler->step();
    
    // 找到想要的状态轨迹
    _desiredStateCommand->convertToStateCommands();
    
    // 运行控制FSM代码
    _controlFSM->runFSM();
}


