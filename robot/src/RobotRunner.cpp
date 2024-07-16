/*!
 * @file RobotRunner.cpp
 * @brief 运行机器人控制器的通用框架
 * 该代码是迷mini cheetah和cheetah 3的控制代码和硬件/仿真之间的通用接口
 */

#include <unistd.h>

#include "RobotRunner.h"
#include "Controllers/ContactEstimator.h"
#include "Controllers/OrientationEstimator.h"
#include "Dynamics/Cheetah3.h"
#include "Dynamics/MiniCheetah.h"
#include "Utilities/Utilities_print.h"
#include "ParamHandler.hpp"
#include "Utilities/Timer.h"
#include "Controllers/PositionVelocityEstimator.h"
//#include "rt/rt_interface_lcm.h"

#ifdef CYBERDOG

#include "Dynamics/CyberdogParams.h"

#endif


/**
 * 构造函数 将运行框架加入任务管理器
 * 在hardwareBridge使用
 * 实例化运行器 传入控制器，任务管理器 参数 名称
 * _robotRunner = new RobotRunner(_controller, &taskManager, _robotParams.controller_dt, "robot-control");
**/
RobotRunner::RobotRunner(RobotController *robot_ctrl,
                         PeriodicTaskManager *manager,
                         float period, std::string name) :
        PeriodicTask(manager, period, name),//添加任务
        _lcm(getLcmUrl(255))
{
    _robot_ctrl = robot_ctrl;
}

/**
 * 初始化机器人模型，状态估计器，腿部控制器,
 * 机器人数据，以及任何控制逻辑特定的数据
 */
void RobotRunner::init()
{
    printf("[RobotRunner] initialize\n");
    
    // 选择要构建的机器人类型，_quadruped里面存了与机器人相关的数据
    if(robotType == RobotType::MINI_CHEETAH)
    {
#ifdef CYBERDOG
#ifdef USE_SIM
        _quadruped = buildMiniCheetah<float>();
#else
        _quadruped = buildCyberdog<float>();
#endif
#else
        _quadruped = buildMiniCheetah<float>();
#endif
    }
    else
    {
        _quadruped = buildCheetah3<float>();
    }
    
    // 初始化机器人模型
    _model = _quadruped.buildModel();
    // 初始化开机时腿的位置
    _jpos_initializer = new JPosInitializer<float>(3., controlParameters->controller_dt);
    
    // 初始化腿控制器
    _legController = new LegController<float>(_quadruped);
    // 初始化状态估计器
    _stateEstimator = new StateEstimatorContainer<float>(
            cheaterState, vectorNavData, _legController->datas,
            &_stateEstimate, controlParameters);
    // 重置状态估计
    initializeStateEstimator(false);
    
    memset(&rc_control, 0, sizeof(rc_control_settings));
    // 初始化DesiredStateCommand对象（所需的状态命令）
    _desiredStateCommand = new DesiredStateCommand<float>(driverCommand,
                                                          &rc_control,
                                                          controlParameters,
                                                          &_stateEstimate,
                                                          controlParameters->controller_dt);
    
    // 控制器初始化
    _robot_ctrl->_model = &_model; //模型
    _robot_ctrl->_quadruped = &_quadruped; //四足机器人物理特性的表征
    _robot_ctrl->_legController = _legController; //腿部控制器，对象
    _robot_ctrl->_stateEstimator = _stateEstimator; //状态估计器
    _robot_ctrl->_stateEstimate = &_stateEstimate; //状态估计值
    _robot_ctrl->_visualizationData = visualizationData; //可视化数据
    _robot_ctrl->_robotType = robotType; //机器人类型
    _robot_ctrl->_driverCommand = driverCommand; //驱动命令
    _robot_ctrl->_controlParameters = controlParameters; //控制参数
    _robot_ctrl->_desiredStateCommand = _desiredStateCommand; //期望状态命令
    
    _robot_ctrl->initializeController(); //初始化控制器
    
}

/**
 * 通过调用每个主要组件运行其各自的步骤，运行整个机器人控制系统
 * 机器人系统每个执行周期的运行内容（运行周期从参数文件获取）
 */
void RobotRunner::run()
{
    // 运行状态预测
    //_stateEstimator->run(cheetahMainVisualization);
    _stateEstimator->run();
    // 可视化数据清零
    //cheetahMainVisualization->p = _stateEstimate.position;
    visualizationData->clear();
    
    // 更新来自机器人的数据到LegController对象
    setupStep();
    
    static int count_ini(0);
    ++count_ini;
    // 计数50次后开启腿控制
    if(count_ini < 10)
    {
        _legController->setEnabled(false);
    }
    else if(20 < count_ini && count_ini < 30)
    {
        _legController->setEnabled(false);
    }
    else if(40 < count_ini && count_ini < 50)
    {
        _legController->setEnabled(false);
    }
    else
    {
        //使能LegController对象
        _legController->setEnabled(true);

//        // 将use_rc设为0，跳过Estop()模式
//        controlParameters->use_rc = 0;
        
        //当遥控器控制时的rc_control.mode为0时，将LegController对象的控制命令数据清零
        if((rc_control.mode == 0) && controlParameters->use_rc)
        {
            if(count_ini % 1000 == 0) printf("ESTOP!\n");
            for(int leg = 0; leg < 4; leg++)
            {
                _legController->commands[leg].zero();
            }
            // Estop调用了_controlFSM->initialize();
            _robot_ctrl->Estop();
        }
        else
        {
            // 如果关节位置初始对象没有初始化时，将LegController对象的各个关节的kp和kd赋值
            if(!_jpos_initializer->IsInitialized(_legController))
            {
                Mat3<float> kpMat;
                Mat3<float> kdMat;
                // 更新jpos反馈增益
                if(robotType == RobotType::MINI_CHEETAH)
                {
                    kpMat << 5, 0, 0, 0, 5, 0, 0, 0, 5;
                    kdMat << 0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1;
                }
                else if(robotType == RobotType::CHEETAH_3)
                {
                    kpMat << 50, 0, 0, 0, 50, 0, 0, 0, 50;
                    kdMat << 1, 0, 0, 0, 1, 0, 0, 0, 1;
                }
                else
                {
                    assert(false);
                }
                
                for(int leg = 0; leg < 4; leg++)
                {
                    _legController->commands[leg].kpJoint = kpMat;
                    _legController->commands[leg].kdJoint = kdMat;
                }
            }
            else
            {
                //初始完成，执行机器人控制对象
                // 运行控制
                _robot_ctrl->runController();
                cheetahMainVisualization->p = _stateEstimate.position;
                
                // 更新可视化
                _robot_ctrl->updateVisualization();
                cheetahMainVisualization->p = _stateEstimate.position;
            }
        }
    }
    
    // 可视化 (之后会作为一个单独的函数)
    for(int leg = 0; leg < 4; leg++)
    {
        for(int joint = 0; joint < 3; joint++)
        {
            cheetahMainVisualization->q[leg * 3 + joint] = _legController->datas[leg].q[joint];
        }
    }
    cheetahMainVisualization->p.setZero();
    cheetahMainVisualization->p = _stateEstimate.position;
    cheetahMainVisualization->quat = _stateEstimate.orientation;
    
    // 将生成的leg controller控制命令数据更新到机器人各控制系统
    finalizeStep();
}

/*!
 * 在运行用户代码之前，设置legController和估计器
 */
void RobotRunner::setupStep()
{
    // 选择机型、更新数据
    if(robotType == RobotType::MINI_CHEETAH)
    {
#ifdef CYBERDOG
#ifdef USE_SIM
        _legController->updateData(spiData);
#else
        _legController->updateData(cyberdogData);
#endif
#else
        _legController->updateData(spiData);
#endif
    }
    else if(robotType == RobotType::CHEETAH_3)
    {
        _legController->updateData(tiBoardData);
    }
    else
    {
        assert(false);
    }
    
    // 为新一次的循环设置设置legController
    _legController->zeroCommand(); //腿部控制命令清零
    _legController->setEnabled(true); //开启腿部控制
//    _legController->setMaxTorqueCheetah3(208.5); //Cheetah3设置最大力矩
    
    // 状态估计器
    // check transition to cheater mode:
    if(!_cheaterModeEnabled && controlParameters->cheater_mode)
    {
        printf("[RobotRunner] Transitioning to Cheater Mode...\n");
        initializeStateEstimator(true);
        // todo any configuration
        _cheaterModeEnabled = true;
    }
    
    // check transition from cheater mode:
    if(_cheaterModeEnabled && !controlParameters->cheater_mode)
    {
        printf("[RobotRunner] Transitioning from Cheater Mode...\n");
        initializeStateEstimator(false);
        // todo any configuration
        _cheaterModeEnabled = false;
    }
    
    // 获得遥控器设置
    get_rc_control_settings(&rc_control);
    
    // todo safety checks, sanity checks, etc...
}

/*!
 * 在用户代码之后，发送腿命令，更新状态估计，并发布调试数据
 */
void RobotRunner::finalizeStep()
{
    //选择机型，发布命令
    if(robotType == RobotType::MINI_CHEETAH)
    {
#ifdef CYBERDOG
#ifdef USE_SIM
        _legController->updateCommand(spiCommand);
#else
        _legController->updateCommand(cyberdogCmd);
#endif
#else
        _legController->updateCommand(spiCommand);
#endif
    }
    else if(robotType == RobotType::CHEETAH_3)
    {
        _legController->updateCommand(tiBoardCommand);
    }
    else
    {
        assert(false);
    }

#ifndef CYBERDOG
    // 设置LCM
    _legController->setLcm(&leg_control_data_lcm, &leg_control_command_lcm);
    _stateEstimate.setLcm(state_estimator_lcm);
    
    // 发布主题
    _lcm.publish("leg_control_command", &leg_control_command_lcm);
    _lcm.publish("leg_control_data", &leg_control_data_lcm);
    _lcm.publish("state_estimator", &state_estimator_lcm);
#endif
    _iterations++;
}

/*!
 * 重置给定模式下的状态估计器
 * @param cheaterMode
 */
void RobotRunner::initializeStateEstimator(bool cheaterMode)
{
    _stateEstimator->removeAllEstimators(); //删除所有估计器
    _stateEstimator->addEstimator<ContactEstimator<float>>(); //添加接触状态估计器
    Vec4<float> contactDefault;
    contactDefault << 0.5, 0.5, 0.5, 0.5;
    _stateEstimator->setContactPhase(contactDefault);
    if(cheaterMode)
    {
        _stateEstimator->addEstimator<CheaterOrientationEstimator<float>>();
        _stateEstimator->addEstimator<CheaterPositionVelocityEstimator<float>>();
    }
    else
    {
        //使用这个 添加位置角度状态估计器
        _stateEstimator->addEstimator<VectorNavOrientationEstimator<float>>();
        _stateEstimator->addEstimator<LinearKFPositionVelocityEstimator<float>>();
    }
}

RobotRunner::~RobotRunner()
{
    delete _legController;
    delete _stateEstimator;
    delete _jpos_initializer;
}

void RobotRunner::cleanup()
{}
