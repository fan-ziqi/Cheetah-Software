/*!
 * @file HardwareBridge.cpp
 * @brief 机器人代码和机器人硬件之间的接口
 * 这个类初始化两个机器人的硬件并允许机器人控制器来访问它
 */
#ifdef linux

#include <sys/mman.h>
#include <unistd.h>
#include <cstring>
#include <thread>
#include "Configuration.h"

#include "HardwareBridge.h"
//#include "rt/rt_rc_interface.h"
#include "rt/rt_sbus.h"
#include "rt/rt_spi.h"
#include "rt/rt_vectornav.h"
#include "rt/rt_ethercat.h"
#include "Utilities/Utilities_print.h"

#include <termios.h>

#define USE_MICROSTRAIN

/*!
 * 如果在初始化过程中发生错误，在启动电机之前，打印错误并退出。
 * @param reason 错误信息字符串
 * @param printErrno If true, also print C errno
 */
void HardwareBridge::initError(const char *reason, bool printErrno)
{
    printf("FAILED TO INITIALIZE HARDWARE: %s\n", reason);
    
    if(printErrno)
    {
        printf("Error: %s\n", strerror(errno));
    }
    
    exit(-1);
}

/*!
 * Cheetah 3和Mini Cheetah所有的硬件初始化都是通用的
 */
void HardwareBridge::initCommon()
{
    printf("[HardwareBridge] Init stack\n");
    prefaultStack();
    printf("[HardwareBridge] Init scheduler\n");
    setupScheduler();
    
    if(!_interfaceLCM.good())
    {
        initError("_interfaceLCM failed to initialize\n", false);
    }
    
    printf("[HardwareBridge] Subscribe LCM\n");
    _interfaceLCM.subscribe("interface", &HardwareBridge::handleGamepadLCM, this);
    _interfaceLCM.subscribe("interface_request", &HardwareBridge::handleControlParameter, this);
    
    printf("[HardwareBridge] Start interface LCM handler\n");
    _interfaceLcmThread = std::thread(&HardwareBridge::handleInterfaceLCM, this);
    
}

/*!
 * 运行lcm通信接口
 */
void HardwareBridge::handleInterfaceLCM()
{
    while(!_interfaceLcmQuit)
    {
        /*
         * LCM自动解码消息，再传给回调函数，回调函数可以识别消息类型。
         * 因为回调函数在lcm.handle()方法中调度，所以不需要并发执行，这些都在一个单线程中完成。
         * 调用lcm.handle()非常重要，函数会保持阻塞直到有任务需要做。
         * */
        _interfaceLCM.handle();
    }
}

/*!
 * 写入堆栈上的16 KB缓冲区。如果我们在堆栈中使用4K页面，这将确保在堆栈增长时不会出现页面错误。
 * 此外，mlock的所有页面都与当前进程相关，这防止了cheetah软件溢出。如果内存耗尽，
 * 机器人程序将被OOM杀手进程杀死（并留下日志），而不是变得没有响应。
 */
void HardwareBridge::prefaultStack()
{
    printf("[Init] Prefault stack...\n");
    volatile char stack[MAX_STACK_SIZE];
    memset(const_cast<char *>(stack), 0, MAX_STACK_SIZE);
    if(mlockall(MCL_CURRENT | MCL_FUTURE) == -1)
    {
        initError(
                "mlockall failed.  This is likely because you didn't run robot as "
                "root.\n",
                true);
    }
}

/*!
 * 为实时优先级配置调度程序
 */
void HardwareBridge::setupScheduler()
{
    printf("[Init] Setup RT Scheduler...\n");
    struct sched_param params;
    params.sched_priority = TASK_PRIORITY;
    if(sched_setscheduler(0, SCHED_FIFO, &params) == -1)
    {
        initError("sched_setscheduler failed.\n", true);
    }
}

/*!
 * 用于gamepad消息的LCM处理程序
 */
void HardwareBridge::handleGamepadLCM(const lcm::ReceiveBuffer *rbuf,
                                      const std::string &chan,
                                      const gamepad_lcmt *msg)
{
    (void) rbuf;
    (void) chan;
    // 接受到的消息设置到手柄命令中
    _gamepadCommand.set(msg);
}

/*!
 * 控制参数的LCM处理程序
 */
void HardwareBridge::handleControlParameter(
        const lcm::ReceiveBuffer *rbuf, const std::string &chan, //频道名
        const control_parameter_request_lcmt *msg) //自定义消息类型
{
    (void) rbuf;
    (void) chan;
    if(msg->requestNumber <= _parameter_response_lcmt.requestNumber)
    {
        // 什么都没做
        printf(
                "[HardwareBridge] Warning: the interface has run a ControlParameter "
                "iteration, but there is no new request!\n");
        // return;
    }
    
    // 完整性检查
    s64 nRequests = msg->requestNumber - _parameter_response_lcmt.requestNumber;
    if(nRequests != 1)
    {
        printf("[ERROR] Hardware bridge: we've missed %ld requests\n",
               nRequests - 1);
    }
    
    switch(msg->requestKind)
    {
        case (s8) ControlParameterRequestKind::SET_USER_PARAM_BY_NAME:
        {
            if(!_userControlParameters)
            {
                printf("[Warning] Got user param %s, but not using user parameters!\n",
                       (char *) msg->name);
            }
            else
            {
                std::string name((char *) msg->name);
                ControlParameter &param = _userControlParameters->collection.lookup(name);
                
                // 类型检查
                if((s8) param._kind != msg->parameterKind)
                {
                    throw std::runtime_error(
                            "type mismatch for parameter " + name + ", robot thinks it is " +
                            controlParameterValueKindToString(param._kind) +
                            " but received a command to set it to " +
                            controlParameterValueKindToString(
                                    (ControlParameterValueKind) msg->parameterKind));
                }
                
                // 在这里实际设置
                ControlParameterValue v;
                memcpy(&v, msg->value, sizeof(v));
                param.set(v, (ControlParameterValueKind) msg->parameterKind);
                
                // respond:
                _parameter_response_lcmt.requestNumber = msg->requestNumber; // 承认设置已经发生
                _parameter_response_lcmt.parameterKind = msg->parameterKind; // 仅用于调试打印语句
                memcpy(_parameter_response_lcmt.value, msg->value, 64);
                //_parameter_response_lcmt.value = _parameter_request_lcmt.value; // just
                // 用于调试打印语句
                strcpy((char *) _parameter_response_lcmt.name, name.c_str());
                _parameter_response_lcmt.requestKind = msg->requestKind;
                
                printf("[User Control Parameter] set %s to %s\n", name.c_str(),
                       controlParameterValueToString(
                               v, (ControlParameterValueKind) msg->parameterKind)
                               .c_str());
            }
        }
            break;
        
        case (s8) ControlParameterRequestKind::SET_ROBOT_PARAM_BY_NAME:
        {
            std::string name((char *) msg->name);
            ControlParameter &param = _robotParams.collection.lookup(name);
            
            // 类型检查
            if((s8) param._kind != msg->parameterKind)
            {
                throw std::runtime_error(
                        "type mismatch for parameter " + name + ", robot thinks it is " +
                        controlParameterValueKindToString(param._kind) +
                        " but received a command to set it to " +
                        controlParameterValueKindToString(
                                (ControlParameterValueKind) msg->parameterKind));
            }
            
            // 在这里实际设置
            ControlParameterValue v;
            memcpy(&v, msg->value, sizeof(v));
            param.set(v, (ControlParameterValueKind) msg->parameterKind);
            
            // respond:
            _parameter_response_lcmt.requestNumber = msg->requestNumber;  // 承认设置已经发生
            _parameter_response_lcmt.parameterKind = msg->parameterKind;  // 仅用于调试打印语句
            memcpy(_parameter_response_lcmt.value, msg->value, 64);
            //_parameter_response_lcmt.value = _parameter_request_lcmt.value; // just
            // 用于调试打印语句
            strcpy((char *) _parameter_response_lcmt.name, name.c_str());
            _parameter_response_lcmt.requestKind = msg->requestKind;
            
            printf("[Robot Control Parameter] set %s to %s\n", name.c_str(),
                   controlParameterValueToString(
                           v, (ControlParameterValueKind) msg->parameterKind)
                           .c_str());
            
        }
            break;
        
        default:
        {
            throw std::runtime_error("parameter type unsupported");
        }
            break;
    }
    _interfaceLCM.publish("interface_response", &_parameter_response_lcmt);
}


MiniCheetahHardwareBridge::MiniCheetahHardwareBridge(RobotController *robot_ctrl, bool load_parameters_from_file)
        : HardwareBridge(robot_ctrl), _spiLcm(getLcmUrl(255)), _microstrainLcm(getLcmUrl(255))
{
    _load_parameters_from_file = load_parameters_from_file;
}

/*!
 * Main method for Mini Cheetah hardware
 */
void MiniCheetahHardwareBridge::run()
{
    //初始化通信模块，调用基类HardwareBridge::initCommon()函数，通过LCM通信模块订阅interface和interface_request两个主题，并创建LCM线程
    initCommon();
    //initHardware();

#ifdef CYBERDOG
    _cyberdogInterface = new CyberdogInterface(500);//500HZ
#else
    //初始化MiniCheetah的spi和IMU
    initHardware();
#endif
    
    if(_load_parameters_from_file)
    {
        printf("[Hardware Bridge] Loading parameters from file...\n");
        
        try
        {
            _robotParams.initializeFromYamlFile(THIS_COM "config/mini-cheetah-defaults.yaml");
        }
        catch(std::exception &e)
        {
            printf("Failed to initialize robot parameters from yaml file: %s\n", e.what());
            exit(1);
        }
        
        if(!_robotParams.isFullyInitialized())
        {
            printf("Failed to initialize all robot parameters\n");
            exit(1);
        }
        
        printf("Loaded robot parameters\n");
        
        if(_userControlParameters)
        {
            try
            {
                _userControlParameters->initializeFromYamlFile(THIS_COM "config/mc-mit-ctrl-user-parameters.yaml");
            }
            catch(std::exception &e)
            {
                printf("Failed to initialize user parameters from yaml file: %s\n", e.what());
                exit(1);
            }
            
            if(!_userControlParameters->isFullyInitialized())
            {
                printf("Failed to initialize all user parameters\n");
                exit(1);
            }
            
            printf("Loaded user parameters\n");
        }
        else
        {
            printf("Did not load user parameters because there aren't any\n");
        }
    }
    else
    {
        printf("[Hardware Bridge] Loading parameters over LCM...\n");
        while(!_robotParams.isFullyInitialized())
        {
            printf("[Hardware Bridge] Waiting for robot parameters...\n");
            usleep(1000000);
        }
        
        if(_userControlParameters)
        {
            while(!_userControlParameters->isFullyInitialized())
            {
                printf("[Hardware Bridge] Waiting for user parameters...\n");
                usleep(1000000);
            }
        }
    }
    
    printf("[Hardware Bridge] Got all parameters, starting up!\n");
    
    //创建和配置RobotRunner对象（任务对象）
    _robotRunner = new RobotRunner(_controller, &taskManager, _robotParams.controller_dt, "robot-control"); //500HZ
    
    _robotRunner->driverCommand = &_gamepadCommand;
#ifdef CYBERDOG
    _robotRunner->cyberdogData = &_cyberdogInterface->cyberdogData;
    _robotRunner->cyberdogCmd = &_cyberdogInterface->cyberdogCmd;
#else
    _robotRunner->spiData = &_spiData;
    _robotRunner->spiCommand = &_spiCommand;
#endif
    _robotRunner->robotType = RobotType::MINI_CHEETAH;
    _robotRunner->vectorNavData = &_vectorNavData;
    _robotRunner->controlParameters = &_robotParams;
    _robotRunner->visualizationData = &_visualizationData;
    _robotRunner->cheetahMainVisualization = &_mainCheetahVisualization;
    
    _robotRunner->init();
    _firstRun = false;
    
    // 初始化控制线程
    
    // 启动状态任务
    statusTask.start();

#ifdef CYBERDOG
    // cyberdog的imu数据从这个线程里获取
    _cyberdogThread = std::thread(&MiniCheetahHardwareBridge::CyberdogProcessData, this);
#else
    // 启动spi通信任务，spi通信负责传输控制命令和接收传感器数据
    PeriodicMemberFunction<MiniCheetahHardwareBridge> spiTask(
            &taskManager, .002, "spi", &MiniCheetahHardwareBridge::runSpi, this);
    spiTask.start();
    
    // 启动IMU线程
    if(_microstrainInit)
    {
        _microstrainThread = std::thread(&MiniCheetahHardwareBridge::runMicrostrain, this);
    }
#endif
    
    // 执行RobotRunner的任务，开启机器人控制器
    _robotRunner->start();

#ifndef CYBERDOG
    // 启动可视化数据LCM通信任务
    PeriodicMemberFunction<MiniCheetahHardwareBridge> visualizationLCMTask(
            &taskManager, .0167, "lcm-vis",
            &MiniCheetahHardwareBridge::publishVisualizationLCM, this);
    visualizationLCMTask.start();
    
    // 启动惯性导航日志任务
    PeriodicMemberFunction<MiniCheetahHardwareBridge> microstrainLogger(
            &taskManager, .001, "microstrain-logger",
            &MiniCheetahHardwareBridge::logMicrostrain, this);
    microstrainLogger.start();
#endif

#ifdef USE_RC
    // 启动遥控器指令接收任务
    _port = init_sbus(false);  // Not Simulation
    PeriodicMemberFunction<HardwareBridge> sbusTask(
            &taskManager, .005, "rc_controller",
            &HardwareBridge::run_sbus, this);
    sbusTask.start();
    ///
     PeriodicMemberFunction<MiniCheetahHardwareBridge> visualizationLCMTask(
            &taskManager, .0167, "lcm-vis",
            &MiniCheetahHardwareBridge::publishVisualizationLCM, this);
    visualizationLCMTask.start();
    
    // 启动惯性导航日志任务
    PeriodicMemberFunction<MiniCheetahHardwareBridge> microstrainLogger(
            &taskManager, .001, "microstrain-logger",
            &MiniCheetahHardwareBridge::logMicrostrain, this);
    microstrainLogger.start();

        // 启动spi通信任务，spi通信负责传输控制命令和接收传感器数据
    PeriodicMemberFunction<MiniCheetahHardwareBridge> spiTask(
            &taskManager, .002, "spi", &MiniCheetahHardwareBridge::runSpi, this);
    spiTask.start();
    
    // 启动IMU线程
     if(_microstrainInit)
     {
        _microstrainThread = std::thread(&MiniCheetahHardwareBridge::runMicrostrain, this);
     }
    
#endif

#ifdef USE_KEYBOARD
    _keyboardThread = std::thread(&MiniCheetahHardwareBridge::run_keyboard, this);
#endif
    
    //每隔1秒循环
    for(;;)
    {
        usleep(1000000);
        // printf("joy %f\n", _robotRunner->driverCommand->leftStickAnalog[0]);
    }
}

/*!
 * 用SBUS接收控制器数据
 */
void HardwareBridge::run_sbus()
{
    if(_port > 0)
    {
        int x = receive_sbus(_port);
        if(x)
        {
            sbus_packet_complete();
        }
    }
}


static bool kbhit()
{
    termios term;
    tcgetattr(0, &term);
    
    termios term2 = term;
    term2.c_lflag &= ~ICANON;
    tcsetattr(0, TCSANOW, &term2);
    
    int byteswaiting;
    ioctl(0, FIONREAD, &byteswaiting);
    
    tcsetattr(0, TCSANOW, &term);
    
    return byteswaiting > 0;
}

extern rc_control_settings rc_control;

void HardwareBridge::run_keyboard()
{
    int c;
    // Check for keyboard input
    while(true)
    {
        if(kbhit())
        {
            c = fgetc(stdin);
            printf("0: switch mode to OFF\r\n");
            printf("6: switch mode to RECOVERY_STAND\r\n");
            printf("3: switch mode to BALANCE_STAND\r\n");
            printf("4: switch mode to LOCOMOTION\r\n");
            printf("7: switch mode to TWO_LWG_PRE\r\n");
            printf("8: switch mode to TWO_LWG\r\n");
            printf("q: height+0.1\r\n");
            printf("z: height-0.1\r\n");
            printf("w: roll+0.1\r\n");
            printf("x: roll-0.1\r\n");
            printf("e: pitch+0.1\r\n");
            printf("c: pitch-0.1\r\n");
            printf("r: yaw+0.1\r\n");
            printf("v: yaw-0.1\r\n");
            printf("t: vx_des+0.1\r\n");
            printf("b: vx_des-0.1\r\n");
            printf("y: vy_des+0.1\r\n");
            printf("n: vy_des-0.1\r\n");
            printf("g: variable[]\r\n");
            printf("h: variable_0\r\n");
            printf("5: PASSIVE\r\n");
            switch(c)
            {
                case '0':
                    printf("switch mode to OFF\r\n");
                    rc_control.mode = 0;
                    break;
                case '6':
                    printf("switch mode to RECOVERY_STAND\r\n");
                    rc_control.mode = 12;
                    break;
                case '3':
                    printf("switch mode to BALANCE_STAND\r\n");
                    rc_control.mode = 3;
                    break;
                case '4':
                    printf("switch mode to LOCOMOTION\r\n");
                    rc_control.mode = 11;
                    break;
                case '5':
                    printf("switch mode to Passive\r\n");
                    rc_control.mode = 2;
                    break;    
                case 'q':
                    printf("height+0.1\r\n");
                    rc_control.height_variation += 0.1;
                    break;
                case 'z':
                    printf("height-0.1\r\n");
                    rc_control.height_variation -= 0.1;
                    break;
                case 'w':
                    printf("roll+0.1\r\n");
                    rc_control.rpy_des[0] += 0.1;
                    break;
                case 'x':
                    printf("roll-0.1\r\n");
                    rc_control.rpy_des[0] -= 0.1;
                    break;
                case 'e':
                    printf("pitch+0.1\r\n");
                    rc_control.rpy_des[1] += 0.1;
                    break;
                case 'c':
                    printf("pitch-0.1\r\n");
                    rc_control.rpy_des[1] -= 0.1;
                    break;
                case 'r':
                    printf("yaw+0.1\r\n");
                    rc_control.rpy_des[2] += 0.1;
                    break;
                case 'v':
                    printf("yaw-0.1\r\n");
                    rc_control.rpy_des[2] -= 0.1;
                    break;
                case 't':
                    printf("vx_des+0.1\r\n");
                    rc_control.v_des[0] += 0.1;
                    break;
                case 'b':
                    printf("vx_des-0.1\r\n");
                    rc_control.v_des[0] -= 0.1;
                    break;
                case 'y':
                    printf("vy_des+0.1\r\n");
                    rc_control.v_des[1] += 0.1;
                    std::cout<< "speed:" << rc_control.v_des[1] <<std::endl;
                    break;
                case 'n':
                    printf("vy_des-0.1\r\n");
                    rc_control.v_des[1] -= 0.1;
                    break;
                case 'g':
                    printf("variable\r\n");
                    //rc_control.variable[0] = 3;
                    rc_control.variable[0] = 8;
                    break;
                case 'h':
                    printf("variable_0\r\n");
                    //rc_control.variable[0] = 3;
                    rc_control.variable[0] = 0 ;
                    break;
                default:
                    break;   
                                  
            }
        }
        usleep(10000);
       // std::cout<< "speed:" << rc_control.v_des[1] <<std::endl;
    }
    
}

void MiniCheetahHardwareBridge::runMicrostrain() {
  while(true) {
    _microstrainImu.run();

#ifdef USE_MICROSTRAIN
    _vectorNavData.accelerometer = _microstrainImu.acc;
    _vectorNavData.quat[0] = _microstrainImu.quat[0];
    _vectorNavData.quat[1] = _microstrainImu.quat[1];
    _vectorNavData.quat[2] = _microstrainImu.quat[2];
    _vectorNavData.quat[3] = _microstrainImu.quat[3];
    _vectorNavData.gyro = _microstrainImu.gyro;
#endif
  }


}

void MiniCheetahHardwareBridge::logMicrostrain()
{
    _microstrainImu.updateLCM(&_microstrainData);
    _microstrainLcm.publish("microstrain", &_microstrainData);
}

/*!
 * 初始化Mini Cheetah特定的硬件
 */
void MiniCheetahHardwareBridge::initHardware()
{
    _vectorNavData.quat << 1, 0, 0, 0;
#ifndef USE_MICROSTRAIN
    printf("[MiniCheetahHardware] Init vectornav\n");
    if (!init_vectornav(&_vectorNavData)) {
      printf("Vectornav failed to initialize\n");
      //initError("failed to initialize vectornav!\n", false);
    }
#endif
    
    init_spi();
    _microstrainInit = _microstrainImu.tryInit(0, 921600);//_microstrainImu.tryInit(0, 921600);//460800
}

/*!
 * 初始化Cheetah3特定的硬件
 */
void Cheetah3HardwareBridge::initHardware()
{
    _vectorNavData.quat << 1, 0, 0, 0;
    printf("[Cheetah 3 Hardware] Init vectornav\n");
    if(!init_vectornav(&_vectorNavData))
    {
        printf("Vectornav failed to initialize\n");
        printf_color(PrintColor::Red, "****************\n"
                                      "**  WARNING!  **\n"
                                      "****************\n"
                                      "  IMU DISABLED  \n"
                                      "****************\n\n");
        //initError("failed to initialize vectornav!\n", false);
    }
}

/*!
 * Run Mini Cheetah SPI
 */
void MiniCheetahHardwareBridge::runSpi() {
  spi_command_t* cmd = get_spi_command();
  spi_data_t* data = get_spi_data();

  memcpy(cmd, &_spiCommand, sizeof(spi_command_t));
  spi_driver_run();
  memcpy(&_spiData, data, sizeof(spi_data_t));

  _spiLcm.publish("spi_data", data);
  _spiLcm.publish("spi_command", cmd);
}

/*!
 * 运行lcm获取和发送信息
 */
void Cheetah3HardwareBridge::runEcat()
{
    rt_ethercat_set_command(_tiBoardCommand);
    rt_ethercat_run();
    rt_ethercat_get_data(_tiBoardData);
    
    publishEcatLCM();
}

/*!
 * 发送自定义消息
 */
void Cheetah3HardwareBridge::publishEcatLCM()
{
    for(int leg = 0; leg < 4; leg++)
    {
        ecatCmdLcm.x_des[leg] = _tiBoardCommand[leg].position_des[0];
        ecatCmdLcm.y_des[leg] = _tiBoardCommand[leg].position_des[1];
        ecatCmdLcm.z_des[leg] = _tiBoardCommand[leg].position_des[2];
        ecatCmdLcm.dx_des[leg] = _tiBoardCommand[leg].velocity_des[0];
        ecatCmdLcm.dy_des[leg] = _tiBoardCommand[leg].velocity_des[1];
        ecatCmdLcm.dz_des[leg] = _tiBoardCommand[leg].velocity_des[2];
        ecatCmdLcm.kpx[leg] = _tiBoardCommand[leg].kp[0];
        ecatCmdLcm.kpy[leg] = _tiBoardCommand[leg].kp[1];
        ecatCmdLcm.kpz[leg] = _tiBoardCommand[leg].kp[2];
        ecatCmdLcm.kdx[leg] = _tiBoardCommand[leg].kd[0];
        ecatCmdLcm.kdy[leg] = _tiBoardCommand[leg].kd[1];
        ecatCmdLcm.kdz[leg] = _tiBoardCommand[leg].kd[2];
        ecatCmdLcm.enable[leg] = _tiBoardCommand[leg].enable;
        ecatCmdLcm.zero_joints[leg] = _tiBoardCommand[leg].zero;
        ecatCmdLcm.fx_ff[leg] = _tiBoardCommand[leg].force_ff[0];
        ecatCmdLcm.fy_ff[leg] = _tiBoardCommand[leg].force_ff[1];
        ecatCmdLcm.fz_ff[leg] = _tiBoardCommand[leg].force_ff[2];
        ecatCmdLcm.tau_abad_ff[leg] = _tiBoardCommand[leg].tau_ff[0];
        ecatCmdLcm.tau_hip_ff[leg] = _tiBoardCommand[leg].tau_ff[1];
        ecatCmdLcm.tau_knee_ff[leg] = _tiBoardCommand[leg].tau_ff[2];
        ecatCmdLcm.q_des_abad[leg] = _tiBoardCommand[leg].q_des[0];
        ecatCmdLcm.q_des_hip[leg] = _tiBoardCommand[leg].q_des[1];
        ecatCmdLcm.q_des_knee[leg] = _tiBoardCommand[leg].q_des[2];
        ecatCmdLcm.qd_des_abad[leg] = _tiBoardCommand[leg].qd_des[0];
        ecatCmdLcm.qd_des_hip[leg] = _tiBoardCommand[leg].qd_des[1];
        ecatCmdLcm.qd_des_knee[leg] = _tiBoardCommand[leg].qd_des[2];
        ecatCmdLcm.kp_joint_abad[leg] = _tiBoardCommand[leg].kp_joint[0];
        ecatCmdLcm.kp_joint_hip[leg] = _tiBoardCommand[leg].kp_joint[1];
        ecatCmdLcm.kp_joint_knee[leg] = _tiBoardCommand[leg].kp_joint[2];
        ecatCmdLcm.kd_joint_abad[leg] = _tiBoardCommand[leg].kd_joint[0];
        ecatCmdLcm.kd_joint_hip[leg] = _tiBoardCommand[leg].kd_joint[1];
        ecatCmdLcm.kd_joint_knee[leg] = _tiBoardCommand[leg].kd_joint[2];
        ecatCmdLcm.max_torque[leg] = _tiBoardCommand[leg].max_torque;
    }
    
    for(int leg = 0; leg < 4; leg++)
    {
        ecatDataLcm.x[leg] = _tiBoardData[leg].position[0];
        ecatDataLcm.y[leg] = _tiBoardData[leg].position[1];
        ecatDataLcm.z[leg] = _tiBoardData[leg].position[2];
        ecatDataLcm.dx[leg] = _tiBoardData[leg].velocity[0];
        ecatDataLcm.dy[leg] = _tiBoardData[leg].velocity[1];
        ecatDataLcm.dz[leg] = _tiBoardData[leg].velocity[2];
        ecatDataLcm.fx[leg] = _tiBoardData[leg].force[0];
        ecatDataLcm.fy[leg] = _tiBoardData[leg].force[1];
        ecatDataLcm.fz[leg] = _tiBoardData[leg].force[2];
        ecatDataLcm.q_abad[leg] = _tiBoardData[leg].q[0];
        ecatDataLcm.q_hip[leg] = _tiBoardData[leg].q[1];
        ecatDataLcm.q_knee[leg] = _tiBoardData[leg].q[2];
        ecatDataLcm.dq_abad[leg] = _tiBoardData[leg].dq[0];
        ecatDataLcm.dq_hip[leg] = _tiBoardData[leg].dq[1];
        ecatDataLcm.dq_knee[leg] = _tiBoardData[leg].dq[2];
        ecatDataLcm.tau_abad[leg] = _tiBoardData[leg].tau[0];
        ecatDataLcm.tau_hip[leg] = _tiBoardData[leg].tau[1];
        ecatDataLcm.tau_knee[leg] = _tiBoardData[leg].tau[2];
        ecatDataLcm.tau_des_abad[leg] = _tiBoardData[leg].tau_des[0];
        ecatDataLcm.tau_des_hip[leg] = _tiBoardData[leg].tau_des[1];
        ecatDataLcm.tau_des_knee[leg] = _tiBoardData[leg].tau_des[2];
        ecatDataLcm.loop_count_ti[leg] = _tiBoardData[leg].loop_count_ti;
        ecatDataLcm.ethercat_count_ti[leg] = _tiBoardData[leg].ethercat_count_ti;
        ecatDataLcm.microtime_ti[leg] = _tiBoardData[leg].microtime_ti;
    }
    
    _ecatLCM.publish("ecat_cmd", &ecatCmdLcm);
    _ecatLCM.publish("ecat_data", &ecatDataLcm);
}

/*!
 * 发送LCM可视化数据
 */
void HardwareBridge::publishVisualizationLCM() {
  cheetah_visualization_lcmt visualization_data;
  for (int i = 0; i < 3; i++) {
    visualization_data.x[i] = _mainCheetahVisualization.p[i];
  }

  for (int i = 0; i < 4; i++) {
    visualization_data.quat[i] = _mainCheetahVisualization.quat[i];
    visualization_data.rgba[i] = _mainCheetahVisualization.color[i];
  }

  for (int i = 0; i < 12; i++) {
    visualization_data.q[i] = _mainCheetahVisualization.q[i];
  }

  _visualizationLCM.publish("main_cheetah_visualization", &visualization_data);
}

/*!
 * Cheetah3HardwareBridge实例化
 */
Cheetah3HardwareBridge::Cheetah3HardwareBridge(RobotController *rc) : HardwareBridge(rc), _ecatLCM(getLcmUrl(255))
{

}

void Cheetah3HardwareBridge::run()
{
    //订阅消息
    initCommon();
    //初始化Cheetah3特定的硬件
    initHardware();
    
    //等待参数加载
    printf("[Hardware Bridge] Loading parameters over LCM...\n");
    while(!_robotParams.isFullyInitialized())
    {
        printf("[Hardware Bridge] Waiting for robot parameters...\n");
        usleep(1000000);
    }
    
    if(_userControlParameters)
    {
        while(!_userControlParameters->isFullyInitialized())
        {
            printf("[Hardware Bridge] Waiting for user parameters...\n");
            usleep(1000000);
        }
    }
    
    //开始运行
    printf("[Hardware Bridge] Got all parameters, starting up!\n");
    
    //实例化运行器 传入控制器，任务管理器 参数 名称
    _robotRunner =
            new RobotRunner(_controller, &taskManager, _robotParams.controller_dt, "robot-control");
    
    //之前获得相关参数给运行器
    _robotRunner->driverCommand = &_gamepadCommand;
    _robotRunner->tiBoardData = _tiBoardData;
    _robotRunner->tiBoardCommand = _tiBoardCommand;
    _robotRunner->robotType = RobotType::CHEETAH_3;
    _robotRunner->controlParameters = &_robotParams;
    _robotRunner->visualizationData = &_visualizationData;
    _robotRunner->cheetahMainVisualization = &_mainCheetahVisualization;
    _robotRunner->vectorNavData = &_vectorNavData;
    
    //初始化运行器
    _robotRunner->init();
    _firstRun = false;
    
    // init control thread
    
    statusTask.start();
    
    rt_ethercat_init();
    // 自定义消息传输任务开始
    PeriodicMemberFunction<Cheetah3HardwareBridge> ecatTask(
            &taskManager, .001, "ecat", &Cheetah3HardwareBridge::runEcat, this);
    ecatTask.start();
    
    // 机器人控制器开始
    _robotRunner->start();
    
    // 可视化开始
    PeriodicMemberFunction<Cheetah3HardwareBridge> visualizationLCMTask(
            &taskManager, .0167, "lcm-vis",
            &MiniCheetahHardwareBridge::publishVisualizationLCM, this);
    visualizationLCMTask.start();
    
    // rc controller disabled for now
//  _port = init_sbus(false);  // Not Simulation
//  PeriodicMemberFunction<HardwareBridge> sbusTask(
//      &taskManager, .005, "rc_controller", &HardwareBridge::run_sbus, this);
//  sbusTask.start();
    
    //主循环 定时打印任务状态
    for(;;)
    {
        usleep(100000);
        taskManager.printStatus();
        // printf("joy %f\n", _robotRunner->driverCommand->leftStickAnalog[0]);
    }
}

void MiniCheetahHardwareBridge::CyberdogProcessData()
{
    long count = 0;
    while(true)
    {
        count++;
        if(count % 200000 == 0)
        {
            count = 0;
            printf("interval:---------%.4f-------------\n", _cyberdogInterface->cyberdogData.ctrl_topic_interval);
            printf("rpy [3]:");
            for(int i = 0; i < 3; i++)
                printf(" %.2f", _cyberdogInterface->cyberdogData.rpy[i]);
            printf("\nacc [3]:");
            for(int i = 0; i < 3; i++)
                printf(" %.2f", _cyberdogInterface->cyberdogData.acc[i]);
            printf("\nquat[4]:");
            for(int i = 0; i < 4; i++)
                printf(" %.2f", _cyberdogInterface->cyberdogData.quat[i]);
            printf("\nomeg[3]:");
            for(int i = 0; i < 3; i++)
                printf(" %.2f", _cyberdogInterface->cyberdogData.omega[i]);
            printf("\nq  [12]:");
            for(int i = 0; i < 12; i++)
                printf(" %.2f", _cyberdogInterface->cyberdogData.q[i]);
            printf("\nqd [12]:");
            for(int i = 0; i < 12; i++)
                printf(" %.2f", _cyberdogInterface->cyberdogData.qd[i]);
            printf("\ntau[12]:");
            for(int i = 0; i < 12; i++)
                printf(" %.2f", _cyberdogInterface->cyberdogData.tau[i]);
            printf("\nq_des[12]:");
            for(int i = 0; i < 12; i++)
                printf(" %.2f", _cyberdogInterface->cyberdogCmd.q_des[i]);
            printf("\nqd_des[12]:");
            for(int i = 0; i < 12; i++)
                printf(" %.2f", _cyberdogInterface->cyberdogCmd.qd_des[i]);
            printf("\nkp_des[12]:");
            for(int i = 0; i < 12; i++)
                printf(" %.2f", _cyberdogInterface->cyberdogCmd.kp_des[i]);
            printf("\nkd_des[12]:");
            for(int i = 0; i < 12; i++)
                printf(" %.2f", _cyberdogInterface->cyberdogCmd.kd_des[i]);
            printf("\ntau_des[12]:");
            for(int i = 0; i < 12; i++)
                printf(" %.2f", _cyberdogInterface->cyberdogCmd.tau_des[i]);
            printf("\n\n");
        }
        
        
        //IMU
        for(int i = 0; i < 3; i++)
        {
         _vectorNavData.accelerometer(i) = _cyberdogInterface->cyberdogData.acc[i];
        // _vectorNavData.accelerometer(0) = _cyberdogInterface->cyberdogData.acc[0];
        // _vectorNavData.accelerometer(1) = _cyberdogInterface->cyberdogData.acc[1];
        // _vectorNavData.accelerometer(2) = _cyberdogInterface->cyberdogData.acc[2];
        //     std::cout<<_vectorNavData.accelerometer(0)  <<"\t""\n";
        //     std::cout<<_vectorNavData.accelerometer(1)  <<"\t""\n";
        //     std::cout<<_vectorNavData.accelerometer(2)  <<"\t""\n";
        //     std::cout<<"        " <<"\t""\n";
            
        }
        for(int i = 0; i < 4; i++)
        {
//             // 注意 Cyberdog SDK 的四元数顺序为 xyzw 需要转成 wxyz
          // _vectorNavData.quat(i) = _cyberdogInterface->cyberdogData.quat[i];
            _vectorNavData.quat[0] = _cyberdogInterface->cyberdogData.quat[0];
            std::cout<<_vectorNavData.quat[0]<<std::endl;
            _vectorNavData.quat[1] = _cyberdogInterface->cyberdogData.quat[1];
            std::cout<<_vectorNavData.quat[1]<<std::endl;
            _vectorNavData.quat[2] = _cyberdogInterface->cyberdogData.quat[2];//test
            std::cout<<_vectorNavData.quat[2]<<std::endl;
            _vectorNavData.quat[3] = _cyberdogInterface->cyberdogData.quat[3];
            std::cout<<_vectorNavData.quat[3]<<std::endl;

        }
        for(int i = 0; i < 3; i++)
        {
            _vectorNavData.gyro(i) = _cyberdogInterface->cyberdogData.omega[i];
            // _vectorNavData.gyro[0] = _cyberdogInterface->cyberdogData.omega[0];
            // _vectorNavData.gyro[1] = _cyberdogInterface->cyberdogData.omega[1];
            // _vectorNavData.gyro[2] = _cyberdogInterface->cyberdogData.omega[2];
        }
        usleep(1);
    }
}

#endif
