/*!
 * @file HardwareBridge.h
 * @brief Interface between robot code and robot hardware
 *
 * This class initializes the hardware of both robots and allows the robot
 * controller to access it
 */

#ifndef PROJECT_HARDWAREBRIDGE_H
#define PROJECT_HARDWAREBRIDGE_H

#ifdef linux

#define MAX_STACK_SIZE 16384  // 16KB  of stack
#define TASK_PRIORITY 49      // linux priority, this is not the nice value

#include <string>
#include <lcm-cpp.hpp>
#include <lord_imu/LordImu.h>

#include "RobotRunner.h"
#include "Utilities/PeriodicTask.h"
#include "control_parameter_request_lcmt.hpp"
#include "control_parameter_respones_lcmt.hpp"
#include "gamepad_lcmt.hpp"
#include "microstrain_lcmt.hpp"
#include "ecat_command_t.hpp"
#include "ecat_data_t.hpp"

#include "Utilities/mahony_filter.hpp"
#include "Utilities/complementary_filter.hpp"

/*!
 * Interface between robot and hardware
 * 父类，抽象类
 */
class HardwareBridge
{
public:
    HardwareBridge(RobotController *robot_ctrl)
            : statusTask(&taskManager, 0.5f),
              _interfaceLCM(getLcmUrl(255)),
              _visualizationLCM(getLcmUrl(255))
    {
        _controller = robot_ctrl;
        _userControlParameters = robot_ctrl->getUserControlParameters();
    }
    
    void prefaultStack();
    void setupScheduler();
    void initError(const char *reason, bool printErrno = false);
    void initCommon();
    
    ~HardwareBridge()
    { delete _robotRunner; }
    
    void handleGamepadLCM(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                          const gamepad_lcmt *msg);
    
    void handleInterfaceLCM();
    void handleControlParameter(const lcm::ReceiveBuffer *rbuf,
                                const std::string &chan,
                                const control_parameter_request_lcmt *msg);
    
    void publishVisualizationLCM();
    void run_sbus();
    
    void run_keyboard();

protected:
    PeriodicTaskManager taskManager;
    PrintTaskStatus statusTask;
    GamepadCommand _gamepadCommand;
    VisualizationData _visualizationData;
    CheetahVisualization _mainCheetahVisualization;
    lcm::LCM _interfaceLCM;
    lcm::LCM _visualizationLCM;
    control_parameter_respones_lcmt _parameter_response_lcmt;
    SpiData _spiData;
    SpiCommand _spiCommand;
    TiBoardCommand _tiBoardCommand[4];
    TiBoardData _tiBoardData[4];
    
    bool _firstRun = true;
    RobotRunner *_robotRunner = nullptr;
    RobotControlParameters _robotParams;
    u64 _iterations = 0;
    std::thread _interfaceLcmThread;
    volatile bool _interfaceLcmQuit = false;
    RobotController *_controller = nullptr;
    ControlParameters *_userControlParameters = nullptr;
    
    int _port;
};

/*!
 * Interface between robot and hardware specialized for Mini Cheetah
 */
class MiniCheetahHardwareBridge : public HardwareBridge
{
public:
    MiniCheetahHardwareBridge(RobotController *rc, bool load_parameters_from_file);
    
    void CyberdogProcessData();
    void runSpi();
    void initHardware();
    void run();
    void runMicrostrain();
    void logMicrostrain();
    void abort(const std::string &reason);
    void abort(const char *reason);
private:
    void FilterImu( Vec3< float > acc_correct, Vec3< float > gyro_correct );

    //bool GetImuByteParams( Serial& serial_port, unsigned char* rec_data, unsigned char start_pos, unsigned char size );

  //  bool GetImuParamsFromMcu( Vec3< float >& a_bias, Vec3< float >& g_bias, Mat3< float >& Ta_1, Mat3< float >& Tg_1, Serial& serial_port );

    void HandleInterfaceRcRec();


private:
    VectorNavData _vectorNavData;
    lcm::LCM _spiLcm;
    lcm::LCM _microstrainLcm;
    std::thread _microstrainThread;
    LordImu _microstrainImu;
    microstrain_lcmt _microstrainData;
    bool _microstrainInit = false;
    bool _load_parameters_from_file;
    
    CyberdogInterface *_cyberdogInterface = nullptr;
    std::thread _cyberdogThread;
    std::thread _keyboardThread;
    
    ComplementaryFilter imu_filter_;
    MahonyFilter        imu_mh_filter_;

    std::mutex imu_mtx_;
    lcm::LCM   spi_lcm_;
    lcm::LCM   imu_lcm_;

   
   

    // IMU hardware realated
    bool             with_imu_;
    bool             is_imu_params_in_yaml_;
    std::thread      imu_thread_;
    bool             first_imu_ = true;


};

class Cheetah3HardwareBridge : public HardwareBridge
{
public:
    Cheetah3HardwareBridge(RobotController *rc);
    void runEcat();
    void initHardware();
    void run();
    void publishEcatLCM();
    // todo imu?

private:
    VectorNavData _vectorNavData;
    lcm::LCM _ecatLCM;
    ecat_command_t ecatCmdLcm;
    ecat_data_t ecatDataLcm;
    // nothing?
};

#endif // END of #ifdef linux
#endif  // PROJECT_HARDWAREBRIDGE_H
