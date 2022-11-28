# Cheetah-Software部署到Cyberdog

环境：Ubuntu20.04

# 安装依赖

安装依赖

```bash
sudo apt install mesa-common-dev freeglut3-dev libblas-dev liblapack-dev libeigen3-dev
```

拷贝eigen3库

```bash
sudo cp -r /usr/include/eigen3  /usr/local/include/eigen3
```

安装JAVA（**需要在安装LCM前安装JAVA保证后续链接成功**）：

```bash
sudo apt install default-jdk
```

安装LCM实时通信库：

```bash
git clone https://github.com/lcm-proj/lcm.git 
cd lcm 
mkdir build && cd build
cmake .. 
make 
sudo make install 
sudo ldconfig
```

安装QT，安装完成后修改`script/find_qt_path.sh`中的对应路径

# 下载代码并生成Makefile

```bash
git clone https://github.com/mit-biomimetics/Cheetah-Software.git
cd Cheetah-Software
mkdir build && cd build
cmake .. # there are still some warnings here
./../scripts/make_types.sh # you may see an error like `rm: cannot remove...` but this is okay
```

会遇到如下错误

 ```bash
Cloning into 'googletest-src'...
fatal: invalid reference: master
CMake Error at googletest-download/googletest-download-prefix/tmp/googletest-download-gitclone.cmake:40 (message):
Failed to checkout tag: 'master'
 ```

将`common/Cmakelist.txt`第30行`master`改成`main`即可

# 编译

```bash
make -j8
```

编译会出现错误

1. ```bash
   error: ‘char* __builtin_strncpy(char*, const char*, long unsigned int)’ specified bound 2056 equals destination size [-Werror=stringop-truncation]
   ```

   将`Cmakelist.txt`里面的两个-Werror删掉

2. 提示缺少stropts.h文件

   ```bash
   cd /usr/include
   sudo touch stropts.h
   ```

3. ```bash
   error: ‘ioctl’ was not declared in this scope
   ```

   修改`robot\src\rt\rt_serial.cpp`

   ```cpp
   #define termios asmtermios
   
   //#include <asm/termios.h>
   #include<asm/ioctls.h>
   #include<asm/termbits.h>
   
   #undef termios
   
   #include<sys/ioctl.h>
   ```

重新编译

```bash
make -j8
```

如果不需要模拟，需要

```bash
cmake -DMINI_CHEETAH_BUILD=TRUE -DNO_SIM=TRUE ..
make -j8
```

# 测试

运行`common/test-common`进行测试

运行`sim/sim`启动模拟器

如果在使用WSL，提示`QOpenGLWidget: Failed to make context current`，需要修改`.bashrc`

```
export LIBGL_ALWAYS_SOFTWARE=1
unset LIBGL_ALWAYS_INDIRECT
```

# 手柄

使用Logitech F310手柄，手柄背面的模式应选择“X”，并重新连接；正面靠近模式按钮的LED应该熄灭。

# 移植到Cyberdog

## 电机SDK创建CyberdogInterface

创建`CyberdogInterface`类，继承`CustomInterface`类。由于父类的robot_data和motor_cmd都是protected，在外部无法读取，且外部引用接口时需要使用Robot_Data与Motor_Cmd结构体来创建指针，故需要为结构体创建别名以便调用

```cpp
using CyberdogData = Robot_Data;
using CyberdogCmd = Motor_Cmd;
```

并在类中添加cyberdogData和cyberdogCmd结构体，为了对应同样也使用别名结构体

```cpp
class CyberdogInterface : public CustomInterface
{
public:
    CyberdogInterface(const double &loop_rate) : CustomInterface(loop_rate) {};
    
    ~CyberdogInterface() {};
    
    CyberdogData cyberdogData;
    CyberdogCmd cyberdogCmd;

private:
    void UserCode();
};
```

`CustomInterface`类会新建一个用户线程循环执行`UserCode()`，所以只需要编写`UserCode()`函数，用来传递接收和发送的数据

```cpp
void CyberdogInterface::UserCode()
{
    cyberdogData = robot_data;
    motor_cmd = cyberdogCmd;
}
```

## 修改MIT_Controller工程

在`robot/include/RobotRunner.h`中的`RobotRunner`类中添加与Cyberdog相关的数据结构体指针

```cpp
CyberdogData *cyberdogData;
CyberdogCmd *cyberdogCmd;
```

控制器的入口在`user/MIT_Controller/main_helper.cpp`，其中执行了如下代码

```cpp
MiniCheetahHardwareBridge hw(ctrl, gMasterConfig.load_from_file);
hw.run();
```

上述代码中的`run()`函数在`robot/src/HardwareBridge.cpp`，注释掉所有与SPI、IMU相关的代码，

在`MiniCheetahHardwareBridge`类中声明指针并在创建`run()`函数中实例化`CyberdogInterface`对象

```cpp
CyberdogInterface *_cyberdogInterface = nullptr;
_cyberdogInterface = new CyberdogInterface(500);
```

将`_cyberdogInterface`中定义的`cyberdogData`与`cyberdogCmd`结构体地址赋给`_robotRunner`的对应指针

```cpp
_robotRunner->cyberdogData = &_cyberdogInterface->cyberdogData;
_robotRunner->cyberdogCmd = &_cyberdogInterface->cyberdogCmd;
```

创建新线程，执行`CyberdogProcessData()`函数，对IMU数据进行处理

```cpp
_cyberdogThread = std::thread(&MiniCheetahHardwareBridge::CyberdogProcessData, this);
```

其中`CyberdogProcessData()`函数对IMU数据进行处理分发并进行数据显示

```cpp
void MiniCheetahHardwareBridge::CyberdogProcessData()
{
    long count = 0;
    while(true)
    {
        count++;
        if(count % 100000000 == 0)
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
        }
        for(int i = 0; i < 4; i++)
        {
            // 注意 Cyberdog SDK 的四元数顺序为 xyzw 需要转成 wxyz
//            _vectorNavData.quat(i) = _cyberdogInterface->cyberdogData.quat[i];
            _vectorNavData.quat[0] = _cyberdogInterface->cyberdogData.quat[1];
            _vectorNavData.quat[1] = _cyberdogInterface->cyberdogData.quat[2];
            _vectorNavData.quat[2] = _cyberdogInterface->cyberdogData.quat[3];
            _vectorNavData.quat[3] = _cyberdogInterface->cyberdogData.quat[0];
        }
        for(int i = 0; i < 3; i++)
        {
            _vectorNavData.gyro(i) = _cyberdogInterface->cyberdogData.omega[i];
        }
        usleep(5000);
    }
}
```

在HardwareBridge`run()`函数的最后执行RobotRunner的任务，

```cpp
_robotRunner->start();
```

RobotRunner类继承PeriodicTask类，函数`start()`会先执行初始化`init()`函数，再创建一个线程，执行`loopFunction()`函数

```cpp
init();
_thread = std::thread(&PeriodicTask::loopFunction, this);
```

在`init();`函数中构建机器人模型，并初始化开机时腿的位置

```cpp
_quadruped = buildCyberdog<float>();
_jpos_initializer = new JPosInitializer<float>(3., controlParameters->controller_dt);
```

在`common/include/Dynamics`路径下新建`CyberdogParams.h`文件并新建`buildCyberdog()`函数，写入cyberdog的参数（不知道是否正确，数据来自 [A姐代码](https://github.com/zbwu/cyberdog_misc/blob/main/parameters/parameters.cpp) ）

```cpp
template<typename T>
Quadruped<T> buildCyberdog()
{
    Quadruped<T> cheetah;
    cheetah._robotType = RobotType::MINI_CHEETAH;
    
    // Parameter
    cheetah._bodyMass = 6.52;
    cheetah._bodyLength = 0.47072;
    cheetah._bodyWidth = 0.1;
    cheetah._bodyHeight = 0.1;
    cheetah._abadGearRatio = 6.0;
    cheetah._hipGearRatio = 6.0;
    cheetah._kneeGearRatio = 9.0;
    cheetah._abadLinkLength = 0.10715;
    cheetah._hipLinkLength = 0.2;
    cheetah._kneeLinkLength = 0.217;
    cheetah._kneeLinkY_offset = 0.0;
    cheetah._maxLegLength = 0.409;
    
    cheetah._motorTauMax = 3.0;
    cheetah._batteryV = 24.0;
    cheetah._motorKT = 0.05;
    cheetah._motorR = 0.173;
    cheetah._jointDamping = 0.01;
    cheetah._jointDryFriction = 0.2;
    
    /* Spatial Inertia */
    MassProperties<T> abadMassProperties;
    abadMassProperties << 5.090000033378601e-01, -2.290499862283468e-03, 9.161999914795160e-04, -2.545000053942204e-03, 3.809741465374827e-04, 6.938322330825031e-04, 4.733563982881606e-04, 5.070999577583279e-06, -1.165249886980746e-05, 1.252289894182468e-05;
    cheetah._abadInertia = SpatialInertia<T>(abadMassProperties);
    MassProperties<T> hipMassProperties;
    hipMassProperties << 6.639999747276306e-01, -1.925599877722561e-03, -2.217759937047958e-02, -1.235039997845888e-02, 3.337649162858725e-03, 2.638501580804586e-03, 1.309316023252904e-03, -9.303330443799496e-06, -1.928161655087024e-04, -7.150374585762620e-07;
    cheetah._hipInertia = SpatialInertia<T>(hipMassProperties);
    MassProperties<T> kneeMassProperties;
    kneeMassProperties << 1.140000000596046e-01, 8.093999931588769e-04, 4.559999979392160e-06, -1.281473971903324e-02, 1.455305144190788e-03, 2.152151660993695e-03, 7.054469315335155e-04, 5.125896223034943e-07, 8.388462447328493e-05, -3.237600054717404e-08;
    cheetah._kneeInertia = SpatialInertia<T>(kneeMassProperties);
    MassProperties<T> abadRotorMassProperties;
    abadRotorMassProperties << 5.499999970197678e-02, 0.000000000000000e+00, 0.000000000000000e+00, 0.000000000000000e+00, 6.299999949987978e-05, 3.300000025774352e-05, 3.300000025774352e-05, 0.000000000000000e+00, 1.311341551839262e-12, 0.000000000000000e+00;
    cheetah._abadRotorInertia = SpatialInertia<T>(abadRotorMassProperties);
    MassProperties<T> hipRotorMassProperties;
    hipRotorMassProperties << 5.499999970197678e-02, 0.000000000000000e+00, 0.000000000000000e+00, 0.000000000000000e+00, 3.300000025774352e-05, 6.299999949987978e-05, 3.300000025774352e-05, -1.311341551839262e-12, 0.000000000000000e+00, 0.000000000000000e+00;
    cheetah._hipRotorInertia = SpatialInertia<T>(hipRotorMassProperties);
    MassProperties<T> kneeRotorMassProperties;
    kneeRotorMassProperties << 5.499999970197678e-02, 0.000000000000000e+00, 0.000000000000000e+00, 0.000000000000000e+00, 3.300000025774352e-05, 6.299999949987978e-05, 3.300000025774352e-05, -1.311341551839262e-12, 0.000000000000000e+00, 0.000000000000000e+00;
    cheetah._kneeRotorInertia = SpatialInertia<T>(kneeRotorMassProperties);
    MassProperties<T> bodyMassProperties;
    bodyMassProperties << 6.519999980926514e+00, 1.284440010786057e-01, -1.564799924381077e-03, -5.737599916756153e-03, 3.205142542719841e-02, 1.370674073696136e-01, 1.494587212800980e-01, 5.662297917297110e-05, 2.728030551224947e-03, -2.321734355064109e-04;
    cheetah._bodyInertia = SpatialInertia<T>(bodyMassProperties);
    
    // locations
    cheetah._abadRotorLocation = Vec3<T>(0.23536, 0.05, 0);
    cheetah._abadLocation = Vec3<T>(cheetah._bodyLength, cheetah._bodyWidth, 0) * 0.5;
    cheetah._hipLocation = Vec3<T>(0, cheetah._abadLinkLength, 0);
    cheetah._hipRotorLocation = Vec3<T>(0, 0.04, 0);
    cheetah._kneeLocation = Vec3<T>(0, 0, -cheetah._hipLinkLength);
    cheetah._kneeRotorLocation = Vec3<T>(0, 0, 0);
    
    return cheetah;
}
```

实例化`JPosInitializer`初始化腿位置会调用`_UpdateParam()`函数，进而读取`config/initial_jpos_ctrl.yaml`文件内的初始值`target_jpos`和`mid_jpos`

```yaml
target_jpos: [
    -0.6, -1.0, 2.7,
    0.6, -1.0, 2.7,
    -0.6, -1.0, 2.7,
    0.6, -1.0, 2.7]

mid_jpos: [
    -1.8, 0., 2.7,
    1.8, 0., 2.7,
    -1.7, 0.5, 0.5,
    1.7, 0.5, 0.5]
```

回到前文，在`loopFunction()`中会循环执行`run()`函数，这个函数是一个纯虚函数，真正执行RobotRunner类中的`run()`函数。这里需要对`robot/src/RobotRunner.cpp`中两个函数进行修改

```cpp
setupStep(); // 更新来自机器人的数据到LegController对象
finalizeStep(); // 将生成的leg controller控制命令数据更新到机器人各控制系统
```

在函数`setupStep()`中，更新来自Cyberdog的数据（这里的`cyberdogData`是`_cyberdogInterface`中的数据地址，由前文传入）

```cpp
_legController->updateData(cyberdogData);
```

重载`updateData()`函数，将前文从Cyberdog获得的数据传入`datas`数组中

```cpp
template<typename T>
void LegController<T>::updateData(const CyberdogData *cyberdogData)
{
    for(int leg = 0; leg < 4; leg++)
    {
        // q: 关节角
        datas[leg].q(0) = cyberdogData->q[0 + leg * 3];
        datas[leg].q(1) = cyberdogData->q[1 + leg * 3];
        datas[leg].q(2) = cyberdogData->q[2 + leg * 3];
        
        // qd 关节角速度
        datas[leg].qd(0) = cyberdogData->qd[0 + leg * 3];
        datas[leg].qd(1) = cyberdogData->qd[1 + leg * 3];
        datas[leg].qd(2) = cyberdogData->qd[2 + leg * 3];
        
        // J and p 雅可比和足端位置
        computeLegJacobianAndPosition<T>(_quadruped, datas[leg].q, &(datas[leg].J),
                                         &(datas[leg].p), leg);
        
        // v 足端速度
        datas[leg].v = datas[leg].J * datas[leg].qd;
        
//        // tau 电机扭矩
//        datas[leg].tauEstimate(0) = cyberdogData->tau[0 + leg * 3];
//        datas[leg].tauEstimate(1) = cyberdogData->tau[1 + leg * 3];
//        datas[leg].tauEstimate(2) = cyberdogData->tau[2 + leg * 3];
    }
}
```

在函数`finalizeStep()`中，向Cyberdog发送控制命令（这里的`cyberdogCmd`是`_cyberdogInterface`中的数据地址，由前文传入）

```cpp
_legController->updateCommand(cyberdogCmd);
```

重载`updateCommand()`函数，将控制命令传入cyberdogCmd

```cpp
template<typename T>
void LegController<T>::updateCommand(CyberdogCmd *cyberdogCmd)
{
    for(int leg = 0; leg < 4; leg++)
    {
        // tauFF 初始化关节力矩
        Vec3<T> legTorque = commands[leg].tauFeedForward;
        
        // forceFF 初始化足端力
        Vec3<T> footForce = commands[leg].forceFeedForward;
        
        // 足端力 f=kp*(pDes-p)+kd*(vDes-v) (直角坐标下pd)
        footForce += commands[leg].kpCartesian * (commands[leg].pDes - datas[leg].p);
        footForce += commands[leg].kdCartesian * (commands[leg].vDes - datas[leg].v);
        
        // Torque 足端力转换成关节力矩tau = J^T*f
        legTorque += datas[leg].J.transpose() * footForce;
        
        // spi设置关节力矩
        cyberdogCmd->tau_des[0 + leg * 3] = legTorque(0); // abad
        cyberdogCmd->tau_des[1 + leg * 3] = legTorque(1); // hip
        cyberdogCmd->tau_des[2 + leg * 3] = legTorque(2); // knee
        
        // 关节空间的PD参数
        cyberdogCmd->kd_des[0 + leg * 3] = commands[leg].kdJoint(0, 0);
        cyberdogCmd->kd_des[1 + leg * 3] = commands[leg].kdJoint(1, 0);
        cyberdogCmd->kd_des[2 + leg * 3] = commands[leg].kdJoint(2, 0);
        
        cyberdogCmd->kp_des[0 + leg * 3] = commands[leg].kpJoint(0, 0);
        cyberdogCmd->kp_des[1 + leg * 3] = commands[leg].kpJoint(1, 0);
        cyberdogCmd->kp_des[2 + leg * 3] = commands[leg].kpJoint(2, 0);
        
        // 期望关节位置
        cyberdogCmd->q_des[0 + leg * 3] = commands[leg].qDes(0);
        cyberdogCmd->q_des[1 + leg * 3] = commands[leg].qDes(1);
        cyberdogCmd->q_des[2 + leg * 3] = commands[leg].qDes(2);
        // 期望关节速度
        cyberdogCmd->qd_des[0 + leg * 3] = commands[leg].qdDes(0);
        cyberdogCmd->qd_des[1 + leg * 3] = commands[leg].qdDes(1);
        cyberdogCmd->qd_des[2 + leg * 3] = commands[leg].qdDes(2);
        
        // 计算关节力矩的估计值 tauEstimate = tau +kp*(qDes-q)+kd*(qdDes-qd)
        // 等于足端受到的外界力产生的关节力加上关节模拟的关节刚度产生的关节力
        datas[leg].tauEstimate =
                legTorque +
                commands[leg].kpJoint * (commands[leg].qDes - datas[leg].q) +
                commands[leg].kdJoint * (commands[leg].qdDes - datas[leg].qd);
    }
}
```

## 修改遥控器相关代码

### 键盘控制

注意！！！测试未通过 TODO

在`robot/src/HardwareBridge.cpp`中，在`void MiniCheetahHardwareBridge::run()`函数中新建键盘读取任务

```cpp
_keyboardThread = std::thread(&MiniCheetahHardwareBridge::run_keyboard, this);
```

其中`run_keyboard()`函数内容为（由于是非阻塞读取键盘输入，需要引入头文件`#include <termios.h>`）

```cpp
extern rc_control_settings rc_control;

void HardwareBridge::run_keyboard()
{
    int c;
    while(true)
    {
        if(kbhit())
        {
            c = fgetc(stdin);
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
                default:
                    break;
            }
        }
        usleep(5000);
    }
}
```

注意，如需让机器人切换到平衡站立模式，键盘操作切换的顺序为`0 -> 6 -> 3`，不能直接从0切换到3，会触发报错。`rc_control.mode`的模式代码在`robot/include/rt/rt_rc_interface.h`文件中，课根据需要自行添加需要的模式

```cpp
namespace RC_mode
{
    constexpr int OFF = 0;
    constexpr int QP_STAND = 3;
    constexpr int BACKFLIP_PRE = 4;
    constexpr int BACKFLIP = 5;
    constexpr int VISION = 6;
    constexpr int LOCOMOTION = 11;
    constexpr int RECOVERY_STAND = 12;
    
    // Experiment Mode
    constexpr int TWO_LEG_STANCE_PRE = 20;
    constexpr int TWO_LEG_STANCE = 21;
};
```

在`robot/src/HardwareBridge.cpp`文件中，注释掉`MiniCheetahHardwareBridge::run()`函数中的遥控器任务，见上文，略

在`robot/src/RobotRunner.cpp`文件中的`RobotRunner::run()`函数中将`use_rc`设置为1

在`user/MIT_Controller/FSM_States/ControlFSM.cpp`文件中的`ControlFSM<T>::runFSM()`函数中，取消注释if(data.controlParameters->use_rc)`

### 程序自动切换状态

由于Cyberdog运控板不能接遥控器，这里使用两种方法改变运动模式

在`robot/src/HardwareBridge.cpp`文件中，注释掉`MiniCheetahHardwareBridge::run()`函数中的遥控器任务

```cpp
//    _port = init_sbus(false);  // Not Simulation
//    PeriodicMemberFunction<HardwareBridge> sbusTask(
//            &taskManager, .005, "rc_controller",
//            &HardwareBridge::run_sbus, this);
//    sbusTask.start();
```

在`robot/src/RobotRunner.cpp`文件中的`RobotRunner::run()`函数中将`use_rc`设置为0，跳过Estop()模式

```cpp
***略***
    
//使能LegController对象
_legController->setEnabled(true);

// 将use_rc设为0，跳过Estop()模式
controlParameters->use_rc = 0;

//当遥控器控制时的rc_control.mode为0时，将LegController对象的控制命令数据清零
if((rc_control.mode == 0) && controlParameters->use_rc)
{
    if(count_ini % 1000 == 0) printf("ESTOP!\n");
    
***略***
```

在`user/MIT_Controller/FSM_States/ControlFSM.cpp`文件中的`ControlFSM<T>::runFSM()`函数中，注释掉`if(data.controlParameters->use_rc)`，并令机器人模式自动切换：

```cpp
***略***

// 为了安全操作，检查机器人状态
operatingMode = safetyPreCheck();

// 状态自动切换
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

// 是否使用遥控器
//if(data.controlParameters->use_rc)
//{
//    // 设定控制模式
//    int rc_mode = data._desiredStateCommand->rcCommand->mode;
//
//    if(rc_mode == RC_mode::RECOVERY_STAND)
//    {
//        data.controlParameters->control_mode = K_RECOVERY_STAND;
//
//    }
//    else if(rc_mode == RC_mode::LOCOMOTION)
//    {
//        data.controlParameters->control_mode = K_LOCOMOTION;
//
//    }
//    else if(rc_mode == RC_mode::QP_STAND)
//    {
//        data.controlParameters->control_mode = K_BALANCE_STAND;
//
//    }
//    else if(rc_mode == RC_mode::VISION)
//    {
//        data.controlParameters->control_mode = K_VISION;
//
//    }
//    else if(rc_mode == RC_mode::BACKFLIP || rc_mode == RC_mode::BACKFLIP_PRE)
//    {
//        data.controlParameters->control_mode = K_BACKFLIP;
//    }
//    //data.controlParameters->control_mode = K_FRONTJUMP;
//    //std::cout<< "control mode: "<<data.controlParameters->control_mode<<std::endl;
//}
    
    
// 如果操作模式是安全的，则运行机器人控制代码。下面为状态机
if(operatingMode != FSM_OperatingMode::ESTOP)
{
    // 如果没有检测到过渡，则运行正常控制
    if(operatingMode == FSM_OperatingMode::NORMAL)
    {

***略***
```

## 编译执行

```bash
cmake ..
make -j16
sudo user/MIT_Controller/mit_ctrl m r f
```

第一个参数为控制器，第二个参数代表`mini cheetah`，第三个参数代表`robot`实体机器人，第四个参数f代表`file`表示从文件中获取机器人参数

## 交叉编译

如果需要在Cyberdog上运行，需要交叉编译

需要注释掉`third-party/JCQP/CholeskyDenseSolver.cpp`和`third-party/JCQP/CholeskySparseSolver.cpp`中的`#include <immintrin.h>`

### 准备工作

#### 安装依赖

安装lcm(本地部署时需要)

```
$ git clone https://github.com/lcm-proj/lcm.git
$ cd lcm
$ mkdir build && cd build
$ cmake .. && make
$ sudo make install
```

安装docker(运控部署时需要)

按照链接所附步骤进行安装：https://docs.docker.com/engine/install/ubuntu/

```
$ sudo apt-get remove docker docker-engine docker.io containerd runc
$ sudo apt-get update
$ sudo apt-get install \
    ca-certificates \
    curl \
    gnupg \
    lsb-release
$ sudo mkdir -p /etc/apt/keyrings
$ curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
$ echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
$ sudo apt-get update
$ sudo apt-get install docker-ce docker-ce-cli containerd.io docker-compose-plugin
$ sudo docker run hello-world

# 给docker设置root权限：
$ sudo groupadd docker
$ sudo usermod -aG docker $USER
$ sudo gpasswd -a $USER docker     #将登陆用户加入到docker用户组中
```

注意，如果使用WSL2，无需在Linux里安装docker，只需在windows里安装并链接到Linux。

下载交叉编译所需docker镜像

```
$ wget https://cdn.cnbj2m.fds.api.mi-img.com/os-temp/loco/loco_arm64_20220118.tar
$ docker load --input loco_arm64_20220118.tar
$ docker images
```

#### 连接机器人

将本地PC连接至铁蛋的USB download type-c 接口(位于中间)，等待出现”L4T-README” 弹窗

```
$ ping 192.168.55.100     #本地PC被分配的ip
$ ssh mi@192.168.55.1     #登录nx应用板 ,密码123
mi@lubuntu:~$ athena_version -v #核对当前版本>=1.0.0.94
$ ssh root@192.168.55.233 #登录运动控制板
```

#### 进入电机控制模式

修改配置开关，激活用户控制模式，运行用户自己的控制器：

```
$ ssh root@192.168.55.233 #登录运动控制板
root@TinaLinux:~# cd /robot
root@TinaLinux:~# ./initialize.sh #拷贝出厂代码到可读写的开发区（/mnt/UDISK/robot-software），切换到开发者模式，仅需执行一次
root@TinaLinux:~# vi /mnt/UDISK/robot-software/config/user_code_ctrl_mode.txt #切换mode:1(0:默认模式，1用户代码控制电机模式),重启机器人生效
```

### 编译及部署

#### 1、用户电脑侧部署

运行在用户pc侧(linux)难以保证实时lcm通信，仅推荐编译验证和简单的位控测试  

```
$ ping 192.168.55.233 #通过type c线连接Cyberdog的Download接口后，确认通信正常
$ ifconfig | grep -B 1 192.168.55.100 | grep "flags"| cut -d ':' -f1 #获取该ip对应网络设备，一般为usb0
$ sudo ifconfig usb0 multicast #usb0替换为上文获取的168.55.100对应网络设备,并配为多播
$ sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev usb0 #添加路由表，usb0对应替换
下载sdk
$ cd cyberdog_motor_sdk/
$ mkdir build && cd build
$ cmake ..
$ make -j4
$ ./Example_MotorCtrl
```

注：lcm通信若不成功，无法正常激活电机控制模式，log提示：Motor control mode has not been activated successfully

#### 2、铁蛋NX应用板部署

因非实时系统，仅推荐编译验证和简单位控测试

```
scp -r ~/{code_path}/Cheetah-Software mi@192.168.55.1:/home/mi #sdk源码拷入应用板，密码123
ssh mi@192.168.55.1 #登录应用板
cd /home/mi/Cheetah-Software
mkdir build && cd build
cmake ..
make -j2
ping 192.168.55.233 #测试和运控板的通信
./mit_ctrl m r f
```

#### 3、铁蛋运控板交叉编译部署

为了能使编译的文件可以直接在机器人上运行，需要在部署交叉编译工具链的docker镜像环境下编译，具体步骤如下：

```
# 如果使用小米官方的docker，需自行按照文章开头部署编译环境
docker run -it --rm --name cyberdog -v /home/fzq614/ROS_Workspaces/Cheetah-Software:/work/build_farm/workspace/cyberdog cr.d.xiaomi.net/athena/athena_cheetah_arm64:2.0 /bin/bash
# 或者使用我的docker
docker run -it --rm --name cyberdog -v /home/fzq614/ROS_Workspaces/Cheetah-Software:/work/build_farm/workspace/cyberdog cyberdog:1.0 /bin/bash

cd /work/build_farm/workspace/cyberdog/
mkdir onboard-build && cd onboard-build

cd /work/build_farm/workspace/cyberdog/onboard-build

cmake -DCMAKE_TOOLCHAIN_FILE=/usr/xcc/aarch64-openwrt-linux-gnu/Toolchain.cmake -DMINI_CHEETAH_BUILD=TRUE -DNO_SIM=TRUE ..
make -j16 #指定交叉编译工具链并编译
exit
```

编译成功后, 将生成的.so文件libcyber_dog_sdk.so和可执行文件Example_MotorCtrl拷贝到运控/mnt/UDISK目录下

```
cd ~/{sdk_path}/onboard-build

ssh root@192.168.55.233 "mkdir /mnt/UDISK/cyberdog" #在运控板内创建文件夹

执行脚本将库与可执行文件拷贝到Cyberdog
../scripts/send_to_mini_cheetah.sh user/MIT_Controller/mit_ctrl

连接到运控板
ssh root@192.168.55.233
设置so库路径变量
export LD_LIBRARY_PATH=/mnt/UDISK/cyberdog_motor_sdk/robot-software/build

或者也可以直接ssh设置
ssh root@192.168.55.233 "export LD_LIBRARY_PATH=/mnt/UDISK/cyberdog_motor_sdk/robot-software/build"

运行控制器
cd /mnt/UDISK/cyberdog_motor_sdk/robot-software/build/
./mit_ctrl m r f
#通过“nohup /mnt/UDISK/cyberdog_motor_sdk/robot-software/build/mit_ctrl m r f &”可后台运行，退出ssh连接不受影响
```

如何添加开机自启动:  
配置/mnt/UDISK/manager_config/fork_para_conf_lists.json 进程管理文件(注意结尾逗号)后重启运控程序  
例：  "600003": {"fork_config":{"name": "Example_MotorCtrl",  "object_path": "/cyberdog_motor_sdk/",  "log_path": "", "paraValues": ["", "", ""] }}  
注：手动关闭程序时，请先关闭用户程序Example_MotorCtrl，触发主程序(ctrl)超时保护趴下，再关闭或重启主程序。同时关闭主程序和用户程序，电机会因CAN总线超时位置锁定，再次启动易发生危险。

#### 错误标志位含义

```
//bit0: warning flag, lost communication between user code and robot over 10[ms]. For safety, commanded tau and qd_des will be forced to divide by (over_time[ms]/10.0);
//bit1: error flag, lost communication between user code and robot over 500[ms]. Robot will enter high-damping mode by setting joint gains kp=0, kd=10, tau=0;
//bit2: warning flag, position command of any abaduction joint changing more than 8 degrees from its previous will be truncated;
//bit3: warning flag, position command of any hip joint changing more than 10 degrees from its previous will be truncated;
//bit4: warning flag, position command of any knee joint changing more than 12 degrees from its previous will be truncated;
```

注：为了避免通信超时导致危险，报err_flag: 0x02 communicate lost over 500ms后先排除故障,关闭Example_MotorCtrl例程进程,再重启运控程序或者直接重启运控板才能清除错误.

```
# 重启运控程序:
ssh root@192.168.55.233 "ps | grep -E 'mit_ctrl' | grep -v grep | awk '{print \$1}' | xargs kill -9"
ssh root@192.168.55.233 "ps | grep -E 'manager|ctrl|imu_online' | grep -v grep | awk '{print \$1}' | xargs kill -9"
ssh root@192.168.55.233 "export LD_LIBRARY_PATH=/mnt/UDISK/robot-software/build;/mnt/UDISK/manager /mnt/UDISK/ >> /mnt/UDISK/manager_log/manager.log 2>&1 &"


ssh root@192.168.55.233 "ps | grep -E 'mit_ctrl' | grep -v grep | awk '{print \$1}' | xargs kill -9" && ssh root@192.168.55.233 "ps | grep -E 'manager|ctrl|imu_online' | grep -v grep | awk '{print \$1}' | xargs kill -9" && ssh root@192.168.55.233 "export LD_LIBRARY_PATH=/mnt/UDISK/robot-software/build;/mnt/UDISK/manager /mnt/UDISK/ >> /mnt/UDISK/manager_log/manager.log 2>&1 &"


# 重启运控板系统:
ssh root@192.168.55.233 "reboot"
```
