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
make -j4
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
make -j4
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

## 创建CyberdogInterface

创建`CyberdogInterface`类，继承`CustomInterface`类。由于父类的Robot_Data和Motor_Cmd都是protected，故需要：

```cpp
using CyberdogData = Robot_Data;
using CyberdogCmd = Motor_Cmd;
```

在类中添加CyberdogData和CyberdogCmd结构体

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

`CustomInterface`类会新建一个用户线程，所以只需要编写`UserCode()`，用来接收数据和准备发送的数据

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

在`robot/include/HardwareBridge.h`中的`HardwareBridge`类中添加与Cyberdog相关的数据结构体

```cppp
CyberdogData _cyberdogData;
CyberdogCmd _cyberdogCmd;
```

控制器的入口在`user/MIT_Controller/main_helper.cpp`，其中执行了如下代码

```cpp
MiniCheetahHardwareBridge hw(ctrl, gMasterConfig.load_from_file);
hw.run();
```

上述代码中的`run()`函数在`robot/src/HardwareBridge.cpp`，注释掉所有与SPI、IMU相关的代码，

将与Cyberdog相关的数据结构体指针赋给`_robotRunner`的对应指针

```cpp
_robotRunner->cyberdogData = &_cyberdogData;
_robotRunner->cyberdogCmd = &_cyberdogCmd;
```

创建`CyberdogInterface`对象，并创建线程，执行`runCyberdog()`函数

```cpp
_cyberdogInterface = new CyberdogInterface(500);
_cyberdogThread = std::thread(&MiniCheetahHardwareBridge::runCyberdog, this);
```

其中`runCyberdog()`函数负责将_cyberdogInterface中的cyberdogData和cyberdogCmd地址传到前文定义的指针

```cpp
void MiniCheetahHardwareBridge::runCyberdog()
{
    while(true)
    {
        _cyberdogData = _cyberdogInterface->cyberdogData;
        _cyberdogCmd = _cyberdogInterface->cyberdogCmd;
        
        //IMU
        for(int i = 0; i < 3; i++)
        {
            _vectorNavData.accelerometer(i) = _cyberdogData.acc[i];
        }
        for(int i = 0; i < 4; i++)
        {
            _vectorNavData.quat(i) = _cyberdogData.quat[i];
        }
        for(int i = 0; i < 3; i++)
        {
            _vectorNavData.gyro(i) = _cyberdogData.omega[i];
        }
    }
}
```

在HardwareBridge`run()`函数的最后执行RobotRunner的任务，

```cpp
_robotRunner->start();
```

RobotRunner类继承PeriodicTask类，函数`start()`会创建一个线程，执行`loopFunction()`函数

```cpp
_thread = std::thread(&PeriodicTask::loopFunction, this);
```

在`loopFunction()`中会循环执行`run()`函数，这个函数是一个纯虚函数，真正执行RobotRunner类中的`run()`函数。这里需要对`robot/src/RobotRunner.cpp`中两个函数进行修改

```cpp
setupStep(); // 更新来自机器人的数据到LegController对象
finalizeStep(); // 将生成的leg controller控制命令数据更新到机器人各控制系统
```

在函数`setupStep()`中，更新来自Cyberdog的数据

```cpp
_legController->updateData(cyberdogData);
```

重载`updateData()`函数，将前文从Cyberdog获得的数据传入datas数组中

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
        
        // tau 电机扭矩
        datas[leg].tauEstimate(0) = cyberdogData->tau[0 + leg * 3];
        datas[leg].tauEstimate(1) = cyberdogData->tau[1 + leg * 3];
        datas[leg].tauEstimate(2) = cyberdogData->tau[2 + leg * 3];
    }
}
```

在函数`finalizeStep()`中，向Cyberdog发送控制命令

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
    }
}
```

## 编译执行

```bash
cmake ..
make -j16
sudo user/MIT_Controller/mit_ctrl m r f
```

第一个参数为控制器，第二个参数代表`mini cheetah`，第三个参数代表`robot`实体机器人，第四个参数f代表`file`表示从文件中获取机器人参数

未完待续
