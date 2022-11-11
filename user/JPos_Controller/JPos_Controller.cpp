#include "JPos_Controller.hpp"

/**
 * 控制器的“runController”函数将以1 kHz的频率自动调用，从该类中基本可以访问所有数据。
 * runController定义在\robot\include\RobotController.h中
 * */
void JPos_Controller::runController()
{
    // 定义3*3矩阵kpMat和kdMat。以common文件夹中Eigen::Matrix<T, 3, 3>为类模板
    Mat3<float> kpMat;
    Mat3<float> kdMat;
    //kpMat << 20, 0, 0, 0, 20, 0, 0, 0, 20;
    //kdMat << 2.1, 0, 0, 0, 2.1, 0, 0, 0, 2.1;
    // 如果使用了用户配置参数，则将用户配置参数赋值给kpMat和kdMat矩阵，按照行顺序依次赋值
    kpMat << userParameters.kp, 0, 0, 0, userParameters.kp, 0, 0, 0, userParameters.kp;
    kdMat << userParameters.kd, 0, 0, 0, userParameters.kd, 0, 0, 0, userParameters.kd;
    
    // 定义一个计数的迭代器
    static int iter(0);
    ++iter;
    // 将每一条腿leg的每一个关节jidx的位置q赋值给_jpos_ini矩阵，采样10次
    if(iter < 10)
    {
        for(int leg(0); leg < 4; ++leg)
        {
            for(int jidx(0); jidx < 3; ++jidx)
            {
                _jpos_ini[3 * leg + jidx] = _legController->datas[leg].q[jidx];
            }
        }
    }
    
    //设定腿部控制器最大扭矩值为150，并使能腿部驱动
    _legController->_maxTorque = 150;
    _legController->_legsEnabled = true;
    
    // 如果calibrate>0.4，则将值赋给编码器校准
    if(userParameters.calibrate > 0.4)
    {
        _legController->_calibrateEncoders = userParameters.calibrate;
    }
    else
    {
        // 如果calibrate<0.4且zero>0.5，_zeroEncoders为true
        if(userParameters.zero > 0.5)
        {
            _legController->_zeroEncoders = true;
        }
        else
        {
            // 如果calibrate<0.4且zero<0.5，_zeroEncoders为false,并开始发送腿部位置指令
            _legController->_zeroEncoders = false;
            
            for(int leg(0); leg < 4; ++leg)
            {
                for(int jidx(0); jidx < 3; ++jidx)
                {
                    // pos以sin函数赋值，函数运行频率为1k，sin函数变化周期为2*pi
                    float pos = std::sin(.001f * iter);
                    // 将pos的值赋给各个关节
                    _legController->commands[leg].qDes[jidx] = pos;
                    // 各个关节角速度为0
                    _legController->commands[leg].qdDes[jidx] = 0.;
                    // 各个关节前馈力矩的大小，关节扭矩tau与足端力关系tau=J^T * f，足端力f = inv(J^T) * tau。前馈用于补偿一分部关节力用于计算足端力，实现方法在LegController.cpp中。
                    _legController->commands[leg].tauFeedForward[jidx] = userParameters.tau_ff;
                }
                // 将设定的pd参数赋给控制器
                _legController->commands[leg].kpJoint = kpMat;
                _legController->commands[leg].kdJoint = kdMat;
            }
        }
    }
    
    
    
    //if(iter%200 ==0){
    //printf("value 1, 2: %f, %f\n", userParameters.testValue, userParameters.testValue2);
    //}
    
    
}
