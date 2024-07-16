/*! @file LegController.h
 *  @brief Common Leg Control Interface and Leg Control Algorithms
 *
 *  Implements low-level leg control for Mini Cheetah and Cheetah 3 Robots
 *  Abstracts away the difference between the SPIne and the TI Boards (the low level leg control boards)
 *  All quantities are in the "leg frame" which has the same orientation as the
 * body frame, but is shifted so that 0,0,0 is at the ab/ad pivot (the "hip
 * frame").
 */

#ifndef PROJECT_LEGCONTROLLER_H
#define PROJECT_LEGCONTROLLER_H

#include "cppTypes.h"
#include "leg_control_command_lcmt.hpp"
#include "leg_control_data_lcmt.hpp"
#include "Dynamics/Quadruped.h"
#include "SimUtilities/SpineBoard.h"
#include "SimUtilities/ti_boardcontrol.h"

#include "CyberdogInterface.h"

/*!
 * 控制器算法下发的执行腿部控制的指令
 */
template<typename T>
struct LegControllerCommand
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    LegControllerCommand()
    { zero(); }
    
    void zero();
    
    // 前馈关节力矩tauFeedForward
    // 前馈足端力forceFeedForward
    // 期望关节位置qDes
    // 期望关节角速度qdDes
    // 期望足端位置pDes
    // 期望足端速度vDes
    Vec3<T> tauFeedForward, forceFeedForward, qDes, qdDes, pDes, vDes;
    // 足端和关节的PD控制器的参数
    Mat3<T> kpCartesian, kdCartesian, kpJoint, kdJoint;
};

/*!
 * 从腿部获取到的实际数据
 */
template<typename T>
struct LegControllerData
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    LegControllerData()
    { zero(); }
    
    void setQuadruped(Quadruped<T> &quad)
    { quadruped = &quad; }
    
    void zero();
    
    // 关节角度 关节角速度 足端位置 足端速度
    Vec3<T> q, qd, p, v;
    // 雅可比
    Mat3<T> J;
    // 估计力矩反馈
    Vec3<T> tauEstimate;
    // 机器人类型 cheetah3 or mini
    Quadruped<T> *quadruped;
};

/*!
 * 四足4条腿的控制器。适用于mini猎豹和猎豹3
 */
template<typename T>
class LegController
{
public:
    LegController(Quadruped<T> &quad) : _quadruped(quad)
    {
        for(auto &data: datas) data.setQuadruped(_quadruped);
    }
    
    void zeroCommand(); //腿部控制命令清零
    void edampCommand(RobotType robot, T gain);
    void updateData(const CyberdogData *cyberdogData);
    void updateCommand(CyberdogCmd *cyberdogCmd);
    void updateData(const SpiData *spiData);
    void updateCommand(SpiCommand *spiCommand);
    void updateData(const TiBoardData *tiBoardData);
    void updateCommand(TiBoardCommand *tiBoardCommand);
    
    
    void setEnabled(bool enabled)
    { _legsEnabled = enabled; };
    void setLcm(leg_control_data_lcmt *data, leg_control_command_lcmt *command);
    
    /*!
     * Set the maximum torque.  This only works on cheetah 3!
     */
    void setMaxTorqueCheetah3(T tau)
    { _maxTorque = tau; }
    
    LegControllerCommand<T> commands[4];
    LegControllerData<T> datas[4];
    Quadruped<T> &_quadruped;
    bool _legsEnabled = false;
    T _maxTorque = 0;
    bool _zeroEncoders = false;
    u32 _calibrateEncoders = 0;
};

// 计算腿部雅克比矩阵和位置
template<typename T>
void computeLegJacobianAndPosition(Quadruped<T> &quad, Vec3<T> &q, Mat3<T> *J,
                                   Vec3<T> *p, int leg);

#endif  // PROJECT_LEGCONTROLLER_H
