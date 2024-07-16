/*! @file LegController.cpp
 *  @brief 常用的腿部控制接口和腿部控制算法
 *
 *  为小型猎豹和猎豹3型机器人实现低水平的腿部控制
 *
 *  所有变量都相对于髋关节坐标系，它与身体坐标系具有相同的方向，但被平移到了ab/ad 关节处
 */

#include "Controllers/LegController.h"

/*!
 * 将腿命令归零，使腿不会输出扭矩（失力）
 */
template<typename T>
void LegControllerCommand<T>::zero()
{
    tauFeedForward = Vec3<T>::Zero();
    forceFeedForward = Vec3<T>::Zero();
    qDes = Vec3<T>::Zero();
    qdDes = Vec3<T>::Zero();
    pDes = Vec3<T>::Zero();
    vDes = Vec3<T>::Zero();
    kpCartesian = Mat3<T>::Zero();
    kdCartesian = Mat3<T>::Zero();
    kpJoint = Mat3<T>::Zero();
    kdJoint = Mat3<T>::Zero();
}

/*!
 * 腿部数据清零
 */
template<typename T>
void LegControllerData<T>::zero()
{
    q = Vec3<T>::Zero();
    qd = Vec3<T>::Zero();
    p = Vec3<T>::Zero();
    v = Vec3<T>::Zero();
    J = Mat3<T>::Zero();
    tauEstimate = Vec3<T>::Zero();
}

/*!
 * 将所有腿部控制命令清零。应运行在任何控制代码之前，
 * 否则如果控制代码混乱并且没有更改腿部控制命令，腿将不会记住最后一个命令。
 */
template<typename T>
void LegController<T>::zeroCommand()
{
    for(auto &cmd: commands)
    {
        cmd.zero();
    }
    _legsEnabled = false;
}

/*!
 * 用于紧急停止
 * 将腿设置为edamp。这将覆盖所有命令数据，并使用给定增益生成紧急阻尼命令。
 * 小型猎豹的edamp增益为Nm/(rad/s)，猎豹3的edamp增益为N/m。
 * 您仍然必须调用updateCommand才能使此命令最终出现在低级命令数据中
 */
template<typename T>
void LegController<T>::edampCommand(RobotType robot, T gain)
{
    zeroCommand();
    if(robot == RobotType::CHEETAH_3)
    {
        for(int leg = 0; leg < 4; leg++)
        {
            for(int axis = 0; axis < 3; axis++)
            {
                commands[leg].kdCartesian(axis, axis) = gain;
            }
        }
    }
    else
    {  // mini-cheetah
        for(int leg = 0; leg < 4; leg++)
        {
            for(int axis = 0; axis < 3; axis++)
            {
                commands[leg].kdJoint(axis, axis) = gain;
            }
        }
    }
}

/*!
 * 从Cyberdog更新腿部信息
 */
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

/*!
 * 从SPIne卡更新腿部信息
 */
template<typename T>
void LegController<T>::updateData(const SpiData *spiData)
{
    for(int leg = 0; leg < 4; leg++)
    {
        // q: 关节角
        datas[leg].q(0) = spiData->q_abad[leg];
        datas[leg].q(1) = spiData->q_hip[leg];
        datas[leg].q(2) = spiData->q_knee[leg];
        
        // qd 关节角速度
        datas[leg].qd(0) = spiData->qd_abad[leg];
        datas[leg].qd(1) = spiData->qd_hip[leg];
        datas[leg].qd(2) = spiData->qd_knee[leg];
        
        // J and p 雅可比和足端位置
        computeLegJacobianAndPosition<T>(_quadruped, datas[leg].q, &(datas[leg].J),
                                         &(datas[leg].p), leg);
        
        // v 足端速度
        datas[leg].v = datas[leg].J * datas[leg].qd;
    }
}

/*!
 * 从TI卡更新腿部信息(cheetah3使用)
 */
template<typename T>
void LegController<T>::updateData(const TiBoardData *tiBoardData)
{
    for(int leg = 0; leg < 4; leg++)
    {
        for(int joint = 0; joint < 3; joint++)
        {
            //datas 是LegControllerData
            datas[leg].q(joint) = tiBoardData[leg].q[joint];
            datas[leg].qd(joint) = tiBoardData[leg].dq[joint];
            datas[leg].p(joint) = tiBoardData[leg].position[joint];
            datas[leg].v(joint) = tiBoardData[leg].velocity[joint];
            
            // J and p
            computeLegJacobianAndPosition<T>(_quadruped, datas[leg].q, &datas[leg].J,
                                             nullptr, leg);
            datas[leg].tauEstimate[joint] = tiBoardData[leg].tau[joint];
        }
        //printf("%d leg, position: %f, %f, %f\n", leg, datas[leg].p[0], datas[leg].p[1], datas[leg].p[2]);
        //printf("%d leg, velocity: %f, %f, %f\n", leg, datas[leg].v[0], datas[leg].v[1], datas[leg].v[2]);
    }
}


/*!
 * 向Cyberdog发送腿部控制命令
 */
template<typename T>
void LegController<T>::updateCommand(CyberdogCmd *cyberdogCmd)
{
    for(int leg = 0; leg < 4; leg++)
    {
        // 见MPC论文2-D
        // tauFF 初始化关节力矩
        Vec3<T> legTorque = commands[leg].tauFeedForward;
        
        // forceFF 初始化足端力
        Vec3<T> footForce = commands[leg].forceFeedForward;
        
        // 足端力 f=kp*(pDes-p)+kd*(vDes-v) (直角坐标下pd)
        footForce += commands[leg].kpCartesian * (commands[leg].pDes - datas[leg].p);
        footForce += commands[leg].kdCartesian * (commands[leg].vDes - datas[leg].v);
        
        // Torque 足端力转换成关节力矩tau = J^T*f
        legTorque += datas[leg].J.transpose() * footForce;
        
        // 设置关节力矩
        cyberdogCmd->tau_des[0 + leg * 3] = legTorque(0); // abad
        cyberdogCmd->tau_des[1 + leg * 3] = legTorque(1); // hip
        cyberdogCmd->tau_des[2 + leg * 3] = legTorque(2); // knee
        
        // 关节空间的PD参数
        cyberdogCmd->kd_des[0 + leg * 3] = commands[leg].kdJoint(0, 0);
        cyberdogCmd->kd_des[1 + leg * 3] = commands[leg].kdJoint(1, 1);
        cyberdogCmd->kd_des[2 + leg * 3] = commands[leg].kdJoint(2, 2);
        
        cyberdogCmd->kp_des[0 + leg * 3] = commands[leg].kpJoint(0, 0);
        cyberdogCmd->kp_des[1 + leg * 3] = commands[leg].kpJoint(1, 1);
        cyberdogCmd->kp_des[2 + leg * 3] = commands[leg].kpJoint(2, 2);
        
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

/*!
 * 向SPIne板发送腿部控制命令
 */
template<typename T>
void LegController<T>::updateCommand(SpiCommand *spiCommand)
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
        spiCommand->tau_abad_ff[leg] = legTorque(0);
        spiCommand->tau_hip_ff[leg] = legTorque(1);
        spiCommand->tau_knee_ff[leg] = legTorque(2);
        
        // 关节空间的PD参数
        spiCommand->kd_abad[leg] = commands[leg].kdJoint(0, 0);
        spiCommand->kd_hip[leg] = commands[leg].kdJoint(1, 1);
        spiCommand->kd_knee[leg] = commands[leg].kdJoint(2, 2);
        
        spiCommand->kp_abad[leg] = commands[leg].kpJoint(0, 0);
        spiCommand->kp_hip[leg] = commands[leg].kpJoint(1, 1);
        spiCommand->kp_knee[leg] = commands[leg].kpJoint(2, 2);
        
        // 期望关节位置
        spiCommand->q_des_abad[leg] = commands[leg].qDes(0);
        spiCommand->q_des_hip[leg] = commands[leg].qDes(1);
        spiCommand->q_des_knee[leg] = commands[leg].qDes(2);
        // 期望关节速度
        spiCommand->qd_des_abad[leg] = commands[leg].qdDes(0);
        spiCommand->qd_des_hip[leg] = commands[leg].qdDes(1);
        spiCommand->qd_des_knee[leg] = commands[leg].qdDes(2);
        
        // 计算关节力矩的估计值 tauEstimate = tau +kp*(qDes-q)+kd*(qdDes-qd)
        // 等于足端受到的外界力产生的关节力加上关节模拟的关节刚度产生的关节力
        datas[leg].tauEstimate =
                legTorque +
                commands[leg].kpJoint * (commands[leg].qDes - datas[leg].q) +
                commands[leg].kdJoint * (commands[leg].qdDes - datas[leg].qd);
        
        spiCommand->flags[leg] = _legsEnabled ? 1 : 0;
    }
}

constexpr float CHEETAH_3_ZERO_OFFSET[4][3] = {{1.f, 4.f, 7.f},
                                               {2.f, 5.f, 8.f},
                                               {3.f, 6.f, 9.f}};

/*!
 * 向TI板发送腿部控制命令
 */
template<typename T>
void LegController<T>::updateCommand(TiBoardCommand *tiBoardCommand)
{
    // 发送四腿力矩
    for(int leg = 0; leg < 4; leg++)
    {
        Vec3<T> tauFF = commands[leg].tauFeedForward.template cast<T>();
        
        // 每个关节参数
        for(int joint = 0; joint < 3; joint++)
        {
            tiBoardCommand[leg].kp[joint] = commands[leg].kpCartesian(joint, joint);
            tiBoardCommand[leg].kd[joint] = commands[leg].kdCartesian(joint, joint);
            tiBoardCommand[leg].tau_ff[joint] = tauFF[joint];
            tiBoardCommand[leg].position_des[joint] = commands[leg].pDes[joint];
            tiBoardCommand[leg].velocity_des[joint] = commands[leg].vDes[joint];
            tiBoardCommand[leg].force_ff[joint] =
                    commands[leg].forceFeedForward[joint];
            tiBoardCommand[leg].q_des[joint] = commands[leg].qDes[joint];
            tiBoardCommand[leg].qd_des[joint] = commands[leg].qdDes[joint];
            tiBoardCommand[leg].kp_joint[joint] = commands[leg].kpJoint(joint, joint);
            tiBoardCommand[leg].kd_joint[joint] = commands[leg].kdJoint(joint, joint);
            tiBoardCommand[leg].zero_offset[joint] = CHEETAH_3_ZERO_OFFSET[leg][joint];
        }
        
        // 这里请只发送1或0，否则机器人会爆炸
        tiBoardCommand[leg].enable = _legsEnabled ? 1 : 0; // 腿使能
        tiBoardCommand[leg].max_torque = _maxTorque; // 最大力矩
        tiBoardCommand[leg].zero = _zeroEncoders ? 1 : 0; // 编码器归零
        if(_calibrateEncoders)
        {
            tiBoardCommand[leg].enable = _calibrateEncoders + 1; // 标定
        }
        
        if(_zeroEncoders)
        {
            tiBoardCommand[leg].enable = 0;
        }
        
    }
}

/*!
 * 从LegControllerCommand和LegControllerData获取数据通过 LCM 通信用于调试数据
 * datas代表实际值，是从updateData中获取的。
 * commands是命令值，是从LegControllerCommand中获取的
 */
template<typename T>
void LegController<T>::setLcm(leg_control_data_lcmt *lcmData, leg_control_command_lcmt *lcmCommand)
{
    for(int leg = 0; leg < 4; leg++)
    {
        for(int axis = 0; axis < 3; axis++)
        {
            int idx = leg * 3 + axis;
            lcmData->q[idx] = datas[leg].q[axis]; // 关节角度
            lcmData->qd[idx] = datas[leg].qd[axis]; // 关节角速度
            lcmData->p[idx] = datas[leg].p[axis]; // 足端位置
            lcmData->v[idx] = datas[leg].v[axis]; // 足端速度
            lcmData->tau_est[idx] = datas[leg].tauEstimate[axis]; // 估计的关节扭矩
            
            lcmCommand->tau_ff[idx] = commands[leg].tauFeedForward[axis]; // 前馈关节力矩
            lcmCommand->f_ff[idx] = commands[leg].forceFeedForward[axis]; // 前馈足端力
            lcmCommand->q_des[idx] = commands[leg].qDes[axis]; // 期望关节角度
            lcmCommand->qd_des[idx] = commands[leg].qdDes[axis]; // 期望关节角速度
            lcmCommand->p_des[idx] = commands[leg].pDes[axis]; // 期望足端位置
            lcmCommand->v_des[idx] = commands[leg].vDes[axis]; // 期望足端速度
            lcmCommand->kp_cartesian[idx] = commands[leg].kpCartesian(axis, axis); // 足端与外界的Kp，用于推算足端力和关节力
            lcmCommand->kd_cartesian[idx] = commands[leg].kdCartesian(axis, axis); // 足端与外界的Kd
            lcmCommand->kp_joint[idx] = commands[leg].kpJoint(axis, axis); // 关节空间的Kp
            lcmCommand->kd_joint[idx] = commands[leg].kdJoint(axis, axis); // 关节空间的Kd
        }
    }
}

template
struct LegControllerCommand<double>;
template
struct LegControllerCommand<float>;

template
struct LegControllerData<double>;
template
struct LegControllerData<float>;

template
class LegController<double>;

template
class LegController<float>;

/*!
 * 计算腿部雅克比矩阵和位置
 * 这是在局部腿部坐标系中完成的。如果J/p为NULL，则将跳过计算。
 * */
template<typename T>
void computeLegJacobianAndPosition(Quadruped<T> &quad, Vec3<T> &q, Mat3<T> *J,
                                   Vec3<T> *p, int leg)
{
    T l1 = quad._abadLinkLength;
    T l2 = quad._hipLinkLength;
    T l3 = quad._kneeLinkLength;
    T l4 = quad._kneeLinkY_offset;
    T sideSign = quad.getSideSign(leg);
    
    T s1 = std::sin(q(0));
    T s2 = std::sin(q(1));
    T s3 = std::sin(q(2));
    
    T c1 = std::cos(q(0));
    T c2 = std::cos(q(1));
    T c3 = std::cos(q(2));
    
    T c23 = c2 * c3 - s2 * s3;
    T s23 = s2 * c3 + c2 * s3;
    
    if(J)
    {
        J->operator()(0, 0) = 0;
        J->operator()(0, 1) = l3 * c23 + l2 * c2;
        J->operator()(0, 2) = l3 * c23;
        J->operator()(1, 0) = l3 * c1 * c23 + l2 * c1 * c2 - (l1 + l4) * sideSign * s1;
        J->operator()(1, 1) = -l3 * s1 * s23 - l2 * s1 * s2;
        J->operator()(1, 2) = -l3 * s1 * s23;
        J->operator()(2, 0) = l3 * s1 * c23 + l2 * c2 * s1 + (l1 + l4) * sideSign * c1;
        J->operator()(2, 1) = l3 * c1 * s23 + l2 * c1 * s2;
        J->operator()(2, 2) = l3 * c1 * s23;
    }
    
    // 足端位置使用正运动学求得
    if(p)
    {
        p->operator()(0) = l3 * s23 + l2 * s2;
        p->operator()(1) = (l1 + l4) * sideSign * c1 + l3 * (s1 * c23) + l2 * c2 * s1;
        p->operator()(2) = (l1 + l4) * sideSign * s1 - l3 * (c1 * c23) - l2 * c1 * c2;
    }
}

template void computeLegJacobianAndPosition<double>(Quadruped<double> &quad,
                                                    Vec3<double> &q,
                                                    Mat3<double> *J,
                                                    Vec3<double> *p, int leg);
template void computeLegJacobianAndPosition<float>(Quadruped<float> &quad,
                                                   Vec3<float> &q,
                                                   Mat3<float> *J,
                                                   Vec3<float> *p, int leg);
