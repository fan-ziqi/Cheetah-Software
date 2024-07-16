/*============================= FSM State =============================*/
/**
 * FSM State 基类
 */

#include "FSM_State.h"

/**
 * FSM State类的构造函数
 *
 * @param _controlFSMData 保存所有相关的控制数据
 * @param stateNameIn 枚举状态名
 * @param stateStringIn 当前FSM状态的字符串名称
 */
template<typename T>
FSM_State<T>::FSM_State(ControlFSMData<T> *_controlFSMData,
                        FSM_StateName stateNameIn, std::string stateStringIn)
        : _data(_controlFSMData),
          stateName(stateNameIn),
          stateString(stateStringIn)
{
    transitionData.zero();
    std::cout << "[FSM_State] Initialized FSM state: " << stateStringIn
              << std::endl;
}

/**
 * 直角坐标下独立控制给定腿（关节）
 *
 * @param leg 要控制的腿序号
 * @param qDes 期望关节角度
 * @param dqDes 期望关节速度
 */
template<typename T>
void FSM_State<T>::jointPDControl(
        int leg, Vec3<T> qDes, Vec3<T> qdDes)
{
    
    kpMat << 100, 0, 0, 0, 100, 0, 0, 0, 100;
    kdMat << 2, 0, 0, 0, 2, 0, 0, 0, 2;
    
    _data->_legController->commands[leg].kpJoint = kpMat;
    _data->_legController->commands[leg].kdJoint = kdMat;
    
    _data->_legController->commands[leg].qDes = qDes;
    _data->_legController->commands[leg].qdDes = qdDes;
}

/**
 * 直角坐标下独立控制给定腿（足）
 *
 * @param leg 要控制的腿序号
 * @param pDes 期望足位置
 * @param vDes 期望足速度
 * @param kp_cartesian P增益
 * @param kd_cartesian D增益
 */
template<typename T>
void FSM_State<T>::cartesianImpedanceControl(int leg, Vec3<T> pDes,
                                             Vec3<T> vDes,
                                             Vec3<double> kp_cartesian,
                                             Vec3<double> kd_cartesian)
{
    _data->_legController->commands[leg].pDes = pDes;
    // 创建直角坐标P增益矩阵
    kpMat << kp_cartesian[0], 0, 0, 0,
            kp_cartesian[1], 0, 0, 0,
            kp_cartesian[2];
    _data->_legController->commands[leg].kpCartesian = kpMat;
    
    _data->_legController->commands[leg].vDes = vDes;
    // 创建直角坐标D增益矩阵
    kdMat << kd_cartesian[0], 0, 0, 0, kd_cartesian[1], 0, 0, 0, kd_cartesian[2];
    _data->_legController->commands[leg].kdCartesian = kdMat;
}

/**
 * 步态独立的表达方式，用于选择适当的GRF和步的位置，并将其转换为腿部控制器可理解的值。
 */
template<typename T>
void FSM_State<T>::runControls()
{
    // 应该从用户界面或自主设置此选项
    int CONTROLLER_OPTION = 1;
    
    // 将力和步长重置为0
    footFeedForwardForces = Mat34<T>::Zero();
    footstepLocations = Mat34<T>::Zero();
    
    // 选择用于选择步骤位置和平衡力的控制器
    if(CONTROLLER_OPTION == 0) //给定位置控制
    {
        // 测试以确保我们可以控制机器人，这些将由控制器计算
        for(int leg = 0; leg < 4; leg++)
        {
            footFeedForwardForces.col(leg) << 0.0, 0.0, 0;  //-220.36;
            // footFeedForwardForces.col(leg) = stateEstimate.rBody *
            // footFeedForwardForces.col(leg);
            
            footstepLocations.col(leg) << 0.0, 0.0, -_data->_quadruped->_maxLegLength / 2;
        }
    }
    else if(CONTROLLER_OPTION == 1)
    {
        // QP平衡控制器
        runBalanceController();
        
        // 用试探法计算摆动脚落地位置
        for(int leg = 0; leg < 4; leg++)
        {
            footstepLocations.col(leg) << 0.0, 0.0, -_data->_quadruped->_maxLegLength / 2;
        }  // footstepHeuristicPlacement();
        
    }
    else if(CONTROLLER_OPTION == 2)
    {
        // WBC
        runWholeBodyController();
        
    }
    else if(CONTROLLER_OPTION == 3)
    {
        // cMPC
        runConvexModelPredictiveController();
        
        // 用试探法计算摆动脚落地位置
        // footstepHeuristicPlacement();
        
    }
    else if(CONTROLLER_OPTION == 4)
    {
        // RPC
        runRegularizedPredictiveController();
        
    }
    else
    {
        // 如果没有选择控制器，则将命令归零
        jointFeedForwardTorques = Mat34<float>::Zero(); // 前馈关节扭矩
        jointPositions = Mat34<float>::Zero(); // 关节角度位置
        jointVelocities = Mat34<float>::Zero(); // 关节角速度
        footFeedForwardForces = Mat34<float>::Zero(); // 脚的前馈力
        footPositions = Mat34<float>::Zero();  // 直角坐标系下脚的位置
        footVelocities = Mat34<float>::Zero(); // 直角坐标系下脚的速度
        
        // 输出错误信息
        std::cout << "[FSM_State] ERROR: No known controller was selected: "
                  << CONTROLLER_OPTION << std::endl;
    }
}

/**
 * QP平衡控制器运行
 */
template<typename T>
void FSM_State<T>::runBalanceController()
{
    double minForce = 25;
    double maxForce = 500;
    double contactStateScheduled[4];  // = {1, 1, 1, 1};
    for(int i = 0; i < 4; i++)
    {
        contactStateScheduled[i] =
                _data->_gaitScheduler->gaitData.contactStateScheduled(i);
    }
    
    // 根据接触状态来设置约束
    double minForces[4];  // = {minForce, minForce, minForce, minForce};
    double maxForces[4];  // = {maxForce, maxForce, maxForce, maxForce};
    for(int leg = 0; leg < 4; leg++)
    {
        minForces[leg] = contactStateScheduled[leg] * minForce;
        maxForces[leg] = contactStateScheduled[leg] * maxForce;
    }
    
    double COM_weights_stance[3] = {1, 1, 10};
    double Base_weights_stance[3] = {20, 10, 10};
    double pFeet[12], p_des[3], p_act[3], v_des[3], v_act[3], O_err[3], rpy[3], omegaDes[3];
    double se_xfb[13];
    double kpCOM[3], kdCOM[3], kpBase[3], kdBase[3]; //机体虚拟弹簧PD系数
    
    for(int i = 0; i < 4; i++)
    {
        // 获取机身当前四元数
        se_xfb[i] = (double) _data->_stateEstimator->getResult().orientation(i);
    }
    // se_xfb[3] = 1.0;
    for(int i = 0; i < 3; i++)
    {
        rpy[i] = 0;  //(double)_data->_stateEstimator->getResult().rpy(i);
        p_des[i] = (double) _data->_stateEstimator->getResult().position(i);
        p_act[i] = (double) _data->_stateEstimator->getResult().position(i);
        omegaDes[i] = 0;  //(double)_data->_stateEstimator->getResult().omegaBody(i);
        v_act[i] = (double) _data->_stateEstimator->getResult().vBody(i);
        v_des[i] = (double) _data->_stateEstimator->getResult().vBody(i);
        
        se_xfb[4 + i] = (double) _data->_stateEstimator->getResult().position(i);
        se_xfb[7 + i] = (double) _data->_stateEstimator->getResult().omegaBody(i);
        se_xfb[10 + i] = (double) _data->_stateEstimator->getResult().vBody(i);
        
        // 设置平移和方向增益
        kpCOM[i] = (double) _data->controlParameters->kpCOM(i);
        kdCOM[i] = (double) _data->controlParameters->kdCOM(i);
        kpBase[i] = (double) _data->controlParameters->kpBase(i);
        kdBase[i] = (double) _data->controlParameters->kdBase(i);
    }
    
    Vec3<T> pFeetVec;
    Vec3<T> pFeetVecCOM;
    // 获取脚相对于COM的位置
    for(int leg = 0; leg < 4; leg++)
    {
        computeLegJacobianAndPosition(**&_data->_quadruped,
                                      _data->_legController->datas[leg].q,
                                      (Mat3<T> *) nullptr, &pFeetVec, 1);
        //pFeetVecCOM = _data->_stateEstimator->getResult().rBody.transpose() *
        //(_data->_quadruped->getHipLocation(leg) + pFeetVec);
        
        pFeetVecCOM = _data->_stateEstimator->getResult().rBody.transpose() *
                      (_data->_quadruped->getHipLocation(leg) + _data->_legController->datas[leg].p);
        
        
        pFeet[leg * 3] = (double) pFeetVecCOM[0];
        pFeet[leg * 3 + 1] = (double) pFeetVecCOM[1];
        pFeet[leg * 3 + 2] = (double) pFeetVecCOM[2];
    }
    
    balanceController.set_alpha_control(0.01);
    balanceController.set_friction(0.5);
    balanceController.set_mass(46.0);
    balanceController.set_wrench_weights(COM_weights_stance, Base_weights_stance);
    balanceController.set_PDgains(kpCOM, kdCOM, kpBase, kdBase);
    balanceController.set_desiredTrajectoryData(rpy, p_des, omegaDes, v_des);
    balanceController.SetContactData(contactStateScheduled, minForces, maxForces);
    balanceController.updateProblemData(se_xfb, pFeet, p_des, p_act, v_des, v_act,
                                        O_err, 0.0);
    
    double fOpt[12]; //解出的足力
    balanceController.solveQP_nonThreaded(fOpt);
    
    // 通过LCM发布结果
    balanceController.publish_data_lcm();
    
    // 将结果复制到前馈力
    for(int leg = 0; leg < 4; leg++)
    {
        footFeedForwardForces.col(leg) << (T) fOpt[leg * 3], (T) fOpt[leg * 3 + 1], (T) fOpt[leg * 3 + 2];
    }
}

/**
 * Gait independent formulation for choosing appropriate GRF and step locations
 * as well as converting them to leg controller understandable values.
 */
template<typename T>
void FSM_State<T>::turnOnAllSafetyChecks()
{
    // Pre controls safety checks
    checkSafeOrientation = true;  // check roll and pitch
    
    // Post control safety checks
    checkPDesFoot = true;          // do not command footsetps too far
    checkForceFeedForward = true;  // do not command huge forces
    checkLegSingularity = true;    // do not let leg
}

/**
 *
 */
template<typename T>
void FSM_State<T>::turnOffAllSafetyChecks()
{
    // Pre controls safety checks
    checkSafeOrientation = false;  // check roll and pitch
    
    // Post control safety checks
    checkPDesFoot = false;          // do not command footsetps too far
    checkForceFeedForward = false;  // do not command huge forces
    checkLegSingularity = false;    // do not let leg
}

// template class FSM_State<double>;
template
class FSM_State<float>;
