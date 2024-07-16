//
// Created by fanziqi on 2022/11/11.
//

#include <CyberdogInterface.h>

void CyberdogInterface::UserCode()
{
    cyberdogData = robot_data;
    motor_cmd = cyberdogCmd;

//    if((count++) % 1000 == 0)
//    {
//        printf("interval:---------%.4f-------------\n", robot_data.ctrl_topic_interval);
//        printf("rpy [3]:");
//        for(int i = 0; i < 3; i++)
//            printf(" %.2f", robot_data.rpy[i]);
//        printf("\nacc [3]:");
//        for(int i = 0; i < 3; i++)
//            printf(" %.2f", robot_data.acc[i]);
//        printf("\nquat[4]:");
//        for(int i = 0; i < 4; i++)
//            printf(" %.2f", robot_data.quat[i]);
//        printf("\nomeg[3]:");
//        for(int i = 0; i < 3; i++)
//            printf(" %.2f", robot_data.omega[i]);
//        printf("\nq  [12]:");
//        for(int i = 0; i < 12; i++)
//            printf(" %.2f", robot_data.q[i]);
//        printf("\nqd [12]:");
//        for(int i = 0; i < 12; i++)
//            printf(" %.2f", robot_data.qd[i]);
//        printf("\ntau[12]:");
//        for(int i = 0; i < 12; i++)
//            printf(" %.2f", robot_data.tau[i]);
//        printf("\nq_des[12]:");
//        for(int i = 0; i < 12; i++)
//            printf(" %.2f", motor_cmd.q_des[i]);
//        printf("\nqd_des[12]:");
//        for(int i = 0; i < 12; i++)
//            printf(" %.2f", motor_cmd.qd_des[i]);
//        printf("\nkp_des[12]:");
//        for(int i = 0; i < 12; i++)
//            printf(" %.2f", motor_cmd.kp_des[i]);
//        printf("\nkd_des[12]:");
//        for(int i = 0; i < 12; i++)
//            printf(" %.2f", motor_cmd.kd_des[i]);
//        printf("\ntau_des[12]:");
//        for(int i = 0; i < 12; i++)
//            printf(" %.2f", motor_cmd.tau_des[i]);
//        printf("\n\n");
//    }
}
