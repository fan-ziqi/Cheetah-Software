//
// Created by fanziqi on 2022/11/11.
//

#ifndef CHEETAH_SOFTWARE_CYBERDOGINTERFACE_H
#define CHEETAH_SOFTWARE_CYBERDOGINTERFACE_H

#include <CustomInterface.h>
#include "cppTypes.h"

#define CYBERDOG

/*!
 * Data from Cyberdog
 */

using CyberdogData = Robot_Data;
using CyberdogCmd = Motor_Cmd;

class CyberdogInterface : public CustomInterface
{
public:
    CyberdogInterface(const double &loop_rate) : CustomInterface(loop_rate)
    {};
    
    ~CyberdogInterface()
    {};
    
    CyberdogData cyberdogData;


private:
    bool first_run = true;
    long long count = 0;
    float init_q[12];
    float target1_q[3] = {0 / 57.3, 80 / 57.3, -135 / 57.3};
    float target2_q[3] = {0 / 57.3, 45 / 57.3, -90 / 57.3};
    
    void UserCode();
};

#endif //CHEETAH_SOFTWARE_CYBERDOGINTERFACE_H
