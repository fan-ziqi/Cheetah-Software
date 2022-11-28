//
// Created by fanziqi on 2022/11/11.
//

#ifndef CHEETAH_SOFTWARE_CYBERDOGINTERFACE_H
#define CHEETAH_SOFTWARE_CYBERDOGINTERFACE_H

#include <CustomInterface.h>
#include "cppTypes.h"

#define CYBERDOG
//#define USE_SIM
#define USE_RC
#define USE_KEYBOARD

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
    CyberdogCmd cyberdogCmd;


private:
    bool first_run = true;
    long long count = 0;
    
    void UserCode();
};

#endif //CHEETAH_SOFTWARE_CYBERDOGINTERFACE_H
