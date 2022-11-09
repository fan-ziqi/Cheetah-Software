/*!
 * @file main.cpp
 * @brief Main Function for the robot program
 * robot的启动文件
 *
 * The main function parses command line arguments and starts the appropriate
 * driver.
 */

#include <cassert>
#include <iostream>

#include "HardwareBridge.h"
#include "SimulationBridge.h"
#include "main_helper.h"
#include "RobotController.h"

MasterConfig gMasterConfig;

/*!
 * Print a message describing the command line flags for the robot program
 */
void printUsage()
{
    printf(
            "Usage: robot [robot-id] [sim-or-robot] [parameters-from-file]\n"
            "\twhere robot-id:     3 for cheetah 3, m for mini-cheetah\n"
            "\t      sim-or-robot: s for sim, r for robot\n"
            "\t      param-file:   f for loading parameters from file, l (or nothing) for LCM\n"
            "                      this option can only be used in robot mode\n");
}

/*!
 * Setup and run the given robot controller
 */
int main_helper(int argc, char **argv, RobotController *ctrl)
{
    //参数不是三个或者四个，输出提示文字
    if (argc != 3 && argc != 4)
    {
        printUsage();
        return EXIT_FAILURE;
    }

    //判断第一个参数，3代表cheetah 3, m代表 mini-cheetah
    if (argv[1][0] == '3')
    {
        gMasterConfig._robot = RobotType::CHEETAH_3;
    }
    else if (argv[1][0] == 'm')
    {
        gMasterConfig._robot = RobotType::MINI_CHEETAH;
    }
    else
    {
        printUsage();
        return EXIT_FAILURE;
    }

    //判断第二个参数，s代表模拟, r代表实物机器人
    if (argv[2][0] == 's')
    {
        gMasterConfig.simulated = true;
    }
    else if (argv[2][0] == 'r')
    {
        gMasterConfig.simulated = false;
    }
    else
    {
        printUsage();
        return EXIT_FAILURE;
    }

    //如果有第四个参数, f代表file, l代表LCM
    if (argc == 4 && argv[3][0] == 'f')
    {
        gMasterConfig.load_from_file = true;
        printf("Load parameters from file\n");
    }
    else
    {
        gMasterConfig.load_from_file = false;
        printf("Load parameters from network\n");
    }

    //输出机器人型号、是否模拟
    printf("[Quadruped] Cheetah Software\n");
    printf("        Quadruped:  %s\n",
           gMasterConfig._robot == RobotType::MINI_CHEETAH ? "Mini Cheetah"
                                                           : "Cheetah 3");
    printf("        Driver: %s\n", gMasterConfig.simulated
                                   ? "Development Simulation Driver"
                                   : "Quadruped Driver");

    // 实物和模拟执行不同的程序
    if (gMasterConfig.simulated)
    {
        if (argc != 3)
        {
            printUsage();
            return EXIT_FAILURE;
        }
        // 如果是模拟器，创建SimulationBridge的对象
        if (gMasterConfig._robot == RobotType::MINI_CHEETAH)
        {
            SimulationBridge simulationBridge(gMasterConfig._robot, ctrl);
            simulationBridge.run();
            printf("[Quadruped] SimDriver run() has finished!\n");
        }
        else if (gMasterConfig._robot == RobotType::CHEETAH_3)
        {
            SimulationBridge simulationBridge(gMasterConfig._robot, ctrl);
            simulationBridge.run();
        }
        else
        {
            printf("[ERROR] unknown robot\n");
            assert(false);
        }
    }
    else
    {
#ifdef linux
        // 如果是实物，创建四足机器人对象
        // MiniCheetahHardwareBridge和CheetahHardwareBridge是HardwareBridge的子类
        if (gMasterConfig._robot == RobotType::MINI_CHEETAH)
        {
            MiniCheetahHardwareBridge hw(ctrl, gMasterConfig.load_from_file);
            hw.run();
            printf("[Quadruped] SimDriver run() has finished!\n");
        }
        else if (gMasterConfig._robot == RobotType::CHEETAH_3)
        {
            Cheetah3HardwareBridge hw(ctrl);
            hw.run();
        }
        else
        {
            printf("[ERROR] unknown robot\n");
            assert(false);
        }
#endif
    }

    return 0;
}
