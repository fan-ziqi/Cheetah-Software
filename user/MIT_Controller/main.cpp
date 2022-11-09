/*!
 * @file main.cpp
 * @brief Main Function for the WBC Controller
 * WBC的程序入口
 *
 * The main function parses command line arguments and starts the appropriate
 * driver.
 */

#include <main_helper.h>
#include "MIT_Controller.hpp"

int main(int argc, char **argv)
{
    // new一个MIT_Controller对象，MIT_Controller继承了RobotController
    // 然后调用main_helper()
    main_helper(argc, argv, new MIT_Controller());
    return 0;
}
