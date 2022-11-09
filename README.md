## Cheetah-Software
This repository contains the Robot and Simulation software project.  For a getting started guide, see the documentation folder.

The common folder contains the common library with dynamics and utilities
The resources folder will contain data files, like CAD of the robot used for the visualization
The robot folder will contain the robot program
The sim folder will contain the simulation program. It is the only program which depends on QT.
The third-party will contain *small* third party libraries that we have modified. This should just be libsoem for Cheetah 3, which Pat modified at one point.



安装依赖

```
sudo apt install mesa-common-dev freeglut3-dev libblas-dev liblapack-dev libeigen3-dev
sudo cp -r /usr/include/eigen3  /usr/local/include/eigen3
```



安装JAVA（**需要在安装LCM前安装JAVA保证后续链接成功**）：

```text
sudo apt install default-jdk
```

安装LCM实时通信库：

```text
git clone https://github.com/lcm-proj/lcm.git 
cd lcm 
mkdir build 
cd build 
cmake .. 
make 
sudo make install 
sudo ldconfig
```

安装QT，安装完成后修改`script/find_qt_path.sh`中的对应路径



## Build

To build all code:
```
mkdir build
cd build
cmake ..
./../scripts/make_types.sh
make -j4
```

`cmake ..`这一步出错，`Cloning into 'googletest-src'...
fatal: invalid reference: master
CMake Error at googletest-download/googletest-download-prefix/tmp/googletest-download-gitclone.cmake:40 (message):
  Failed to checkout tag: 'master'`，将`common/Cmakelist.txt`第30行`master`改成`main`



编译出现错误` error: ‘char* __builtin_strncpy(char*, const char*, long unsigned int)’ specified bound 2056 equals destination size [-Werror=stringop-truncation]`，将`Cmakelist.txt`里面的两个-Werror删掉



提示缺少stropts.h文件

```
cd /usr/include
sudo touch stropts.h
```



提示`error: ‘ioctl’ was not declared in this scope`，修改`robot\src\rt\rt_serial.cpp`

```
#define termios asmtermios

//#include <asm/termios.h>
#include<asm/ioctls.h>
#include<asm/termbits.h>

#undef termios

#include<sys/ioctl.h>
```



If you are building code on your computer that you would like to copy over to the mini cheetah, you must replace the cmake command with

```
cmake -DMINI_CHEETAH_BUILD=TRUE
```
otherwise it will not work.  If you are building mini cheetah code one the mini cheetah computer, you do not need to do this.

This build process builds the common library, robot code, and simulator. If you just change robot code, you can simply run `make -j4` again. If you change LCM types, you'll need to run `cmake ..; make -j4`. This automatically runs `make_types.sh`.

To test the common library, run `common/test-common`. To run the robot code, run `robot/robot`. To run the simulator, run `sim/sim`.



wsl bug`QOpenGLWidget: Failed to make context current`，修改bashrc

```
export LIBGL_ALWAYS_SOFTWARE=1
unset LIBGL_ALWAYS_INDIRECT
```



Part of this build process will automatically download the gtest software testing framework and sets it up. After it is done building, it will produce a `libbiomimetics.a` static library and an executable `test-common`.  Run the tests with `common/test-common`. This output should hopefully end with

```
[----------] Global test environment tear-down
[==========] 18 tests from 3 test cases ran. (0 ms total)
[  PASSED  ] 18 tests.
```
## Run simulator
To run the simulator:
1. Open the control board
```
./sim/sim
```
2. In the another command window, run the robot control code
```
./user/${controller_folder}/${controller_name} ${robot_name} ${target_system}
```
Example)
```
./user/JPos_Controller/jpos_ctrl 3 s
```
3: Cheetah 3, m: Mini Cheetah
s: simulation, r: robot

## Run Mini cheetah
1. Create build folder `mkdir mc-build`
2. Build as mini cheetah executable `cd mc-build; cmake -DMINI_CHEETAH_BUILD=TRUE ..; make -j`
3. Connect to mini cheetah over ethernet, verify you can ssh in
4. Copy program to mini cheetah with `../scripts/send_to_mini_cheetah.sh`
5. ssh into the mini cheetah `ssh user@10.0.0.34`
6. Enter the robot program folder `cd robot-software-....`
7. Run robot code `./run_mc.sh` 



## Dependencies:
- Qt 5.10 - https://www.qt.io/download-qt-installer
- LCM - https://lcm-proj.github.io/ (Please make it sure that you have a java to let lcm compile java-extension together)
- Eigen - http://eigen.tuxfamily.org
- `mesa-common-dev`
- `freeglut3-dev`
- `libblas-dev liblapack-dev`

To use Ipopt, use CMake Ipopt option. Ex) cmake -DIPOPT_OPTION=ON ..
