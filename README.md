# Cheetah-Software部署

环境：Ubuntu20.04

# 安装依赖

安装依赖

```bash
sudo apt install mesa-common-dev freeglut3-dev libblas-dev liblapack-dev libeigen3-dev
```

拷贝eigen3库

```bash
sudo cp -r /usr/include/eigen3  /usr/local/include/eigen3
```

安装JAVA（**需要在安装LCM前安装JAVA保证后续链接成功**）：

```bash
sudo apt install default-jdk
```

安装LCM实时通信库：

```bash
git clone https://github.com/lcm-proj/lcm.git 
cd lcm 
mkdir build && cd build
cmake .. 
make 
sudo make install 
sudo ldconfig
```

安装QT，安装完成后修改`script/find_qt_path.sh`中的对应路径

# 下载代码并生成Makefile

```bash
git clone https://github.com/mit-biomimetics/Cheetah-Software.git
cd Cheetah-Software
mkdir build && cd build
cmake .. # there are still some warnings here
./../scripts/make_types.sh # you may see an error like `rm: cannot remove...` but this is okay
```

会遇到如下错误

 ```bash
Cloning into 'googletest-src'...
fatal: invalid reference: master
CMake Error at googletest-download/googletest-download-prefix/tmp/googletest-download-gitclone.cmake:40 (message):
Failed to checkout tag: 'master'
 ```

将`common/Cmakelist.txt`第30行`master`改成`main`即可

# 编译

```bash
make -j4
```

编译会出现错误

1. ```bash
   error: ‘char* __builtin_strncpy(char*, const char*, long unsigned int)’ specified bound 2056 equals destination size [-Werror=stringop-truncation]
   ```

   将`Cmakelist.txt`里面的两个-Werror删掉

2. 提示缺少stropts.h文件

   ```bash
   cd /usr/include
   sudo touch stropts.h
   ```

3. ```bash
   error: ‘ioctl’ was not declared in this scope
   ```

   修改`robot\src\rt\rt_serial.cpp`

   ```cpp
   #define termios asmtermios
   
   //#include <asm/termios.h>
   #include<asm/ioctls.h>
   #include<asm/termbits.h>
   
   #undef termios
   
   #include<sys/ioctl.h>
   ```

重新编译

```bash
make -j4
```

# 测试

运行`common/test-common`进行测试

运行`sim/sim`启动模拟器

如果在使用WSL，提示`QOpenGLWidget: Failed to make context current`，需要修改`.bashrc`

```
export LIBGL_ALWAYS_SOFTWARE=1
unset LIBGL_ALWAYS_INDIRECT
```

# 手柄

使用Logitech F310手柄，手柄背面的模式应选择“X”，并重新连接；正面靠近模式按钮的LED应该熄灭。

# 移植到Cyberdog

