# realsense-odroid

odroid ubuntu + realsense sdk + opencv + qt creator

# Flashing an Ubuntu Image on an OSX Computer

1. Download ubuntu from [this page](https://wiki.odroid.com/odroid-xu4/os_images/linux/ubuntu_4.14/20171213). The systsem I downloaded is Ubuntu 16.04.3 LTS (v1.0).

2. Uncompress the image.

   `xz -d <path-to-compressed-image-file>`
   
   If you don't have xzutils package, download from [xzutils](http://tukaani.org/xz/)
   
3. Check the disk name on your computer, unmount it, and flash it by following:

    `diskutil list`
    
    `diskutil unmountdisk /dev/diskX`
    
    `sudo dd of=/dev/diskX bs=1m if=<path-to-uncompressed-image-file>`
    
    
    USB adapter’s device name is in the format /dev/diskX, make sure you change it accordingly.
    
 # Install realsense sdk 2.0
 Follow [this link](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md) with some changes. This may only work with Ubuntu 16.04
 
 1. Ubuntu Build Dependencies.
 
    - Install git. `sudo apt-get install git`
  
    - Install cmake3 from source.</br>
        `sudo apt-get install build-essential` </br>
        `wget http://www.cmake.org/files/v3.11/cmake-3.11.0-rc4.tar.gz`</br>
        `tar xf cmake-3.11.0-rc4.tar.gz`</br>
        `cd cmake-3.11.0-rc4`</br>
        `./configure`</br>
        `make`</br>
        `make install`</br>
        
 2. Make Ubuntu Up to Date
 
    `sudo apt-get update && sudo apt-get upgrade && sudo apt-get dist-upgrade`</br>
    `sudo reboot`</br>
    
 3. Video4Linux backend preparation 
 
    - Ensure no Intel RealSense cameras are plugged in</br>
    - Install openssl package required for kernel module build</br>
      `sudo apt-get install libssl-dev` </br>
    - Install udev rules located in librealsense source directory</br>
      `cd librealsense`</br>
      `sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/`</br>
      `sudo udevadm control --reload-rules && udevadm trigger`</br>
    - Build and apply patched kernel modules for **Ubuntu 14/16 LTS**</br>
      `./scripts/patch-realsense-ubuntu-xenial.sh`</br>
    - Check installation by examining the patch-generated log as well as inspecting the latest entries in kernel log</br>
      `sudo dmesg | tail -n 50`</br>
       The log should indicate that a new uvcvideo driver has been registered.</br>
    - Install the packages required for librealsense build</br>
      `sudo apt-get install libusb-1.0-0-dev pkg-config libgtk-3-dev`</br>
      `sudo apt-get install libglfw3-dev`</br>
    - TM1-specifics</br>
      `echo 'hid_sensor_custom' | sudo tee -a /etc/modules`</br>
      
4. Building librealsense2 SDK

    - Navigate to librealsense root directory</br>
      `cd librealsense`</br>
      `mkdir build && cd build`</br>
    - Run CMake</br>
      `cmake ../ -DBUILD_EXAMPLES=true`</br>
      Builds librealsense along with the demos and tutorials, see CMake [other options](https://github.com/Chunhai/librealsense/blob/232f6ee82f21e3215c42b1bde6837c62b89e23d5/doc/installation.md)</br>
    - open librealsense/src/image.cpp, comment line 20-35, and replace with the code as following.</br>
       ~~#ifdef _WIN32 </br>
       #define cpuid(info, x)    __cpuidex(info, x, 0)</br>
       #else</br>
       #include <cpuid.h></br>
       void cpuid(int info[4], int info_type){</br>
           __cpuid_count(info_type, 0, info[0], info[1], info[2], info[3]);</br>
       }</br>
       #endif~~</br>

       ~~bool has_avx()</br>
       {</br>
           int info[4];</br>
           cpuid(info, 0);</br>
           cpuid(info, 0x80000000);</br>
           return (info[2] & ((int)1 << 28)) != 0;</br>
        }~~</br>
    
        bool has_avx() { return false; }
      
     - Recompile and install librealsense binaries</br>
       `sudo make uninstall && make clean && make && sudo make install`</br>
       
# Install qt creator

1. Install via apt-get</br>
   `sudo apt-get install qt5-default qtcreator`
   
2. Launch qt</br>
    `qtcreator -noload Welcome `
    
3. Configuration </br>
   **“Tools” - “Option” - “Build & Run”**</br>
   - Add Device</br>
     Fill in the information as needed. Check your IP by `ifconfig`. A example is as following:</br>
     >The name to identify this configuration: Odroid</br>
     >The device hostname or IP address: 204.12.0.100</br>
     >The username to log into this device: odroid</br>
     >The user's passwork: *****</br>
   - Add Compiler</br>
     Set name. For example, GCC(odroid)
     Set compiler path as `/usr/bin/arm-linux-gnueabihf-g++`</br>
   - Add Kit</br>
     Set "Device type" as "Generic Linux Device"</br>
     Set "Device" as the device you just added</br>
     Set "Compiler" as the one you just added</br>
     
 4. Creat a test project</br>
    **"File" - "New File or Project" - "Non-qt Project" - "Console"**</br>
    This is a hello word test case, before compile and run, open **.pro** file and add the followings</br>
    >INSTALLS        = target</br>
    >target.files    = test      <-------------------- project name</br>
    >target.path     = /home/odroid/projects_exe <--------Path that stores the exe file in a different location, should be different from the path where exe file is generated</br>
    
    In order to compile and run realsense library, add the static lib.</br>
    > LIBS += -L/usr/local/lib -lrealsense2</br>

   
# Install opencv

  - Install Dependencies</br>
    `sudo apt-get install libeigen3-dev libvtk6-dev v4l-utils`
    
  - Install opencv3.4</br>
    Follow [official guide](https://docs.opencv.org/trunk/d7/d9f/tutorial_linux_install.html)
    
  - Run `export OpenCV_DIR=~/opencv3.4/build`
  
  - Configure the environment(Thanks to [GaoHongchen](https://github.com/GaoHongchen/DIPDemo/issues/1))</br>
    (1) Find the folder containing **libopencv_core.so.3.2** </br>
        `sudo find / -name "libopencv_core.so.3.2*"`</br>
    (2) Create a file called /etc/ld.so.conf.d/opencv.conf and write to it the path to the folder where the binary is stored.For example, I wrote /usr/local/lib/ to my opencv.conf file</br>
    (3) `sudo ldconfig -v`
  - Configure qt
    For linker purpose, add the "opencv" library path</br>
    > LIBS += -L/usr/local/lib -lopencv_core -lopencv_imgcodecs -lopencv_highgui</br>
    or</br>
    > LIBS += `pkg-config opencv --libs`</br>
    or</br>
    > UNIX {
    >  CONFIG += link_pkgconfig
    >  PKGCONFIG += opencv
    >  }
    
