# Lab Streaming Layer Client for PSMoveService
LSL application to stream data from PSMoveService-Client

There are still some problems:

* The srate is IRREGULAR
    * Position might be polled faster than the data can change.
	* Each sample is pushed independently so the timestamps could be reliable.
* The streams are not properly cleaned up so there might be a memory leak when stopping/starting.

# Build

## Windows

* Using vcpkg, install qt5 for your target architecture
    * vcpkg install qt5:x64-windows
* Right click on the folder and choose "Open in Visual Studio".
* Use the CMake menu to edit the configuration, and add the following section (change vcpkg location accordingly)
    ```
    ,
        "variables": [{
          "name": "CMAKE_TOOLCHAIN_FILE",
          "value": "E:\\SachsLab\\Tools\\Misc\\vcpkg\\scripts\\buildsystems\\vcpkg.cmake"
        }]
    ```