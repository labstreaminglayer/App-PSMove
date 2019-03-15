# Lab Streaming Layer Client for PSMoveService
LSL application to stream data from PSMoveService-Client

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