# LabStreamingLayer PSMoveClient application for PSMoveService

## Build Instructions

### Requirements

* [CMake](https://cmake.org/download/)
* [Qt5](https://www.qt.io/download-open-source/)
* PSMoveService (specifically the CAPI)
* liblsl
* Build environment
    * Tested with MSVC 2015 Win64

### Configure Project

#### Alongside root LSL project

TODO

#### After root LSL is built

TODO

#### Standalone

* Windows
    * Right-click on folder and choose "Open in Visual Studio". Wait a minute or two for CMake to generate the project.
    See [here](https://docs.microsoft.com/en-us/cpp/ide/cmake-tools-for-visual-cpp?view=vs-2017#ide-integration).
    * Using the dropdown menu, make sure your configuration is set to x64-Release (or Debug if so desired).
    * Set your CMake configuration settings from the cmake menu
        ```
        ,
            "variables": [{
              "name": "CMAKE_TOOLCHAIN_FILE",
              "value": "D:\\Tools\\Misc\\vcpkg\\scripts\\buildsystems\\vcpkg.cmake"
            }]
        ```

### Build


## Usage Instructions

## Known Issues
