| Supported Targets | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-C6 | ESP32-H2 | ESP32-P4 | ESP32-S2 | ESP32-S3 |
| ----------------- | ----- | -------- | -------- | -------- | -------- | -------- | -------- | -------- |

# _Sample project_

(See the README.md file in the upper level 'examples' directory for more information about examples.)

This is the simplest buildable example. The example is used by command `idf.py create-project`
that copies the project to user specified path and set it's name. For more information follow the [docs page](https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html#start-a-new-project)



## How to use example
We encourage the users to use the example as a template for the new projects.
A recommended way is to follow the instructions on a [docs page](https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html#start-a-new-project).

## Example folder contents

The project **sample_project** contains one source file in C language [main.c](main/main.c). The file is located in folder [main](main).

ESP-IDF projects are built using CMake. The project build configuration is contained in `CMakeLists.txt`
files that provide set of directives and instructions describing the project's source files and targets
(executable, library, or both). 

Below is short explanation of remaining files in the project folder.

```
├── CMakeLists.txt
├── main
│   ├── CMakeLists.txt
│   └── main.c
└── README.md                  This is the file you are currently reading
```
Additionally, the sample project contains Makefile and component.mk files, used for the legacy Make based build system. 
They are not used or needed when building with CMake and idf.py.

## settings.json

I work on Mac as well as Windows10 machine and use vscode to develop this code.
Right settings.json is required to seamlessly move between the platforms.
See https://code.visualstudio.com/docs/getstarted/settings#_settings-precedence

The settings for ESP-IDF containing ESP_IDF and ESP_IDF_TOOLS paths and python
environment should be local to the platform specific installations (therefore
not part of this repo).

Only certain things like the idfTarget, idfPort etc. are project specific and
that should be stored in .vscode/settings.json and is part of the repo. 

Note that everytime ESP-IDF is configured, it updates .vscode/seetings.json with the platform-specific settings.

.vscode/c_cpp_properties.json is manually edited to include two configurations with names ESP-IDF(Mac) and ESP-IDF(Win). Configuring through command panel, will create one with platform specific paths; manual edit was required to combine the two under different names. Based on the platform you are working on, choose the appropriate configuration by clicking on the configuration (at the very right end) on the status bar of vscode.

## CMake
Sometimes the settings get all messed up and/or cmake makes it so. It cannot find IDF_PATH variable. For this reason and also to be able to run idf.py from command line it is advisable to add the tools path to system PATH variable. The instructions are at this link.
https://docs.espressif.com/projects/esp-idf/en/v3.3/get-started-cmake/add-idf_path-to-profile.html

However, it seems that I ran into this problem since I clicked on a prompt in VSCODE to run cmake at every startup. Once I disabled it in settings, the build seemed to happen just fine. 