  # CODERDBC 
  
  ## What is it

  This fork is intended to adapt the utility for generating code snippets for a Robot Operating System noetic ninjemys (ROS1 noetic) node from a dbc (CAN database) file. 
  
  This is explicitly intended for usage with the Ixxat FRC-EP170 gateway, which is able to receive FlexRay frames, from e.g. a car's internal network, and deploy them over a TCP ethernet connection. Also see https://github.com/AdriveLivingLab/FRos for that.
  
  So in the end, this utility will produce the following files in the subfolder /ros:
 
  | File           | Description |
  | -------------- | ----------- |
  | CMakeLists.txt | For building the node, includes the .msg files                     |
  | convert.h      | Functions for converting raw signals to physical signals           |
  | decode.h       | Structs for extracting the raw signals from the stream             |
  | evaluation.txt | Code snippet with functions for calling the convert function based on the message ID |
  | publishers.txt | Code snippet for copy-pasting into the node code to setup a publisher for every message                   |
  | .msg           | For every defined message, a .msg file is generated |

  CLI utilty for generating C code from dbc (CAN matrix) files

  ## Build and run

  This manual works on Ubuntu 20.04 and Windows 10. You need to ensure that your system has
  C++ compile and builing toolchain (**c++17**)

  To build coderdbc you need to make next steps:
  
  1 install cmake
  
  2 download source code:

  `git clone https://github.com/astand/c-coderdbc.git coderdbc`
  
  3 goto source code directory:

  `cd coderdbc`

  4 run cmake configuration to 'build' directory:

  `cmake -S src -B build`

  5 run cmake build:

  `cmake --build build --config release`

  6 goto to build directory and run:

  `cd build`

  `./coderdbc`

  Call without argument will print instruction how to pass DBC file for generation

  `./coderdbc -dbc ./example.dbc -out ./example -drvname example_gw`

  ## Driver functionality description

  For the full functionality please also see the original project linked in the Acknowledgement section at the bottom.
  The here proposed functionalities are as follows:

    (1) ros/msg/

    For each message found in the .dbc file, a .msg file will be generated according to the ROS definition. It will include the ROS message header and each signal found in the message. For signals that have conversion rules defined, the data type will be changed to double.
    
    (2) ros/CMakeLists.txt

    Build instructins for building the catkin package. This includes all the generated .msg files and ready to run.

    (3) decode.h
    
    Structs for extracting the raw signals from the stream and assigning the signals according to their type.
    
    (4) convert.h

    Functions for converting raw signals to physical signals using C++ structs and the conversion rules defined in the .dbc.

    (5) evaluation.txt

    Simple if conditions to find the functions to call for a given message ID.

  ## Acknowledgement
  We would like to thank the github user [astand](https://github.com/astand) that prepared this beautiful program that, thanks to his contributions to the open source world, we were able to use as a framework to be able to realize this project. You can view the original work [here.](https://github.com/astand/c-coderdbc)
  This fork was realized by [Ludwig Kastner](https://github.com/ludwig-kastner) and [Daniel Schneider](https://www.github.com/lnxdxc).