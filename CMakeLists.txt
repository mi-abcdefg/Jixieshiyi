cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME CRTC2026_RobotA)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        Chasis/Inc/mecanum_kinematic.h
        Chasis/Src/mecanum_kinematic.cpp
        Motor/Inc/motor_pwm_driver.h
        Motor/Src/motor_pwm_driver.cpp
        Servo/Inc/servo_driver.h
        Servo/Src/servo_driver.cpp
        Arm/Inc/arm_action.h
        Arm/Src/arm_action.c
        Arm/Inc/arm_control.h
        Arm/Src/arm_control.cpp
        Arm/Inc/arm_ik.h
        Arm/Src/arm_ik.cpp
        Arm/Inc/arm_manual.h
        Arm/Src/arm_manual.cpp
        Arm/Inc/gamepad.h
        Arm/Src/gamepad.cpp
        Arm/Inc/robot_control.h
        Arm/Src/robot_control.cpp
        ControlMode/Inc/control_mode.h
        ControlMode/Src/control_mode.cpp
        Task/Inc/chasis_control_task.h
        Task/Src/chasis_control_task.cpp
        Task/Inc/arm_task.h
        Task/Src/arm_task.cpp
        PS2/Inc/ps2_driver.h
        PS2/Src/ps2_driver.cpp
        PS2/Inc/ps2_decode.h
        PS2/Src/ps2_decode.cpp
        Task/TaskBase.h
        Task/Inc/remote_comm_task.h
        Task/Src/remote_comm_task.cpp
        Task/Inc/test_task.h
        Task/Src/test_task.cpp)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/Chasis/Inc
    ${CMAKE_SOURCE_DIR}/Motor/Inc
    ${CMAKE_SOURCE_DIR}/Servo/Inc
    ${CMAKE_SOURCE_DIR}/Arm/Inc
    ${CMAKE_SOURCE_DIR}/ControlMode/Inc
    ${CMAKE_SOURCE_DIR}/PS2/Inc
    ${CMAKE_SOURCE_DIR}/Task/Inc
    ${CMAKE_SOURCE_DIR}/Task
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
