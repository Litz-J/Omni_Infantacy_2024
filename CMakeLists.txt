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
set(CMAKE_PROJECT_NAME HYOmni)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# 源文件指定

file(GLOB_RECURSE SOURCES
    "bsp/*.c"
    "modules/*.c"
    "application/*.c"
    "Middlewares/Third_Party/SEGGER/*.c"
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    ${SOURCES}
)

# 头文件包含
# Gather all unique directories for BSP, SEGGER, and Modules headers
file(GLOB_RECURSE BSP_INCLUDES "bsp/*.h")
file(GLOB_RECURSE MODULES_INCLUDES "modules/*.h")
file(GLOB_RECURSE APP_INCLUDES "application/*.h")
file(GLOB_RECURSE SEGGER_INCLUDES "Middlewares/Third_Party/SEGGER/*.h")

# 设置一个列表来保存唯一路径
# Initialize a list to hold unique include directories
set(INCLUDE_DIRS)
# 提取路径
# Function to extract unique directories from header files
function(add_unique_directories HEADER_FILES)
    # 检查传入的 HEADER_FILES 是否为空或未定义
    if(NOT HEADER_FILES)
        #message(STATUS "No headers found.")
        return()
    endif()

    foreach(HEADER_PATH ${HEADER_FILES})
        #message(STATUS "Found header: ${HEADER_PATH}")  # 打印找到的头文件路径
        get_filename_component(DIR_PATH ${HEADER_PATH} DIRECTORY)
        if(NOT DIR_PATH IN_LIST INCLUDE_DIRS)  # 检查路径是否已经在列表中
            list(APPEND INCLUDE_DIRS ${DIR_PATH})
        endif()
        #message(STATUS "INCLUDE_DIR NOW:${INCLUDE_DIRS}")
    endforeach()

    set(INCLUDE_DIRS ${INCLUDE_DIRS} PARENT_SCOPE)  # 确保 INCLUDE_DIRS 在父作用域中可见
endfunction()

# Extract directories for each set of headers
# 提取每个头文件集的目录
add_unique_directories("${BSP_INCLUDES}")
add_unique_directories("${MODULES_INCLUDES}")
add_unique_directories("${APP_INCLUDES}")
add_unique_directories("${SEGGER_INCLUDES}")
#打印收集到的头文件目录
message("includepath:${INCLUDE_DIRS}")

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    ${INCLUDE_DIRS}
)

# 添加汇编文件路径
# ENABLE_LANGUAGE(ASM)
set(ASM_SOURCES
    startup_stm32f407xx.s
    Middlewares/Third_Party/SEGGER/RTT/SEGGER_RTT_ASM_ARMv7M.s
)
set_source_files_properties(${ASM_SOURCES} PROPERTIES COMPILE_FLAGS "-x assembler-with-cpp")

target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${ASM_SOURCES})

# 宏定义
# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
    ARM_MATH_CM4
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
