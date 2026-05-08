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
set(CMAKE_PROJECT_NAME FOC_DRIVE)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        Middle_Layer/ottohesl/ottohesl.c
        Middle_Layer/ottohesl/ottohesl.h
        BSP_Layer/Menu/oledfont.c
        BSP_Layer/Menu/ottohesl_OLED.c
        BSP_Layer/Encoder/as5600.c
        BSP_Layer/Main_Freertos/Main_Freertos.c
        BSP_Layer/Main_Freertos/Main_Freertos.h
        BSP_Layer/FOC/FOC_Math.h
        BSP_Layer/FOC/FOC_Math.c
        BSP_Layer/FOC/FOC_PID.c
        BSP_Layer/FOC/FOC_PID.h
        BSP_Layer/FOC/FOC_RUN.c
        BSP_Layer/FOC/FOC_RUN.h
        BSP_Layer/Menu/LCD_1.14.c
        BSP_Layer/Menu/LCD_1.14.h
)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Add touchgfx generated sources and libraries
add_subdirectory(cmake/touchgfx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
        Middle_Layer/ottohesl
        BSP_Layer/Menu
        BSP_Layer/Encoder
        BSP_Layer/FOC
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
        Middle_Layer/ottohesl
        BSP_Layer/Menu
        BSP_Layer/Encoder
        BSP_Layer/FOC
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
set(COMMON_FLAGS "-u _printf_float")  # 启用浮点printf支持
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${COMMON_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${COMMON_FLAGS}")

