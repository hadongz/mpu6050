cmake_minimum_required(VERSION 3.10)
project(MPU6050_Project C CXX)

# Set C/C++ standards
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 11)

# Set output directories for binary 
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)

# Add include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Find bcm2835 library
find_library(BCM2835_LIBRARY bcm2835)
if(NOT BCM2835_LIBRARY)
    message(FATAL_ERROR "bcm2835 library not found!")
endif()

# Create a library for I2Cdev and MPU6050
add_library(i2cdev_mpu STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/I2Cdev.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/MPU6050.cpp
)

# Specify the target type for the library
set_target_properties(i2cdev_mpu PROPERTIES
    POSITION_INDEPENDENT_CODE ON
)

# Your main executable (adjust main.c to your actual filename)
# add_executable(mpu6050_app main.c)

# If your main file is C++, use this instead:
add_executable(mpu6050_app main.cpp)

# Link libraries to the executable
target_link_libraries(mpu6050_app
    i2cdev_mpu
    ${BCM2835_LIBRARY}
    # Add -lm for math library (often needed)
    m
)

# Set compiler flags
target_compile_options(mpu6050_app PRIVATE -Wall -Wextra)
target_compile_options(i2cdev_mpu PRIVATE -Wall -Wextra)

# If you need to link additional libraries like pthread, add them here
target_link_libraries(mpu6050_app pthread)

# Installation rules
install(TARGETS mpu6050_app DESTINATION bin)
install(FILES ${CMAKE_BINARY_DIR}/bin/mpu6050_app DESTINATION bin
        PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE
                    GROUP_READ GROUP_EXECUTE
                    WORLD_READ WORLD_EXECUTE)
