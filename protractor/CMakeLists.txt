cmake_minimum_required(VERSION 3.10)
project(protractor)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find OpenCV
find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc imgcodecs videoio)

# Create executable
add_executable(protractor protractor.cpp)

# Link OpenCV
target_link_libraries(protractor ${OpenCV_LIBS})

# Include directories
target_include_directories(protractor PRIVATE ${OpenCV_INCLUDE_DIRS})
