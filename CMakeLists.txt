cmake_minimum_required(VERSION 3.5)
project(MainProgram)
set(CMAKE_CXX_STANDARD 11)

find_package(OpenCV REQUIRED)

add_executable(MainProgram
        main.cpp
        skin.cpp
        contour.cpp
        SideFootDirection.cpp
        UpperFootDirection.cpp
        common.cpp
        spline.hpp
        common.hpp
        CardDetector.cpp
        CardDetector.h
        )


target_link_libraries(MainProgram
        ${OpenCV_LIBS}
        )
file(COPY img/foot_up_3.jpg DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
