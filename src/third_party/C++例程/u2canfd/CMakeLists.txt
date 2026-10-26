cmake_minimum_required(VERSION 3.16)
project(dm_main)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)


# 在 CMakeLists.txt 的任何位置
message("CMAKE_CURRENT_SOURCE_DIR = ${CMAKE_CURRENT_SOURCE_DIR}")
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include
                        ${CMAKE_CURRENT_SOURCE_DIR}/lib
                        ${LIBUSB_INCLUDE_DIRS})
                        


add_executable(${PROJECT_NAME}
    src/main.cpp
    src/protocol/damiao.cpp 
    src/protocol/usb_class.cpp
)

target_link_libraries(${PROJECT_NAME} PRIVATE 
                        ${CMAKE_CURRENT_SOURCE_DIR}/lib/linux/libdm_device.so
                        ${LIBUSB_LIBRARIES})


add_executable(dev_sn
    src/dev_sn.cpp
)

# 链接库到可执行文件
target_link_libraries(dev_sn PRIVATE
    ${LIBUSB_LIBRARIES}
)