cmake_minimum_required(VERSION 3.10)
project(oni_driver_astra LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Release optimization for production (BFS filters need -O2+)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Try pkg-config first, fall back to find_library, then nix store
find_package(PkgConfig)
if(PKG_CONFIG_FOUND)
    pkg_check_modules(LIBUSB libusb-1.0)
endif()

if(NOT LIBUSB_FOUND)
    find_library(LIBUSB_LIBRARY NAMES usb-1.0 usb)
    find_path(LIBUSB_INCLUDE_DIR NAMES libusb.h PATH_SUFFIXES libusb-1.0)
    if(LIBUSB_LIBRARY)
        set(LIBUSB_FOUND TRUE)
        set(LIBUSB_LIBRARIES ${LIBUSB_LIBRARY})
        set(LIBUSB_INCLUDE_DIRS "${LIBUSB_INCLUDE_DIR}")
    endif()
endif()

if(NOT LIBUSB_FOUND)
    # Try nix-store libusb dev + lib
    set(LIBUSB_NIX_INC "/home/bowmanhan/.local/share/nix/root/nix/store/kbin0714jdahqd917p6cpdflwbynibg8-libusb-1.0.29-dev/include")
    set(LIBUSB_NIX_LIB "/home/bowmanhan/.local/share/nix/root/nix/store/4v4ns2sw2qffrfaqz0zmqmz778ijzy7v-libusb-1.0.29/lib")
    if(EXISTS "${LIBUSB_NIX_INC}/libusb-1.0/libusb.h")
        set(LIBUSB_FOUND TRUE)
        set(LIBUSB_LIBRARIES "${LIBUSB_NIX_LIB}/libusb-1.0.so")
        set(LIBUSB_INCLUDE_DIRS "${LIBUSB_NIX_INC}")
        set(LIBUSB_LIBRARY_DIRS "${LIBUSB_NIX_LIB}")
    endif()
endif()

if(NOT LIBUSB_FOUND)
    message(FATAL_ERROR "libusb-1.0 not found. Install libusb1-devel or equivalent.")
endif()

set(OPENNI2_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_library(oni_driver_astra SHARED
    src/AstraDriver.cpp
    src/AstraDevice.cpp
    src/AstraDepthStream.cpp
    src/AstraIRStream.cpp
    src/UsbDevice.cpp
    src/PacketParser.cpp
    src/FrameProcessor.cpp
    src/IrProcessor.cpp
    src/DepthProcessor.cpp
    src/FirmwareCmd.cpp
    src/SoftFilter.cpp
)

target_include_directories(oni_driver_astra
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${OPENNI2_INCLUDE_DIR}
        ${LIBUSB_INCLUDE_DIRS}
)

target_compile_definitions(oni_driver_astra
    PRIVATE
        OPENNI2_EXPORT
        $<$<CONFIG:Debug>:ASTRA_DEBUG>
)

target_link_directories(oni_driver_astra
    PRIVATE
        ${LIBUSB_LIBRARY_DIRS}
)

target_link_libraries(oni_driver_astra
    PRIVATE
        ${LIBUSB_LIBRARIES}
        pthread
)

set_target_properties(oni_driver_astra PROPERTIES
    PREFIX ""
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
)

# Install driver .so and .ini to OpenNI2 Drivers directory.
# OpenNI2 loads every *.so it finds in <libOpenNI2.so dir>/OpenNI2/Drivers/, so
# the install target and the post-build copy below must both write there — not
# to the parent plugin dir.
set(OPENNI2_DRIVERS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../AstraSDK-v2.1.3/lib/Plugins/openni2/OpenNI2/Drivers" CACHE PATH "OpenNI2 drivers directory")

# OpenNI2's plugin loader only picks up files matching `lib*.so`, so the
# installed name must start with `lib` even though the target build artifact
# is `oni_driver_astra.so` (PREFIX="").
set(INSTALLED_DRIVER_NAME "libAstraDriver.so")

install(FILES $<TARGET_FILE:oni_driver_astra>
    DESTINATION ${OPENNI2_DRIVERS_DIR}
    RENAME ${INSTALLED_DRIVER_NAME})
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/astra.ini DESTINATION ${OPENNI2_DRIVERS_DIR})

# Post-build: refresh the loaded driver every rebuild so test programs pick up
# the latest binary without a separate `cmake --install` step.
add_custom_command(TARGET oni_driver_astra POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:oni_driver_astra>
        ${OPENNI2_DRIVERS_DIR}/${INSTALLED_DRIVER_NAME}
    COMMENT "Installing ${INSTALLED_DRIVER_NAME} to ${OPENNI2_DRIVERS_DIR}"
)

# Standalone test for FirmwareCmd (requires real hardware)
add_executable(test_firmware_cmd
    test_firmware_cmd.cpp
    src/UsbDevice.cpp
    src/FirmwareCmd.cpp
)
target_include_directories(test_firmware_cmd
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${LIBUSB_INCLUDE_DIRS}
)
target_link_directories(test_firmware_cmd
    PRIVATE
        ${LIBUSB_LIBRARY_DIRS}
)
target_link_libraries(test_firmware_cmd
    PRIVATE
        ${LIBUSB_LIBRARIES}
        pthread
)

# Integration test (links against OpenNI2, requires hardware + installed driver)
add_executable(test_driver test_driver.cpp)
set(OPENNI2_PUBLIC_INCLUDE "/home/bowmanhan/Downloads/OpenNI_SDK_ROS2_v1.0.2_20220809_b32e47_linux/ros2_astra_camera/astra_camera/include/openni2")
target_include_directories(test_driver
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${OPENNI2_PUBLIC_INCLUDE}
)
target_link_directories(test_driver
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../AstraSDK-v2.1.3/lib/Plugins/openni2
)
target_link_libraries(test_driver
    PRIVATE
        OpenNI2
        pthread
)

# --- Example viewers (GTK3 + OpenCV + OpenNI2) ---
option(BUILD_EXAMPLES "Build example viewers" ON)

if(BUILD_EXAMPLES)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(GTK3 REQUIRED gtk+-3.0)
    pkg_check_modules(OPENCV4 REQUIRED opencv4)

    set(OPENNI2_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../AstraSDK-v2.1.3/lib/Plugins/openni2")
    set(OPENNI2_RPATH "${OPENNI2_LIB_DIR}")

    # Depth viewer
    add_executable(depth_viewer examples/test_depth_viewer.c)
    set_source_files_properties(examples/test_depth_viewer.c PROPERTIES LANGUAGE CXX)
    target_include_directories(depth_viewer PRIVATE
        ${OPENNI2_PUBLIC_INCLUDE} ${GTK3_INCLUDE_DIRS} ${OPENCV4_INCLUDE_DIRS})
    target_link_directories(depth_viewer PRIVATE ${OPENNI2_LIB_DIR})
    target_link_libraries(depth_viewer
        PRIVATE OpenNI2 ${GTK3_LIBRARIES} ${OPENCV4_LIBRARIES} pthread
    )
    target_compile_options(depth_viewer PRIVATE ${GTK3_CFLAGS_OTHER} ${OPENCV4_CFLAGS_OTHER})
    set_target_properties(depth_viewer PROPERTIES
        INSTALL_RPATH "${OPENNI2_RPATH}"
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/examples
    )

    # IR viewer
    add_executable(ir_viewer examples/test_ir_viewer.c)
    set_source_files_properties(examples/test_ir_viewer.c PROPERTIES LANGUAGE CXX)
    target_include_directories(ir_viewer PRIVATE
        ${OPENNI2_PUBLIC_INCLUDE} ${GTK3_INCLUDE_DIRS} ${OPENCV4_INCLUDE_DIRS})
    target_link_directories(ir_viewer PRIVATE ${OPENNI2_LIB_DIR})
    target_link_libraries(ir_viewer
        PRIVATE OpenNI2 ${GTK3_LIBRARIES} ${OPENCV4_LIBRARIES} pthread
    )
    target_compile_options(ir_viewer PRIVATE ${GTK3_CFLAGS_OTHER} ${OPENCV4_CFLAGS_OTHER})
    set_target_properties(ir_viewer PROPERTIES
        INSTALL_RPATH "${OPENNI2_RPATH}"
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/examples
    )

    # Depth+IR dual viewer
    add_executable(depth_ir_viewer examples/test_depth_ir_viewer.c)
    set_source_files_properties(examples/test_depth_ir_viewer.c PROPERTIES LANGUAGE CXX)
    target_include_directories(depth_ir_viewer PRIVATE
        ${OPENNI2_PUBLIC_INCLUDE} ${GTK3_INCLUDE_DIRS} ${OPENCV4_INCLUDE_DIRS})
    target_link_directories(depth_ir_viewer PRIVATE ${OPENNI2_LIB_DIR})
    target_link_libraries(depth_ir_viewer
        PRIVATE OpenNI2 ${GTK3_LIBRARIES} ${OPENCV4_LIBRARIES} pthread
    )
    target_compile_options(depth_ir_viewer PRIVATE ${GTK3_CFLAGS_OTHER} ${OPENCV4_CFLAGS_OTHER})
    set_target_properties(depth_ir_viewer PROPERTIES
        INSTALL_RPATH "${OPENNI2_RPATH}"
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/examples
    )

    # RGB-D viewer requires OrbbecSDK v1 headers (not included by default)
    # To build: add OrbbecSDK include path manually

    # Hello Astra — device info + streaming test (no GTK/OpenCV dependency)
    add_executable(hello_astra examples/hello_astra.c)
    set_source_files_properties(examples/hello_astra.c PROPERTIES LANGUAGE CXX)
    target_include_directories(hello_astra PRIVATE ${OPENNI2_PUBLIC_INCLUDE})
    target_link_directories(hello_astra PRIVATE ${OPENNI2_LIB_DIR})
    target_link_libraries(hello_astra PRIVATE OpenNI2 pthread)
    set_target_properties(hello_astra PROPERTIES
        INSTALL_RPATH "${OPENNI2_RPATH}"
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/examples
    )

    # Console depth test (no GTK/OpenCV)
    add_executable(depth_console examples/test_depth_console.c)
    set_source_files_properties(examples/test_depth_console.c PROPERTIES LANGUAGE CXX)
    target_include_directories(depth_console PRIVATE ${OPENNI2_PUBLIC_INCLUDE})
    target_link_directories(depth_console PRIVATE ${OPENNI2_LIB_DIR})
    target_link_libraries(depth_console PRIVATE OpenNI2 pthread)
    set_target_properties(depth_console PROPERTIES
        INSTALL_RPATH "${OPENNI2_RPATH}"
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/examples
    )

    # S2D table dumper
    add_executable(s2d_table examples/test_s2d_table.c)
    set_source_files_properties(examples/test_s2d_table.c PROPERTIES LANGUAGE CXX)
    target_include_directories(s2d_table PRIVATE ${OPENNI2_PUBLIC_INCLUDE})
    target_link_directories(s2d_table PRIVATE ${OPENNI2_LIB_DIR})
    target_link_libraries(s2d_table PRIVATE OpenNI2 pthread m)
    set_target_properties(s2d_table PROPERTIES
        INSTALL_RPATH "${OPENNI2_RPATH}"
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/examples
    )

    # IR + Laser control viewer
    add_executable(ir_control examples/ir_control.c)
    set_source_files_properties(examples/ir_control.c PROPERTIES LANGUAGE CXX)
    target_include_directories(ir_control PRIVATE
        ${OPENNI2_PUBLIC_INCLUDE} ${GTK3_INCLUDE_DIRS} ${OPENCV4_INCLUDE_DIRS})
    target_link_directories(ir_control PRIVATE ${OPENNI2_LIB_DIR})
    target_link_libraries(ir_control
        PRIVATE OpenNI2 ${GTK3_LIBRARIES} ${OPENCV4_LIBRARIES} pthread
    )
    target_compile_options(ir_control PRIVATE ${GTK3_CFLAGS_OTHER} ${OPENCV4_CFLAGS_OTHER})
    set_target_properties(ir_control PROPERTIES
        INSTALL_RPATH "${OPENNI2_RPATH}"
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/examples
    )

    # Automated depth comparison tool (capture + compare)
    add_executable(dump_compare examples/dump_compare.c)
    set_source_files_properties(examples/dump_compare.c PROPERTIES LANGUAGE CXX)
    target_include_directories(dump_compare PRIVATE ${OPENNI2_PUBLIC_INCLUDE})
    target_link_directories(dump_compare PRIVATE ${OPENNI2_LIB_DIR})
    target_link_libraries(dump_compare PRIVATE OpenNI2 pthread m)
    set_target_properties(dump_compare PROPERTIES
        INSTALL_RPATH "${OPENNI2_RPATH}"
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/examples
    )

    # Frame dump tool (calibration capture)
    add_executable(dump_frame examples/dump_frame.c)
    set_source_files_properties(examples/dump_frame.c PROPERTIES LANGUAGE CXX)
    target_include_directories(dump_frame PRIVATE ${OPENNI2_PUBLIC_INCLUDE})
    target_link_directories(dump_frame PRIVATE ${OPENNI2_LIB_DIR})
    target_link_libraries(dump_frame PRIVATE OpenNI2 pthread)
    set_target_properties(dump_frame PROPERTIES
        INSTALL_RPATH "${OPENNI2_RPATH}"
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/examples
    )

    # SoftFilter unit test (no hardware dependency)
    add_executable(test_softfilter tests/test_softfilter.cpp src/SoftFilter.cpp)
    target_include_directories(test_softfilter PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
    set_target_properties(test_softfilter PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/tests
    )

    # S2D calibration tool — pure math mode (no hardware)
    add_executable(s2d_calibrate tools/s2d_calibrate.cpp)
    target_link_libraries(s2d_calibrate PRIVATE m)
    set_target_properties(s2d_calibrate PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/tools
    )

    # S2D calibration tool — capture mode (requires hardware + OpenNI2)
    add_executable(s2d_calibrate_capture tools/s2d_calibrate.cpp)
    target_include_directories(s2d_calibrate_capture PRIVATE ${OPENNI2_PUBLIC_INCLUDE})
    target_link_directories(s2d_calibrate_capture PRIVATE ${OPENNI2_LIB_DIR})
    target_link_libraries(s2d_calibrate_capture PRIVATE OpenNI2 m pthread)
    target_compile_definitions(s2d_calibrate_capture PRIVATE CAPTURE_MODE)
    set_target_properties(s2d_calibrate_capture PROPERTIES
        INSTALL_RPATH "${OPENNI2_RPATH}"
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/tools
    )
endif()
