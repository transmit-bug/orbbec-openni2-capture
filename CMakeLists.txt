cmake_minimum_required(VERSION 3.10)
project(orbbec_astra_pro LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- OpenNI2 runtime (minimal) + custom Astra driver ---
set(OPENNI_DIR ${CMAKE_SOURCE_DIR}/third_party/openni2_runtime)
set(OPENNI_INCLUDE_DIR ${OPENNI_DIR}/Include)
set(OPENNI_LIB ${OPENNI_DIR}/libOpenNI2.so)
set(ASTRA_DRIVER_DIR ${CMAKE_SOURCE_DIR}/third_party/bugparty-openni2-astra-pro-driver-47bf052)

# --- Executable ---
add_executable(astra_capture
    src/main.cpp
    src/camera_openni.cpp
)

target_include_directories(astra_capture PRIVATE
    ${OPENNI_INCLUDE_DIR}
    ${ASTRA_DRIVER_DIR}/include
    ${CMAKE_SOURCE_DIR}/src
)

target_link_libraries(astra_capture PRIVATE
    ${OPENNI_LIB}
)

# --- Build custom Astra Pro driver (replaces buggy liborbbec.so) ---
include(ExternalProject)
ExternalProject_Add(astra_driver
    SOURCE_DIR ${ASTRA_DRIVER_DIR}
    CMAKE_ARGS
        -DCMAKE_BUILD_TYPE=Release
        -DBUILD_EXAMPLES=OFF
        -DOPENNI2_DRIVERS_DIR=${CMAKE_BINARY_DIR}/OpenNI2/Drivers
    BUILD_COMMAND cmake --build <BINARY_DIR> --target oni_driver_astra -- -j$ENV{NPROC}
    BUILD_ALWAYS TRUE
    INSTALL_COMMAND ""
)
add_dependencies(astra_capture astra_driver)

# --- Copy minimal runtime next to executable ---
add_custom_command(TARGET astra_capture POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${OPENNI_DIR}/OpenNI2
        $<TARGET_FILE_DIR:astra_capture>/OpenNI2
    COMMAND ${CMAKE_COMMAND} -E copy
        ${OPENNI_LIB}
        $<TARGET_FILE_DIR:astra_capture>
    COMMAND ${CMAKE_COMMAND} -E copy
        ${OPENNI_DIR}/OpenNI.ini
        $<TARGET_FILE_DIR:astra_capture>
    # Copy the freshly built custom driver
    COMMAND ${CMAKE_COMMAND} -E copy
        ${CMAKE_BINARY_DIR}/astra_driver-prefix/src/astra_driver-build/oni_driver_astra.so
        $<TARGET_FILE_DIR:astra_capture>/OpenNI2/Drivers/libAstraDriver.so
    COMMENT "Copying minimal OpenNI2 runtime + custom Astra driver..."
)
