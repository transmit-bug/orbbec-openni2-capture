/**
 * Quick integration test for oni_driver_astra.so
 * Tests: driver load, device discovery, IR stream start/stop, depth stream start/stop
 */

#include <OpenNI.h>
#include <OniCProperties.h>
#include <cstdio>
#include <csignal>
#include <chrono>
#include <thread>
#include <atomic>

static std::atomic<bool> g_running{true};
static void signal_handler(int) { g_running = false; }

int main()
{
    signal(SIGINT, signal_handler);
    setvbuf(stdout, NULL, _IONBF, 0);

    printf("=== oni_driver_astra integration test ===\n");

    // Point OpenNI2 to our drivers
    setenv("OPENNI2_REDIST", "/home/bowmanhan/Code/OrbbecSDK/AstraSDK-v2.1.3/lib/Plugins/openni2/", 1);

    OniStatus rc = oniInitialize(ONI_API_VERSION);
    if (rc != ONI_STATUS_OK) {
        fprintf(stderr, "FAIL: oniInitialize returned %d\n", rc);
        return 1;
    }
    printf("PASS: oniInitialize OK\n");

    // List devices
    OniDeviceInfo* deviceInfos;
    int deviceCount;
    rc = oniGetDeviceList(&deviceInfos, &deviceCount);
    if (rc != ONI_STATUS_OK || deviceCount == 0) {
        fprintf(stderr, "FAIL: No devices found (count=%d, rc=%d)\n", deviceCount, rc);
        oniShutdown();
        return 1;
    }
    printf("PASS: Found %d device(s)\n", deviceCount);
    for (int i = 0; i < deviceCount; i++) {
        printf("  Device[%d]: uri='%s' name='%s' vendor='%s'\n",
               i, deviceInfos[i].uri, deviceInfos[i].name, deviceInfos[i].vendor);
    }

    // Open device
    OniDeviceHandle device;
    rc = oniDeviceOpen(deviceInfos[0].uri, &device);
    if (rc != ONI_STATUS_OK) {
        fprintf(stderr, "FAIL: oniDeviceOpen returned %d\n", rc);
        oniShutdown();
        return 1;
    }
    printf("PASS: Device opened\n");

    // Query sensor info
    const OniSensorInfo* depthInfo = oniDeviceGetSensorInfo(device, ONI_SENSOR_DEPTH);
    const OniSensorInfo* irInfo = oniDeviceGetSensorInfo(device, ONI_SENSOR_IR);

    if (depthInfo) {
        printf("PASS: Depth sensor available (%d modes)\n", depthInfo->numSupportedVideoModes);
        for (int i = 0; i < depthInfo->numSupportedVideoModes; i++) {
            auto& m = depthInfo->pSupportedVideoModes[i];
            printf("  Depth[%d]: %dx%d @ %dfps fmt=%d\n", i, m.resolutionX, m.resolutionY, m.fps, m.pixelFormat);
        }
    } else {
        printf("WARN: No depth sensor info\n");
    }

    if (irInfo) {
        printf("PASS: IR sensor available (%d modes)\n", irInfo->numSupportedVideoModes);
        for (int i = 0; i < irInfo->numSupportedVideoModes; i++) {
            auto& m = irInfo->pSupportedVideoModes[i];
            printf("  IR[%d]: %dx%d @ %dfps fmt=%d\n", i, m.resolutionX, m.resolutionY, m.fps, m.pixelFormat);
        }
    } else {
        printf("WARN: No IR sensor info\n");
    }

    // Test IR stream
    if (irInfo) {
        OniStreamHandle irStream;
        rc = oniDeviceCreateStream(device, ONI_SENSOR_IR, &irStream);
        if (rc == ONI_STATUS_OK) {
            printf("PASS: IR stream created\n");

            OniVideoMode mode;
            mode.resolutionX = 640;
            mode.resolutionY = 480;
            mode.fps = 30;
            mode.pixelFormat = ONI_PIXEL_FORMAT_GRAY16;
            rc = oniStreamSetProperty(irStream, ONI_STREAM_PROPERTY_VIDEO_MODE, &mode, sizeof(mode));
            printf("  Set IR video mode: rc=%d\n", rc);

            rc = oniStreamStart(irStream);
            if (rc == ONI_STATUS_OK) {
                printf("PASS: IR stream started, reading 10 frames...\n");
                int framesRead = 0;
                auto t0 = std::chrono::steady_clock::now();
                while (framesRead < 10 && g_running) {
                    OniFrame* frame;
                    rc = oniStreamReadFrame(irStream, &frame);
                    if (rc == ONI_STATUS_OK && frame && frame->dataSize > 0) {
                        framesRead++;
                        if (framesRead <= 3) {
                            printf("  IR frame[%d]: %dx%d dataSize=%d ts=%lu\n",
                                   framesRead, frame->width, frame->height,
                                   frame->dataSize, (unsigned long)frame->timestamp);
                        }
                        oniFrameRelease(frame);
                    } else {
                        if (!g_running) break;
                    }
                }
                auto t1 = std::chrono::steady_clock::now();
                float dt = std::chrono::duration<float>(t1 - t0).count();
                printf("PASS: Read %d IR frames in %.2fs (%.1f FPS)\n",
                       framesRead, dt, framesRead > 0 ? framesRead / dt : 0);

                oniStreamStop(irStream);
                printf("PASS: IR stream stopped\n");
            } else {
                fprintf(stderr, "FAIL: IR stream start returned %d\n", rc);
            }
            oniStreamDestroy(irStream);
        } else {
            fprintf(stderr, "FAIL: IR stream create returned %d\n", rc);
        }
    }

    // Test depth stream
    if (depthInfo) {
        OniStreamHandle depthStream;
        rc = oniDeviceCreateStream(device, ONI_SENSOR_DEPTH, &depthStream);
        if (rc == ONI_STATUS_OK) {
            printf("PASS: Depth stream created\n");

            OniVideoMode mode;
            mode.resolutionX = 640;
            mode.resolutionY = 480;
            mode.fps = 30;
            mode.pixelFormat = ONI_PIXEL_FORMAT_DEPTH_1_MM;
            rc = oniStreamSetProperty(depthStream, ONI_STREAM_PROPERTY_VIDEO_MODE, &mode, sizeof(mode));
            printf("  Set depth video mode: rc=%d\n", rc);

            rc = oniStreamStart(depthStream);
            if (rc == ONI_STATUS_OK) {
                printf("PASS: Depth stream started, reading 10 frames...\n");
                int framesRead = 0;
                auto t0 = std::chrono::steady_clock::now();
                while (framesRead < 10 && g_running) {
                    OniFrame* frame;
                    rc = oniStreamReadFrame(depthStream, &frame);
                    if (rc == ONI_STATUS_OK && frame && frame->dataSize > 0) {
                        framesRead++;
                        if (framesRead <= 3) {
                            uint16_t* pixels = (uint16_t*)frame->data;
                            int nonzero = 0;
                            for (int i = 0; i < frame->width * frame->height; i++) {
                                if (pixels[i] > 0) nonzero++;
                            }
                            printf("  Depth frame[%d]: %dx%d dataSize=%d nonzero=%d\n",
                                   framesRead, frame->width, frame->height,
                                   frame->dataSize, nonzero);
                        }
                        oniFrameRelease(frame);
                    } else {
                        if (!g_running) break;
                    }
                }
                auto t1 = std::chrono::steady_clock::now();
                float dt = std::chrono::duration<float>(t1 - t0).count();
                printf("PASS: Read %d depth frames in %.2fs (%.1f FPS)\n",
                       framesRead, dt, framesRead > 0 ? framesRead / dt : 0);

                oniStreamStop(depthStream);
                printf("PASS: Depth stream stopped\n");
            } else {
                fprintf(stderr, "FAIL: Depth stream start returned %d\n", rc);
            }
            oniStreamDestroy(depthStream);
        } else {
            fprintf(stderr, "FAIL: Depth stream create returned %d\n", rc);
        }
    }

    // Test laser property
    uint8_t laserVal = 1;
    rc = oniDeviceSetProperty(device, OBEXTENSION_ID_LASER_EN, &laserVal, sizeof(laserVal));
    printf("Laser ON: rc=%d\n", rc);
    laserVal = 0;
    rc = oniDeviceSetProperty(device, OBEXTENSION_ID_LASER_EN, &laserVal, sizeof(laserVal));
    printf("Laser OFF: rc=%d\n", rc);
    laserVal = 1;
    rc = oniDeviceSetProperty(device, OBEXTENSION_ID_LASER_EN, &laserVal, sizeof(laserVal));
    printf("Laser ON: rc=%d\n", rc);

    oniDeviceClose(device);
    oniShutdown();
    printf("=== Test complete ===\n");
    return 0;
}
