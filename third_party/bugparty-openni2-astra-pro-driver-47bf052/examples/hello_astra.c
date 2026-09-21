/**
 * Astra Pro Hello World — OpenNI2 C API
 *
 * Discovers the device, prints device info + all readable properties,
 * lists sensor modes, streams a few depth + IR frames, then exits.
 * Equivalent of OrbbecSDK's HelloOrbbec but using our OpenNI2 driver.
 */

#include <OpenNI.h>
#include <OniCProperties.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>

static volatile int g_running = 1;
void sig_handler(int) { g_running = 0; }

void printDeviceProperties(OniDeviceHandle device) {
    struct { int id; const char* name; int isInt; } props[] = {
        { ONI_DEVICE_PROPERTY_FIRMWARE_VERSION,   "FIRMWARE_VERSION", 1 },
        { ONI_DEVICE_PROPERTY_SERIAL_NUMBER,       "SERIAL_NUMBER", 0 },
        { OBEXTENSION_ID_SERIALNUMBER,              "OBEXT_SERIALNUMBER", 0 },
        { OBEXTENSION_ID_LASER_EN,                 "LASER_EN", 0 },
        { OBEXTENSION_ID_LDP_EN,                   "LDP_EN", 0 },
        { OBEXTENSION_ID_IR_GAIN,                  "IR_GAIN", 0 },
        { OBEXTENSION_ID_IR_EXP,                   "IR_EXP", 0 },
        // CAM_PARAMS reads flash (slow, may timeout) — skipped unless --all flag
    };
    for (int i = 0; i < (int)(sizeof(props)/sizeof(props[0])); i++) {
        char buf[4096] = {};
        int sz = sizeof(buf);
        OniStatus rc = oniDeviceGetProperty(device, props[i].id, buf, &sz);
        if (rc == ONI_STATUS_OK && sz > 0) {
            if (props[i].isInt && sz == 4) {
                uint32_t v = *(uint32_t*)buf;
                printf("  %-20s (id=%2d): %u.%u.%u\n", props[i].name, props[i].id,
                       (v >> 16) & 0xFF, (v >> 8) & 0xFF, v & 0xFF);
            } else if (!props[i].isInt && sz >= 4 && buf[0] >= ' ' && buf[0] <= '~') {
                int slen = sz;
                while (slen > 0 && buf[slen-1] == 0) slen--;
                printf("  %-20s (id=%2d): \"%.*s\"\n", props[i].name, props[i].id, slen, buf);
            } else if (sz == 1) {
                printf("  %-20s (id=%2d): %u\n", props[i].name, props[i].id, (unsigned char)buf[0]);
            } else if (sz == 2) {
                printf("  %-20s (id=%2d): %u\n", props[i].name, props[i].id, *(uint16_t*)buf);
            } else if (sz == 4) {
                printf("  %-20s (id=%2d): 0x%08x (%u)\n", props[i].name, props[i].id, *(uint32_t*)buf, *(uint32_t*)buf);
            } else {
                printf("  %-20s (id=%2d): [%d bytes]", props[i].name, props[i].id, sz);
                int show = sz > 64 ? 64 : sz;
                for (int j = 0; j < show; j++) printf(" %02x", (unsigned char)buf[j]);
                if (sz > 64) printf(" ...");
                printf("\n");
            }
        } else if (rc == ONI_STATUS_OK) {
            printf("  %-20s (id=%2d): (empty)\n", props[i].name, props[i].id);
        } else {
            printf("  %-20s (id=%2d): not available\n", props[i].name, props[i].id);
        }
    }
}

void printSensorModes(OniDeviceHandle device) {
    OniSensorType types[] = {ONI_SENSOR_DEPTH, ONI_SENSOR_IR, ONI_SENSOR_COLOR};
    const char* names[] = {"Depth", "IR", "Color"};

    for (int s = 0; s < 3; s++) {
        const OniSensorInfo* info = oniDeviceGetSensorInfo(device, types[s]);
        if (!info) continue;

        printf("\n%s sensor: %d mode(s)\n", names[s], info->numSupportedVideoModes);
        for (int i = 0; i < info->numSupportedVideoModes; i++) {
            const OniVideoMode* m = &info->pSupportedVideoModes[i];
            const char* fmt;
            switch (m->pixelFormat) {
            case ONI_PIXEL_FORMAT_DEPTH_1_MM:   fmt = "DEPTH_1_MM"; break;
            case ONI_PIXEL_FORMAT_DEPTH_100_UM:  fmt = "DEPTH_100_UM"; break;
            case ONI_PIXEL_FORMAT_GRAY8:        fmt = "GRAY8"; break;
            case ONI_PIXEL_FORMAT_GRAY16:       fmt = "GRAY16"; break;
            case ONI_PIXEL_FORMAT_RGB888:       fmt = "RGB888"; break;
            default: fmt = "UNKNOWN"; break;
            }
            printf("  [%d] %dx%d @ %dfps  %s\n", i, m->resolutionX, m->resolutionY, m->fps, fmt);
        }
    }
}

int main(int argc, char** argv) {
    signal(SIGINT, sig_handler);
    signal(SIGTERM, sig_handler);
    setenv("OPENNI2_REPO", "/home/bowmanhan/Code/OrbbecSDK/AstraSDK-v2.1.3/lib/Plugins/openni2/OpenNI2/Drivers", 1);
    setvbuf(stdout, NULL, _IONBF, 0);

    OniStatus rc = oniInitialize(ONI_API_VERSION);
    if (rc != ONI_STATUS_OK) {
        fprintf(stderr, "OpenNI2 init failed: %d\n", rc);
        return 1;
    }
    printf("OpenNI2 initialized\n");

    OniDeviceInfo* devInfos;
    int devCount;
    rc = oniGetDeviceList(&devInfos, &devCount);
    if (rc != ONI_STATUS_OK || devCount == 0) {
        fprintf(stderr, "No devices found\n");
        oniShutdown();
        return 1;
    }
    printf("Found %d device(s)\n", devCount);

    OniDeviceHandle device;
    rc = oniDeviceOpen(devInfos[0].uri, &device);
    if (rc != ONI_STATUS_OK) {
        fprintf(stderr, "Device open failed\n");
        oniShutdown();
        return 1;
    }

    printf("\n--- Device Info ---\n");
    printf("Name: %s\n", devInfos[0].name);
    printf("URI:  %s\n", devInfos[0].uri);

    printSensorModes(device);

    // Start depth stream — this claims USB so firmware property queries work
    printf("\n--- Device Properties ---\n");
    OniStreamHandle depthStream;
    bool depthOk = false;
    if (oniDeviceCreateStream(device, ONI_SENSOR_DEPTH, &depthStream) == ONI_STATUS_OK) {
        depthOk = (oniStreamStart(depthStream) == ONI_STATUS_OK);
    }
    if (depthOk) {
        printDeviceProperties(device);
    } else {
        printf("  (depth stream not available — properties unavailable)\n");
    }

    // Stream 5 depth frames
    printf("\n--- Streaming Test ---\n");
    if (depthOk) {
        printf("Depth stream started\n");
        for (int i = 0; i < 5 && g_running; i++) {
            OniFrame* frame;
            if (oniStreamReadFrame(depthStream, &frame) == ONI_STATUS_OK && frame) {
                uint16_t* data = (uint16_t*)frame->data;
                double minV = 1e9, maxV = 0;
                int valid = 0;
                for (int p = 0; p < frame->width * frame->height; p++) {
                    if (data[p] > 0) {
                        if (data[p] < minV) minV = data[p];
                        if (data[p] > maxV) maxV = data[p];
                        valid++;
                    }
                }
                printf("  Depth frame %d: %dx%d  valid=%d  range=%.0f-%.0fmm\n",
                       i, frame->width, frame->height, valid, minV, maxV);
                oniFrameRelease(frame);
            }
        }
        oniStreamStop(depthStream);
        oniStreamDestroy(depthStream);
    }

    // IR stream
    OniStreamHandle irStream;
    if (oniDeviceCreateStream(device, ONI_SENSOR_IR, &irStream) == ONI_STATUS_OK) {
        OniVideoMode mode;
        mode.resolutionX = 640; mode.resolutionY = 480;
        mode.fps = 30; mode.pixelFormat = ONI_PIXEL_FORMAT_GRAY16;
        oniStreamSetProperty(irStream, ONI_STREAM_PROPERTY_VIDEO_MODE, &mode, sizeof(mode));

        uint8_t laserOn = 1;
        oniDeviceSetProperty(device, OBEXTENSION_ID_LASER_EN, &laserOn, sizeof(laserOn));

        if (oniStreamStart(irStream) == ONI_STATUS_OK) {
            printf("IR stream started\n");
            for (int i = 0; i < 5 && g_running; i++) {
                OniFrame* frame;
                if (oniStreamReadFrame(irStream, &frame) == ONI_STATUS_OK && frame) {
                    uint16_t* data = (uint16_t*)frame->data;
                    double maxV = 0;
                    for (int p = 0; p < frame->width * frame->height; p++) {
                        if (data[p] > maxV) maxV = data[p];
                    }
                    printf("  IR frame %d: %dx%d  maxVal=%.0f\n",
                           i, frame->width, frame->height, maxV);
                    oniFrameRelease(frame);
                }
            }
            oniStreamStop(irStream);
        }
        uint8_t laserOff = 0;
        oniDeviceSetProperty(device, OBEXTENSION_ID_LASER_EN, &laserOff, sizeof(laserOff));
        oniStreamDestroy(irStream);
    }

    oniDeviceClose(device);
    oniShutdown();
    printf("\nDone.\n");
    return 0;
}
