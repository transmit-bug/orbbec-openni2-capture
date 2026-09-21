/**
 * Dump specific pixel values from depth frames.
 * Useful for comparing official vs our driver at specific coordinates.
 */
#include <OpenNI.h>
#include <OniCProperties.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

int main(int argc, char** argv) {
    setvbuf(stdout, NULL, _IONBF, 0);
    setenv("OPENNI2_REPO", "/home/bowmanhan/Code/OrbbecSDK/AstraSDK-v2.1.3/lib/Plugins/openni2/Drivers", 1);

    // Default target pixel
    int target_x = 213, target_y = 186;
    if (argc >= 3) {
        target_x = atoi(argv[1]);
        target_y = atoi(argv[2]);
    }

    OniStatus rc = oniInitialize(ONI_API_VERSION);
    if (rc != ONI_STATUS_OK) { fprintf(stderr, "Init failed: %d\n", rc); return 1; }

    OniDeviceInfo* devs;
    int n;
    rc = oniGetDeviceList(&devs, &n);
    if (rc != ONI_STATUS_OK || n == 0) { fprintf(stderr, "No devices\n"); oniShutdown(); return 1; }

    OniDeviceHandle dev;
    rc = oniDeviceOpen(devs[0].uri, &dev);
    if (rc != ONI_STATUS_OK) { fprintf(stderr, "Open failed\n"); oniShutdown(); return 1; }
    printf("Device: %s\n", devs[0].name);

    OniStreamHandle stream;
    rc = oniDeviceCreateStream(dev, ONI_SENSOR_DEPTH, &stream);
    if (rc != ONI_STATUS_OK) { fprintf(stderr, "Create stream failed\n"); oniDeviceClose(dev); oniShutdown(); return 1; }

    OniVideoMode mode;
    mode.resolutionX = 640; mode.resolutionY = 480; mode.fps = 30;
    mode.pixelFormat = ONI_PIXEL_FORMAT_DEPTH_1_MM;
    oniStreamSetProperty(stream, ONI_STREAM_PROPERTY_VIDEO_MODE, &mode, sizeof(mode));

    rc = oniStreamStart(stream);
    if (rc != ONI_STATUS_OK) { fprintf(stderr, "Start failed: %d\n", rc); return 1; }

    printf("Streaming... watching pixel (%d,%d)\n", target_x, target_y);
    printf("Also sampling center region (row180-192, col200-230) for context\n\n");

    for (int f = 0; f < 15; f++) {
        OniFrame* frame;
        rc = oniStreamReadFrame(stream, &frame);
        if (rc != ONI_STATUS_OK || !frame || frame->dataSize == 0) {
            printf("Frame %d: read failed\n", f);
            continue;
        }

        uint16_t* pixels = (uint16_t*)frame->data;
        int total = frame->width * frame->height;
        int nonZero = 0;
        uint16_t minV = 0xFFFF, maxV = 0;
        for (int i = 0; i < total; i++) {
            if (pixels[i] != 0) {
                nonZero++;
                if (pixels[i] < minV) minV = pixels[i];
                if (pixels[i] > maxV) maxV = pixels[i];
            }
        }

        // Target pixel
        uint16_t targetVal = pixels[target_y * frame->width + target_x];

        printf("Frame %2d: %d/%d non-zero, min=%u max=%u (%.2fm), pixel(%d,%d)=%u (%.3fm)\n",
               f, nonZero, total, minV, maxV, maxV/1000.0f,
               target_x, target_y, targetVal, targetVal/1000.0f);

        // Dump center region around target for context
        if (f >= 5 && f <= 7) {
            printf("  Region around target (rows %d-%d, cols %d-%d):\n",
                   target_y-5, target_y+5, target_x-10, target_x+10);
            int x1 = target_x - 10, x2 = target_x + 10;
            int y1 = target_y - 5, y2 = target_y + 5;
            for (int y = y1; y <= y2 && y < frame->height; y++) {
                printf("  row%3d:", y);
                for (int x = x1; x <= x2 && x < frame->width; x++) {
                    uint16_t v = pixels[y * frame->width + x];
                    if (v == 0) printf("  ---");
                    else printf(" %4u", v);
                }
                printf("\n");
            }
        }

        oniFrameRelease(frame);
    }

    oniStreamStop(stream);
    oniStreamDestroy(stream);
    oniDeviceClose(dev);
    oniShutdown();
    printf("\nDone\n");
    return 0;
}
