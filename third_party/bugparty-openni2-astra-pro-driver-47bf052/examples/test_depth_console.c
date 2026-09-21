/**
 * Console-only depth test — no GTK/OpenCV needed.
 * Reads 10 depth frames and prints pixel statistics.
 */
#include <OpenNI.h>
#include <OniCProperties.h>
#include <stdio.h>
#include <unistd.h>

int main() {
    setvbuf(stdout, NULL, _IONBF, 0);
    setenv("OPENNI2_REPO", "/home/bowmanhan/Code/OrbbecSDK/AstraSDK-v2.1.3/lib/Plugins/openni2/Drivers", 1);

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

    printf("Streaming...\n");
    for (int f = 0; f < 10; f++) {
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

        printf("Frame %d: %dx%d stride=%d dataSize=%d, %d/%d non-zero",
               f, frame->width, frame->height, frame->stride, frame->dataSize, nonZero, total);
        if (nonZero > 0) printf(", min=%u max=%u (%.2fm)", minV, maxV, maxV/1000.0f);
        printf("\n");

        // Print first 10 pixels of row 0 and row 1
        printf("  row0[0..19]:");
        for (int i = 0; i < 20; i++) printf(" %u", pixels[i]);
        printf("\n  row1[0..19]:");
        for (int i = 640; i < 660; i++) printf(" %u", pixels[i]);
        printf("\n  row2[0..19]:");
        for (int i = 1280; i < 1300; i++) printf(" %u", pixels[i]);
        printf("\n  row10[0..19]:");
        for (int i = 640*10; i < 640*10+20; i++) printf(" %u", pixels[i]);
        printf("\n  row100[0..19]:");
        for (int i = 640*100; i < 640*100+20; i++) printf(" %u", pixels[i]);
        printf("\n");

        oniFrameRelease(frame);
    }

    oniStreamStop(stream);
    oniStreamDestroy(stream);
    oniDeviceClose(dev);
    oniShutdown();
    printf("Done\n");
    return 0;
}
