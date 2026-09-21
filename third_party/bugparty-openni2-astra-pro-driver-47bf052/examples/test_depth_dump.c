/**
 * Dump raw depth frames to /tmp for offline analysis.
 * Reads 3 frames, saves raw 16-bit depth as binary.
 */
#include <OpenNI.h>
#include <OniCProperties.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>

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
    for (int f = 0; f < 5; f++) {
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

        printf("Frame %d: %dx%d %d/%d non-zero min=%u max=%u (%.2fm)\n",
               f, frame->width, frame->height, nonZero, total, minV, maxV, maxV/1000.0f);

        // Save first 3 frames as binary
        if (f < 3) {
            char path[128];
            snprintf(path, sizeof(path), "/tmp/depth_frame_%d.raw", f);
            FILE* fp = fopen(path, "wb");
            if (fp) {
                fwrite(frame->data, 1, frame->dataSize, fp);
                fclose(fp);
                printf("  Saved %s (%d bytes)\n", path, frame->dataSize);
            }
        }

        // Detailed analysis of first frame
        if (f == 0) {
            // Count depth value histogram in 100mm bins
            int bins[100] = {0};
            for (int i = 0; i < total; i++) {
                if (pixels[i] > 0) {
                    int bin = pixels[i] / 100;
                    if (bin >= 100) bin = 99;
                    bins[bin]++;
                }
            }
            printf("  Depth histogram (100mm bins):\n");
            for (int b = 0; b < 100; b++) {
                if (bins[b] > 0) {
                    printf("    %d.%dm-%d.%dm: %d px\n", b, b, b+1, b+1, bins[b]);
                }
            }

            // Check for periodic patterns in first row
            printf("  Row 0 (all 640 pixels):\n    ");
            for (int x = 0; x < 640; x++) {
                printf("%u ", pixels[x]);
                if ((x+1) % 40 == 0) printf("\n    ");
            }
            printf("\n");

            // Check a row in the middle of the scene
            printf("  Row 200 (first 40 pixels):\n    ");
            for (int x = 0; x < 40; x++) {
                printf("%u ", pixels[640*200 + x]);
            }
            printf("\n");

            // Count consecutive zero runs in row 0
            int zeroRuns = 0, maxZeroRun = 0, curZero = 0;
            for (int x = 0; x < 640; x++) {
                if (pixels[x] == 0) {
                    curZero++;
                } else {
                    if (curZero > 0) {
                        zeroRuns++;
                        if (curZero > maxZeroRun) maxZeroRun = curZero;
                    }
                    curZero = 0;
                }
            }
            if (curZero > 0) { zeroRuns++; if (curZero > maxZeroRun) maxZeroRun = curZero; }
            printf("  Row 0 zero runs: %d runs, max length %d\n", zeroRuns, maxZeroRun);
        }

        oniFrameRelease(frame);
    }

    oniStreamStop(stream);
    oniStreamDestroy(stream);
    oniDeviceClose(dev);
    oniShutdown();
    printf("Done\n");
    return 0;
}
