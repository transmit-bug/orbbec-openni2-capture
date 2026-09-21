/**
 * Dump full depth frame to a binary file for calibration.
 * Format: 640x480 uint16_t values (row-major, little-endian).
 */
#include <OpenNI.h>
#include <OniCProperties.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

int main(int argc, char** argv) {
    setvbuf(stdout, NULL, _IONBF, 0);
    setenv("OPENNI2_REPO", "/home/bowmanhan/Code/OrbbecSDK/AstraSDK-v2.1.3/lib/Plugins/openni2/Drivers", 1);

    const char* outfile = "frame_raw.bin";
    if (argc >= 2) {
        outfile = argv[1];
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

    printf("Capturing 3 frames, will save the last one...\n");

    uint16_t best_frame[640*480];
    int best_nonzero = 0;
    uint16_t best_min = 0xFFFF, best_max = 0;

    for (int f = 0; f < 3; f++) {
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

        printf("Frame %d: %d/%d non-zero, min=%u max=%u (%.2fm)\n",
               f, nonZero, total, minV, maxV, maxV/1000.0f);

        if (nonZero > best_nonzero) {
            best_nonzero = nonZero;
            best_min = minV;
            best_max = maxV;
            memcpy(best_frame, pixels, sizeof(best_frame));
        }

        oniFrameRelease(frame);
    }

    oniStreamStop(stream);
    oniStreamDestroy(stream);
    oniDeviceClose(dev);
    oniShutdown();

    // Save best frame
    FILE* fp = fopen(outfile, "wb");
    if (!fp) {
        fprintf(stderr, "Failed to open %s for writing\n", outfile);
        return 1;
    }

    // Write header: width, height, min, max, nonZero count (all uint32 LE)
    uint32_t hdr[5];
    hdr[0] = 640;
    hdr[1] = 480;
    hdr[2] = best_min;
    hdr[3] = best_max;
    hdr[4] = best_nonzero;
    fwrite(hdr, sizeof(uint32_t), 5, fp);

    // Write pixel data
    fwrite(best_frame, sizeof(uint16_t), 640*480, fp);
    fclose(fp);

    printf("Saved %d non-zero pixels to %s (min=%u max=%u)\n",
           best_nonzero, outfile, best_min, best_max);
    return 0;
}
