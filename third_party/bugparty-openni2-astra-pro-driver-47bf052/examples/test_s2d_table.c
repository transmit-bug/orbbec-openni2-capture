/**
 * Dump the S2D table to understand which shift values map to valid depth.
 * Also decode raw PSCompressed data with shift values (not depth) to see
 * what the decoder is producing before S2D conversion.
 */
#include <OpenNI.h>
#include <OniCProperties.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <math.h>

// PrimeSense S2D formula (same as AstraDevice::computeShiftToDepthTable)
void computeS2D(uint16_t* table, int maxShift) {
    memset(table, 0, maxShift * sizeof(uint16_t));
    const double ZPD = 100.0;
    const double ZPPS = 0.03;
    const double EmitterD = 51.0;
    const double nParamCoeff = 4.0;
    const double nConstShift_raw = -1171.4247;
    const double nShiftScale = 1.0;
    const double minCutoff = 322.0;
    const double maxCutoff = 10000.0;
    const int pixelSizeFactor = 2;  // VGA

    double dPlanePixelSize = ZPPS * pixelSizeFactor;
    double nConstShift = (nParamCoeff * nConstShift_raw) / pixelSizeFactor;

    for (int s = 1; s < maxShift; s++) {
        double refX = (s - nConstShift) / nParamCoeff - 0.375;
        double metric = refX * dPlanePixelSize;
        if (metric < 0.0 || metric >= EmitterD) continue;
        double depth = nShiftScale * (metric * ZPD / (EmitterD - metric) + ZPD);
        if (depth >= minCutoff && depth < maxCutoff)
            table[s] = (uint16_t)depth;
    }
}

int main() {
    setvbuf(stdout, NULL, _IONBF, 0);

    // 1. Print S2D table statistics
    uint16_t s2d[2048];
    computeS2D(s2d, 2048);

    int validCount = 0;
    int firstValid = -1, lastValid = -1;
    for (int i = 0; i < 2048; i++) {
        if (s2d[i] != 0) {
            validCount++;
            if (firstValid < 0) firstValid = i;
            lastValid = i;
        }
    }
    printf("S2D table: %d/%d valid entries, range [%d, %d]\n", validCount, 2048, firstValid, lastValid);
    printf("First 20 entries: ");
    for (int i = 0; i < 20; i++) printf("[%d]=%u ", i, s2d[i]);
    printf("\n");
    printf("Entries around 500: ");
    for (int i = 490; i < 520; i++) printf("[%d]=%u ", i, s2d[i]);
    printf("\n");
    printf("Last 20 valid entries: ");
    for (int i = lastValid - 5; i <= lastValid; i++) printf("[%d]=%u ", i, s2d[i]);
    printf("\n");

    // 2. Also read depth frames and compute per-pixel shift histogram
    // (infer shift from depth by reverse-mapping)
    setenv("OPENNI2_REPO", "/home/bowmanhan/Code/OrbbecSDK/AstraSDK-v2.1.3/lib/Plugins/openni2/Drivers", 1);

    OniStatus rc = oniInitialize(ONI_API_VERSION);
    if (rc != ONI_STATUS_OK) { fprintf(stderr, "Init failed\n"); return 1; }

    OniDeviceInfo* devs; int n;
    rc = oniGetDeviceList(&devs, &n);
    if (rc != ONI_STATUS_OK || n == 0) { fprintf(stderr, "No devices\n"); oniShutdown(); return 1; }

    OniDeviceHandle dev;
    rc = oniDeviceOpen(devs[0].uri, &dev);
    if (rc != ONI_STATUS_OK) { fprintf(stderr, "Open failed\n"); oniShutdown(); return 1; }

    OniStreamHandle stream;
    rc = oniDeviceCreateStream(dev, ONI_SENSOR_DEPTH, &stream);
    if (rc != ONI_STATUS_OK) { fprintf(stderr, "Create stream failed\n"); return 1; }

    OniVideoMode mode;
    mode.resolutionX = 640; mode.resolutionY = 480; mode.fps = 30;
    mode.pixelFormat = ONI_PIXEL_FORMAT_DEPTH_1_MM;
    oniStreamSetProperty(stream, ONI_STREAM_PROPERTY_VIDEO_MODE, &mode, sizeof(mode));

    rc = oniStreamStart(stream);
    if (rc != ONI_STATUS_OK) { fprintf(stderr, "Start failed\n"); return 1; }

    // Read 3 frames and analyze
    for (int f = 0; f < 3; f++) {
        OniFrame* frame;
        rc = oniStreamReadFrame(stream, &frame);
        if (rc != ONI_STATUS_OK || !frame) continue;

        uint16_t* pixels = (uint16_t*)frame->data;
        int total = frame->width * frame->height;

        // Histogram of depth values
        int zeroCount = 0;
        int hist[1024] = {0};
        for (int i = 0; i < total; i++) {
            if (pixels[i] == 0) zeroCount++;
            else if (pixels[i] < 1024) hist[pixels[i]]++;
        }

        // Find top 10 most common non-zero depth values
        printf("\nFrame %d: %d/%d zero (%.0f%%)\n", f, zeroCount, total, 100.0*zeroCount/total);
        printf("Top depth values: ");
        for (int rep = 0; rep < 20; rep++) {
            int best = 0, bestIdx = -1;
            for (int i = 1; i < 1024; i++) {
                if (hist[i] > best) { best = hist[i]; bestIdx = i; }
            }
            if (bestIdx < 0) break;
            printf("%dmm(%dpx) ", bestIdx, best);
            hist[bestIdx] = 0;
        }
        printf("\n");

        oniFrameRelease(frame);
    }

    oniStreamStop(stream);
    oniStreamDestroy(stream);
    oniDeviceClose(dev);
    oniShutdown();
    return 0;
}
