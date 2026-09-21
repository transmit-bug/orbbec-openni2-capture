/// Minimal demo: open Astra Pro, read one depth + one color frame, save to files.
/// Usage: ./astra_capture

#include "camera_openni.h"
#include <cstdio>
#include <cstring>
#include <cstdlib>

static bool savePPM(const char* path, const uint8_t* data, int w, int h) {
    FILE* f = fopen(path, "wb");
    if (!f) return false;
    fprintf(f, "P6\n%d %d\n255\n", w, h);
    fwrite(data, 1, w * h * 3, f);
    fclose(f);
    return true;
}

static bool savePGM16(const char* path, const uint16_t* data, int w, int h) {
    FILE* f = fopen(path, "wb");
    if (!f) return false;
    // PGM big-endian, 16-bit
    fprintf(f, "P5\n%d %d\n65535\n", w, h);
    for (int i = 0; i < w * h; ++i) {
        uint8_t be[2] = { (uint8_t)(data[i] >> 8), (uint8_t)(data[i] & 0xFF) };
        fwrite(be, 1, 2, f);
    }
    fclose(f);
    return true;
}

int main() {
    // Required: old Astra Pro firmware 5.8 uses PrimeSense depth format
    setenv("ASTRA_DEPTH_FORMAT", "ps", 1);
    // Workaround for glibc malloc corruption in OpenNI2 beta
    setenv("MALLOC_CHECK_", "0", 1);

    printf("[main] Starting...\n"); fflush(stdout);
    CameraOpenNI cam;

    if (!cam.open()) {
        fprintf(stderr, "ERROR: %s\n", cam.lastError().c_str());
        fprintf(stderr, "Hint: run 'sudo ./third_party/OpenNI_2.3*/rules/install.sh'\n");
        // Give OpenNI driver thread time to clean up before exit
        usleep(100000);
        return 1;
    }

    printf("[main] Device opened OK\n"); fflush(stdout);
    if (!cam.startDepth()) {
        fprintf(stderr, "ERROR startDepth: %s\n", cam.lastError().c_str());
        return 2;
    }

    printf("[main] Depth stream started, reading frame...\n"); fflush(stdout);
    cam.startColor();
    cam.enableRegistration();
    printf("[main] Calling readDepth with 3s timeout...\n"); fflush(stdout);

    // Read one depth frame
    std::vector<uint16_t> depthData;
    int dw, dh;
    bool depthOk = cam.readDepth(depthData, dw, dh, 3000);
    printf("[main] readDepth returned: %s\n", depthOk ? "OK" : "FAIL"); fflush(stdout);
    if (depthOk) {
        char path[256];
        snprintf(path, sizeof(path), "output/depth_%dx%d.pgm", dw, dh);
        if (savePGM16(path, depthData.data(), dw, dh)) {
            // Print stats
            uint16_t minD = 65535, maxD = 0;
            int valid = 0;
            for (auto d : depthData) {
                if (d > 0 && d < 65535) { ++valid; if (d < minD) minD = d; if (d > maxD) maxD = d; }
            }
            printf("[OK] Depth saved: %s (%dx%d, valid=%d, range=%d-%d mm)\n",
                   path, dw, dh, valid, minD, maxD);
        }
    } else {
        fprintf(stderr, "ERROR readDepth: %s\n", cam.lastError().c_str());
    }

    // Read one color frame
    if (cam.isColorRunning()) {
        std::vector<uint8_t> colorData;
        int cw, ch;
        if (cam.readColor(colorData, cw, ch)) {
            char path[256];
            snprintf(path, sizeof(path), "output/color_%dx%d.ppm", cw, ch);
            if (savePPM(path, colorData.data(), cw, ch)) {
                printf("[OK] Color saved: %s (%dx%d)\n", path, cw, ch);
            }
        } else {
            fprintf(stderr, "WARN readColor: %s\n", cam.lastError().c_str());
        }
    }

    cam.close();
    printf("Done.\n");
    return 0;
}
