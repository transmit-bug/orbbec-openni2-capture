/**
 * Automated depth comparison tool.
 *
 * Captures a frame from whatever driver is currently deployed,
 * then compares it against a reference frame dump (if provided).
 *
 * Usage:
 *   dump_compare                    # capture new frame to /tmp/our_depth.bin
 *   dump_compare /tmp/ref.bin      # capture + compare against reference
 *   dump_compare /tmp/ref.bin /tmp/our_depth.bin  # save new capture to specified path
 *
 * Output (with reference):
 *   - Per-pixel mean absolute error (MAE)
 *   - Mean absolute percentage error (MAPE)
 *   - Error distribution (histogram)
 *   - Region-of-interest analysis (center 11x11 window)
 */
#include <OpenNI.h>
#include <OniCProperties.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

#define WIDTH 640
#define HEIGHT 480
#define TOTAL (WIDTH * HEIGHT)

typedef struct {
    uint16_t pixels[TOTAL];
    uint16_t min_val, max_val;
    int nonzero_count;
} Frame;

static int capture_frame(Frame* frame) {
    OniStatus rc = oniInitialize(ONI_API_VERSION);
    if (rc != ONI_STATUS_OK) { fprintf(stderr, "Init failed: %d\n", rc); return -1; }

    OniDeviceInfo* devs;
    int n;
    rc = oniGetDeviceList(&devs, &n);
    if (rc != ONI_STATUS_OK || n == 0) {
        fprintf(stderr, "No devices\n"); oniShutdown(); return -1;
    }

    OniDeviceHandle dev;
    rc = oniDeviceOpen(devs[0].uri, &dev);
    if (rc != ONI_STATUS_OK) {
        fprintf(stderr, "Open failed\n"); oniShutdown(); return -1;
    }
    fprintf(stderr, "Device: %s\n", devs[0].name);

    OniStreamHandle stream;
    rc = oniDeviceCreateStream(dev, ONI_SENSOR_DEPTH, &stream);
    if (rc != ONI_STATUS_OK) {
        fprintf(stderr, "Create stream failed\n"); oniDeviceClose(dev); oniShutdown(); return -1;
    }

    OniVideoMode mode;
    mode.resolutionX = WIDTH; mode.resolutionY = HEIGHT; mode.fps = 30;
    mode.pixelFormat = ONI_PIXEL_FORMAT_DEPTH_1_MM;
    oniStreamSetProperty(stream, ONI_STREAM_PROPERTY_VIDEO_MODE, &mode, sizeof(mode));

    rc = oniStreamStart(stream);
    if (rc != ONI_STATUS_OK) {
        fprintf(stderr, "Start failed: %d\n", rc); return -1;
    }

    memset(frame->pixels, 0, sizeof(frame->pixels));
    frame->min_val = 0xFFFF;
    frame->max_val = 0;
    frame->nonzero_count = 0;

    int best_nonzero = 0;

    // Capture 5 frames, pick the one with most non-zero pixels
    for (int f = 0; f < 5; f++) {
        OniFrame* oframe;
        rc = oniStreamReadFrame(stream, &oframe);
        if (rc != ONI_STATUS_OK || !oframe || oframe->dataSize == 0) {
            fprintf(stderr, "Frame %d: read failed\n", f);
            continue;
        }

        uint16_t* pixels = (uint16_t*)oframe->data;
        int total = oframe->width * oframe->height;
        int nz = 0;
        uint16_t mn = 0xFFFF, mx = 0;
        for (int i = 0; i < total; i++) {
            if (pixels[i] != 0) {
                nz++;
                if (pixels[i] < mn) mn = pixels[i];
                if (pixels[i] > mx) mx = pixels[i];
            }
        }
        fprintf(stderr, "  Frame %d: %d/%d non-zero (%.1f%%), min=%u max=%u\n",
                f, nz, total, 100.0 * nz / total, mn, mx);

        if (nz > best_nonzero) {
            best_nonzero = nz;
            memcpy(frame->pixels, pixels, sizeof(frame->pixels));
            frame->min_val = mn;
            frame->max_val = mx;
            frame->nonzero_count = nz;
        }
        oniFrameRelease(oframe);
    }

    oniStreamStop(stream);
    oniStreamDestroy(stream);
    oniDeviceClose(dev);
    oniShutdown();

    fprintf(stderr, "Best frame: %d non-zero (%.1f%%), min=%u max=%u\n",
            frame->nonzero_count, 100.0 * frame->nonzero_count / TOTAL,
            frame->min_val, frame->max_val);

    return 0;
}

static int save_frame(const Frame* frame, const char* path) {
    FILE* fp = fopen(path, "wb");
    if (!fp) { fprintf(stderr, "Cannot open %s for writing\n", path); return -1; }

    uint32_t hdr[5];
    hdr[0] = WIDTH;
    hdr[1] = HEIGHT;
    hdr[2] = frame->min_val;
    hdr[3] = frame->max_val;
    hdr[4] = frame->nonzero_count;
    fwrite(hdr, sizeof(uint32_t), 5, fp);
    fwrite(frame->pixels, sizeof(uint16_t), TOTAL, fp);
    fclose(fp);

    fprintf(stderr, "Saved to %s\n", path);
    return 0;
}

static int load_frame(Frame* frame, const char* path) {
    FILE* fp = fopen(path, "rb");
    if (!fp) { fprintf(stderr, "Cannot open %s for reading\n", path); return -1; }

    uint32_t hdr[5];
    if (fread(hdr, sizeof(uint32_t), 5, fp) != 5) {
        fprintf(stderr, "Header read failed\n"); fclose(fp); return -1;
    }
    if (fread(frame->pixels, sizeof(uint16_t), TOTAL, fp) != TOTAL) {
        fprintf(stderr, "Pixel read failed\n"); fclose(fp); return -1;
    }

    frame->min_val = hdr[2];
    frame->max_val = hdr[3];
    frame->nonzero_count = hdr[4];
    fclose(fp);

    fprintf(stderr, "Loaded %s: %d non-zero, min=%u max=%u\n",
            path, frame->nonzero_count, frame->min_val, frame->max_val);
    return 0;
}

static void compare_frames(const Frame* ref, const Frame* test) {
    // Only compare pixels where BOTH are non-zero
    int n = 0;
    double sum_abs_err = 0;
    double sum_pct_err = 0;
    double sum_sq_err = 0;
    int max_abs_err = 0;

    // Error histogram bins: 0, 1, 2, 5, 10, 20, 50, 100+
    int hist_0 = 0, hist_1 = 0, hist_2 = 0, hist_5 = 0;
    int hist_10 = 0, hist_20 = 0, hist_50 = 0, hist_100 = 0;

    for (int i = 0; i < TOTAL; i++) {
        if (ref->pixels[i] != 0 && test->pixels[i] != 0) {
            int err = (int)test->pixels[i] - (int)ref->pixels[i];
            int abs_err = abs(err);

            sum_abs_err += abs_err;
            sum_pct_err += 100.0 * abs_err / ref->pixels[i];
            sum_sq_err += (double)err * err;

            if (abs_err > max_abs_err) max_abs_err = abs_err;

            if (abs_err == 0) hist_0++;
            else if (abs_err <= 1) hist_1++;
            else if (abs_err <= 2) hist_2++;
            else if (abs_err <= 5) hist_5++;
            else if (abs_err <= 10) hist_10++;
            else if (abs_err <= 20) hist_20++;
            else if (abs_err <= 50) hist_50++;
            else hist_100++;

            n++;
        }
    }

    // Center ROI (11x11 at cx=320,cy=240)
    int roi_n = 0;
    double roi_sum_err = 0;
    int roi_max = 0;
    for (int y = 235; y <= 245; y++) {
        for (int x = 315; x <= 325; x++) {
            int i = y * WIDTH + x;
            if (ref->pixels[i] != 0 && test->pixels[i] != 0) {
                int err = abs((int)test->pixels[i] - (int)ref->pixels[i]);
                roi_sum_err += err;
                roi_n++;
                if (err > roi_max) roi_max = err;
            }
        }
    }

    printf("\n========== COMPARISON RESULTS ==========\n");
    printf("Valid pixel pairs: %d / %d (%.1f%%)\n", n, TOTAL, 100.0 * n / TOTAL);
    printf("Reference: %d non-zero (min=%u max=%u)\n", ref->nonzero_count, ref->min_val, ref->max_val);
    printf("Test:      %d non-zero (min=%u max=%u)\n", test->nonzero_count, test->min_val, test->max_val);

    if (n > 0) {
        double mae = sum_abs_err / n;
        double mape = sum_pct_err / n;
        double rmse = sqrt(sum_sq_err / n);
        printf("\n--- Full Frame ---\n");
        printf("  MAE:  %.1f mm\n", mae);
        printf("  RMSE: %.1f mm\n", rmse);
        printf("  MAPE: %.2f%%\n", mape);
        printf("  Max abs error: %d mm\n", max_abs_err);

        printf("\n--- Error Histogram ---\n");
        printf("  = 0mm:  %6d (%5.1f%%)\n", hist_0, 100.0*hist_0/n);
        printf("  <= 1mm: %6d (%5.1f%%)\n", hist_1, 100.0*hist_1/n);
        printf("  <= 2mm: %6d (%5.1f%%)\n", hist_2, 100.0*hist_2/n);
        printf("  <= 5mm: %6d (%5.1f%%)\n", hist_5, 100.0*hist_5/n);
        printf("  <=10mm: %6d (%5.1f%%)\n", hist_10, 100.0*hist_10/n);
        printf("  <=20mm: %6d (%5.1f%%)\n", hist_20, 100.0*hist_20/n);
        printf("  <=50mm: %6d (%5.1f%%)\n", hist_50, 100.0*hist_50/n);
        printf("  > 50mm: %6d (%5.1f%%)\n", hist_100, 100.0*hist_100/n);
    }

    if (roi_n > 0) {
        printf("\n--- Center ROI (11x11) ---\n");
        printf("  Mean abs error: %.1f mm\n", roi_sum_err / roi_n);
        printf("  Max abs error:  %d mm\n", roi_max);
        printf("  Valid pixels:   %d\n", roi_n);
    }

    printf("========================================\n");
}

int main(int argc, char** argv) {
    setvbuf(stdout, NULL, _IONBF, 0);

    const char* ref_path = NULL;
    const char* out_path = "/tmp/dump_compare_out.bin";

    if (argc >= 2) ref_path = argv[1];
    if (argc >= 3) out_path = argv[2];

    // Capture frame
    fprintf(stderr, "Capturing frame from current driver...\n");
    Frame test_frame;
    if (capture_frame(&test_frame) != 0) return 1;

    // Save captured frame
    save_frame(&test_frame, out_path);

    // Compare if reference provided
    if (ref_path) {
        Frame ref_frame;
        if (load_frame(&ref_frame, ref_path) != 0) return 1;
        compare_frames(&ref_frame, &test_frame);
    }

    return 0;
}
