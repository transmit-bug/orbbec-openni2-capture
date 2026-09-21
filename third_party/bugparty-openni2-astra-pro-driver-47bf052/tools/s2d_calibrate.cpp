/**
 * S2D Multi-Point Calibration Tool
 *
 * Captures raw shift values at multiple known distances, fits the stereo
 * triangulation formula parameters (focalBL, fittingCoeff) using least
 * squares, and outputs the calibrated constants for the driver.
 *
 * Usage:
 *   # Step 1: Capture mode — at each distance, press Enter to sample
 *   ASTRA_S2D_IDENTITY=1 ./s2d_calibrate --capture
 *
 *   # Step 2 (alternative): Manual mode — provide calibration points
 *   ./s2d_calibrate --fit 500,823 700,912 1000,1061 1313,841
 *
 *   # Step 3: Validate — print S2D table comparison
 *   ./s2d_calibrate --validate --focalBL 953451.8 --fittingCoeff 2445.3
 *
 * Formula: depth = focalBL / (fittingCoeff - shift)
 *   Rearranged: fittingCoeff - shift = focalBL / depth
 *   So: shift = fittingCoeff - focalBL / depth
 *   This is linear in shift vs (1/depth), enabling linear regression.
 *
 * Build:
 *   g++ -std=c++17 -o s2d_calibrate s2d_calibrate.cpp -I../AstraSDK-v2.1.3/include -L../AstraSDK-v2.1.3/lib -lOpenNI2
 */

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <vector>
#include <algorithm>
#include <string>
#include <sstream>
#include <fstream>

#ifdef CAPTURE_MODE
#include <OpenNI.h>
#include <OniCProperties.h>
#endif

// --- Calibration data structures ---

struct CalibPoint {
    double shift;     // median shift value from raw frame
    double depth_mm;  // known ground-truth distance in mm
};

struct S2DParams {
    double focalBL;      // focal_length * baseline
    double fittingCoeff;  // zero-disparity shift offset
    double rmsError;      // RMS fitting error in mm
};

// --- Median computation ---

double median(std::vector<uint16_t>& v) {
    if (v.empty()) return 0;
    size_t n = v.size();
    std::sort(v.begin(), v.end());
    if (n % 2 == 0) return (v[n/2 - 1] + v[n/2]) / 2.0;
    return v[n/2];
}

// --- Least-squares fit of stereo formula ---
//
// depth = focalBL / (fittingCoeff - shift)
// => 1/depth = (fittingCoeff - shift) / focalBL
// => shift = fittingCoeff - focalBL * (1/depth)
// This is LINEAR: shift = A + B * (1/depth)  where A=fittingCoeff, B=-focalBL
//
// So we do linear regression: shift = A + B * x, where x = 1/depth_mm

S2DParams fitStereoFormula(const std::vector<CalibPoint>& points) {
    S2DParams result = {0, 0, 0};
    int n = points.size();
    if (n < 2) {
        fprintf(stderr, "Need at least 2 calibration points, got %d\n", n);
        return result;
    }

    // Linear regression: shift = A + B * (1/depth)
    double sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
    for (const auto& p : points) {
        double x = 1.0 / p.depth_mm;  // independent variable
        double y = p.shift;             // dependent variable
        sumX += x;
        sumY += y;
        sumXY += x * y;
        sumX2 += x * x;
    }

    double B = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);
    double A = (sumY - B * sumX) / n;

    result.fittingCoeff = A;
    result.focalBL = -B;  // B is negative because shift = A - focalBL/depth

    // Compute RMS error
    double sumSqErr = 0;
    for (const auto& p : points) {
        double predicted = result.focalBL / (result.fittingCoeff - p.shift);
        double err = predicted - p.depth_mm;
        sumSqErr += err * err;
    }
    result.rmsError = sqrt(sumSqErr / n);

    return result;
}

// --- Print S2D table for given parameters ---

void printS2DTable(double focalBL, double fittingCoeff, int nShiftScale = 1,
                   int minDepth = 300, int maxDepth = 8000, int maxShift = 2048) {
    printf("S2D Table (focalBL=%.1f, fittingCoeff=%.1f, nShiftScale=%d):\n",
           focalBL, fittingCoeff, nShiftScale);
    printf("  shift  →  depth(mm)  →  depth(m)\n");
    int validCount = 0;
    int firstValid = -1, lastValid = -1;
    for (int s = 1; s < maxShift; s++) {
        double disparity = (fittingCoeff - s) / nShiftScale;
        if (disparity <= 0) continue;
        if (disparity >= focalBL) continue;
        double depth = focalBL / disparity;
        if (depth < minDepth || depth > maxDepth) continue;
        if (validCount < 20 || (maxShift - s) < 20) {
            printf("  %4d   →  %7.1f    →  %.3f\n", s, depth, depth / 1000.0);
        } else if (validCount == 20) {
            printf("  ... (skipping middle entries) ...\n");
        }
        validCount++;
        if (firstValid < 0) firstValid = s;
        lastValid = s;
    }
    printf("  Total: %d valid entries, shift range [%d, %d]\n", validCount, firstValid, lastValid);
}

// --- Compare two S2D parameter sets ---

void compareS2D(double old_fBL, double old_fC, double new_fBL, double new_fC,
                int maxShift = 2048) {
    printf("\n=== S2D Parameter Comparison ===\n");
    printf("  Parameter      | Old (current)  | New (fitted)\n");
    printf("  ---------------|----------------|----------------\n");
    printf("  focalBL        | %.1f      | %.1f\n", old_fBL, new_fBL);
    printf("  fittingCoeff   | %.1f     | %.1f\n", old_fC, new_fC);

    printf("\n  shift | old_depth | new_depth | diff(mm) | diff(%%)\n");
    printf("  ------|-----------|-----------|---------|--------\n");

    int minDepth = 300, maxDepth = 8000;
    for (int s = 200; s <= 2000; s += 100) {
        double d_old = 0, d_new = 0;
        double disp_old = (old_fC - s);
        double disp_new = (new_fC - s);
        if (disp_old > 0 && disp_old < old_fBL) d_old = old_fBL / disp_old;
        if (disp_new > 0 && disp_new < new_fBL) d_new = new_fBL / disp_new;
        if (d_old >= minDepth && d_old <= maxDepth && d_new >= minDepth && d_new <= maxDepth) {
            double diff = d_new - d_old;
            double pct = (d_old > 0) ? 100.0 * diff / d_old : 0;
            printf("  %4d  | %7.1f   | %7.1f   | %7.1f | %5.1f%%\n",
                   s, d_old, d_new, diff, pct);
        }
    }
}

// --- Parse calibration points from command line ---
// Format: depth_mm,shift  pairs, e.g. 500,823 700,912

std::vector<CalibPoint> parseCalibPoints(const char* arg) {
    std::vector<CalibPoint> points;
    std::string s(arg);
    std::istringstream ss(s);
    std::string token;
    while (std::getline(ss, token, ' ')) {
        // Parse "depth,shift"
        size_t comma = token.find(',');
        if (comma == std::string::npos) {
            fprintf(stderr, "Bad format: '%s' (expected depth,shift)\n", token.c_str());
            continue;
        }
        CalibPoint p;
        p.depth_mm = std::stod(token.substr(0, comma));
        p.shift = std::stod(token.substr(comma + 1));
        points.push_back(p);
    }
    return points;
}

// ========================================================================
// CAPTURE MODE: Stream raw shifts and sample median at known distances
// ========================================================================

#ifdef CAPTURE_MODE

void captureMode() {
    setenv("OPENNI2_REPO", "/home/bowmanhan/Code/OrbbecSDK/AstraSDK-v2.1.3/lib/Plugins/openni2/Drivers", 1);
    setenv("ASTRA_S2D_IDENTITY", "1", 1);  // Raw shift mode

    OniStatus rc = oniInitialize(ONI_API_VERSION);
    if (rc != ONI_STATUS_OK) { fprintf(stderr, "Init failed: %d\n", rc); return; }

    OniDeviceInfo* devs;
    int n;
    rc = oniGetDeviceList(&devs, &n);
    if (rc != ONI_STATUS_OK || n == 0) { fprintf(stderr, "No devices\n"); oniShutdown(); return; }

    OniDeviceHandle dev;
    rc = oniDeviceOpen(devs[0].uri, &dev);
    if (rc != ONI_STATUS_OK) { fprintf(stderr, "Open failed\n"); oniShutdown(); return; }
    printf("Device: %s\n", devs[0].name);

    OniStreamHandle stream;
    rc = oniDeviceCreateStream(dev, ONI_SENSOR_DEPTH, &stream);
    if (rc != ONI_STATUS_OK) { fprintf(stderr, "Create stream failed\n"); oniDeviceClose(dev); oniShutdown(); return; }

    OniVideoMode mode;
    mode.resolutionX = 640; mode.resolutionY = 480; mode.fps = 30;
    mode.pixelFormat = ONI_PIXEL_FORMAT_DEPTH_1_MM;
    oniStreamSetProperty(stream, ONI_STREAM_PROPERTY_VIDEO_MODE, &mode, sizeof(mode));

    rc = oniStreamStart(stream);
    if (rc != ONI_STATUS_OK) { fprintf(stderr, "Start failed: %d\n", rc); return; }

    printf("\n=== S2D Multi-Point Calibration ===\n");
    printf("Place a flat target (wall, whiteboard, etc.) at a KNOWN distance.\n");
    printf("For each distance:\n");
    printf("  1. Type the distance in mm and press Enter (e.g. 500)\n");
    printf("  2. The tool captures 10 frames and computes the median shift\n");
    printf("  3. Repeat for 3+ distances (recommended: 0.5m, 1.0m, 2.0m, 3.0m)\n");
    printf("  4. Type 'fit' to fit the S2D formula\n");
    printf("  5. Type 'quit' to exit\n\n");

    std::vector<CalibPoint> calibPoints;

    // Center region for median sampling (avoid edges)
    const int W = 640, H = 480;
    const int ROI_X1 = W/2 - 40, ROI_X2 = W/2 + 40;  // 80px wide center strip
    const int ROI_Y1 = H/2 - 40, ROI_Y2 = H/2 + 40;  // 80px tall center strip

    char line[256];
    while (true) {
        printf("Enter distance (mm) or 'fit' or 'quit': ");
        fflush(stdout);
        if (!fgets(line, sizeof(line), stdin)) break;
        line[strcspn(line, "\n")] = 0;

        if (strcmp(line, "quit") == 0) break;
        if (strcmp(line, "fit") == 0) {
            if (calibPoints.size() < 2) {
                printf("Need at least 2 calibration points!\n");
                continue;
            }

            printf("\n--- Calibration Points ---\n");
            for (const auto& p : calibPoints) {
                printf("  shift=%.1f → depth=%.0fmm (%.2fm)\n",
                       p.shift, p.depth_mm, p.depth_mm / 1000.0);
            }

            S2DParams params = fitStereoFormula(calibPoints);
            printf("\n--- Fitted Parameters ---\n");
            printf("  focalBL      = %.1f\n", params.focalBL);
            printf("  fittingCoeff = %.1f\n", params.fittingCoeff);
            printf("  RMS error    = %.1f mm\n", params.rmsError);

            // Compare with current hardcoded values
            compareS2D(953451.8, 2445.3, params.focalBL, params.fittingCoeff);

            printf("\n--- To update the driver, edit AstraDevice.cpp: ---\n");
            printf("  const double focalBL = %.1f;\n", params.focalBL);
            printf("  const double fittingCoeff = %.1f;\n", params.fittingCoeff);

            // Save calibration to file
            std::ofstream calFile("/tmp/s2d_calibration.txt");
            calFile << "# S2D Calibration — " << calibPoints.size() << " points\n";
            calFile << "# shift,depth_mm\n";
            for (const auto& p : calibPoints) {
                calFile << p.shift << "," << p.depth_mm << "\n";
            }
            calFile << "# Fitted parameters\n";
            calFile << "focalBL=" << params.focalBL << "\n";
            calFile << "fittingCoeff=" << params.fittingCoeff << "\n";
            calFile << "rmsError=" << params.rmsError << "\n";
            calFile.close();
            printf("\nCalibration saved to /tmp/s2d_calibration.txt\n");
            continue;
        }

        // Parse distance
        double distance_mm = atof(line);
        if (distance_mm < 100 || distance_mm > 10000) {
            printf("Distance out of range (100-10000mm)\n");
            continue;
        }

        printf("Capturing at %.0fmm (%.2fm) — hold target steady...\n", distance_mm, distance_mm / 1000.0);

        // Capture 10 frames, collect all valid shifts from center ROI
        std::vector<uint16_t> allShifts;
        int framesCaptured = 0;
        for (int f = 0; f < 30 && framesCaptured < 10; f++) {
            OniFrame* frame;
            rc = oniStreamReadFrame(stream, &frame);
            if (rc != ONI_STATUS_OK || !frame || frame->dataSize == 0) continue;

            uint16_t* pixels = (uint16_t*)frame->data;
            int validInFrame = 0;
            for (int y = ROI_Y1; y < ROI_Y2 && y < frame->height; y++) {
                for (int x = ROI_X1; x < ROI_X2 && x < frame->width; x++) {
                    uint16_t s = pixels[y * frame->width + x];
                    if (s > 0 && s < 2048) {
                        allShifts.push_back(s);
                        validInFrame++;
                    }
                }
            }
            framesCaptured++;
            printf("  Frame %d: %d valid shifts in ROI\n", framesCaptured, validInFrame);
            oniFrameRelease(frame);
        }

        if (allShifts.empty()) {
            printf("  No valid shifts captured! Check laser and target.\n");
            continue;
        }

        double medShift = median(allShifts);
        printf("  Median shift: %.1f (from %zu samples, %d frames)\n",
               medShift, allShifts.size(), framesCaptured);
        printf("  → shift=%.1f, depth=%.0fmm\n\n", medShift, distance_mm);

        calibPoints.push_back({medShift, distance_mm});
    }

    oniStreamStop(stream);
    oniStreamDestroy(stream);
    oniDeviceClose(dev);
    oniShutdown();
}

#endif  // CAPTURE_MODE

// ========================================================================
// FIT MODE: Fit parameters from manually provided calibration points
// ========================================================================

void fitMode(const char* pointsArg) {
    auto points = parseCalibPoints(pointsArg);
    if (points.size() < 2) {
        fprintf(stderr, "Need at least 2 points\n");
        return;
    }

    printf("--- Input Calibration Points ---\n");
    for (const auto& p : points) {
        printf("  shift=%.1f → depth=%.0fmm (%.2fm)\n",
               p.shift, p.depth_mm, p.depth_mm / 1000.0);
    }

    S2DParams params = fitStereoFormula(points);
    printf("\n--- Fitted Parameters ---\n");
    printf("  focalBL      = %.1f\n", params.focalBL);
    printf("  fittingCoeff = %.1f\n", params.fittingCoeff);
    printf("  RMS error    = %.1f mm\n", params.rmsError);

    // Per-point errors
    printf("\n--- Per-Point Errors ---\n");
    for (const auto& p : points) {
        double predicted = params.focalBL / (params.fittingCoeff - p.shift);
        double err = predicted - p.depth_mm;
        printf("  shift=%.1f → predicted=%.1fmm, actual=%.0fmm, error=%.1fmm (%.2f%%)\n",
               p.shift, predicted, p.depth_mm, err, 100.0 * err / p.depth_mm);
    }

    compareS2D(953451.8, 2445.3, params.focalBL, params.fittingCoeff);

    printf("\n--- To update the driver, edit AstraDevice.cpp: ---\n");
    printf("  const double focalBL = %.1f;\n", params.focalBL);
    printf("  const double fittingCoeff = %.1f;\n", params.fittingCoeff);
}

// ========================================================================
// VALIDATE MODE: Print S2D table for given parameters
// ========================================================================

void validateMode(double focalBL, double fittingCoeff) {
    printS2DTable(focalBL, fittingCoeff);

    if (focalBL != 953451.8 || fittingCoeff != 2445.3) {
        printf("\n--- Current driver parameters for comparison ---\n");
        printS2DTable(953451.8, 2445.3);
        compareS2D(953451.8, 2445.3, focalBL, fittingCoeff);
    }
}

// ========================================================================
// MAIN
// ========================================================================

void printUsage(const char* prog) {
    printf("S2D Calibration Tool\n\n");
    printf("Usage:\n");
#ifdef CAPTURE_MODE
    printf("  %s --capture\n", prog);
    printf("    Interactive capture: place target at known distances, press Enter\n");
    printf("    Requires ASTRA_S2D_IDENTITY=1 (set automatically)\n\n");
#endif
    printf("  %s --fit <depth1,shift1> <depth2,shift2> ...\n", prog);
    printf("    Fit from manual calibration points\n");
    printf("    Example: --fit 500,823 700,912 1000,1061 1313,841\n\n");
    printf("  %s --validate [--focalBL F] [--fittingCoeff C]\n", prog);
    printf("    Print S2D table for given parameters\n\n");
    printf("  %s --file <path>\n", prog);
    printf("    Read calibration points from file (format: shift,depth_mm per line)\n");
}

int main(int argc, char** argv) {
    if (argc < 2) { printUsage(argv[0]); return 1; }

    std::string mode = argv[1];

    if (mode == "--capture") {
#ifdef CAPTURE_MODE
        captureMode();
#else
        fprintf(stderr, "Capture mode requires CAPTURE_MODE macro. Rebuild with -DCAPTURE_MODE\n");
        return 1;
#endif
    } else if (mode == "--fit") {
        if (argc < 3) {
            fprintf(stderr, "Usage: --fit <depth1,shift1> <depth2,shift2> ...\n");
            return 1;
        }
        // Join remaining args as one string
        std::string pointsStr;
        for (int i = 2; i < argc; i++) {
            if (i > 2) pointsStr += " ";
            pointsStr += argv[i];
        }
        fitMode(pointsStr.c_str());
    } else if (mode == "--validate") {
        double fBL = 953451.8, fC = 2445.3;
        for (int i = 2; i < argc - 1; i++) {
            if (strcmp(argv[i], "--focalBL") == 0) fBL = atof(argv[++i]);
            else if (strcmp(argv[i], "--fittingCoeff") == 0) fC = atof(argv[++i]);
        }
        validateMode(fBL, fC);
    } else if (mode == "--file") {
        if (argc < 3) {
            fprintf(stderr, "Usage: --file <path>\n");
            return 1;
        }
        std::ifstream f(argv[2]);
        if (!f.is_open()) {
            fprintf(stderr, "Cannot open %s\n", argv[2]);
            return 1;
        }
        std::vector<CalibPoint> points;
        std::string line;
        while (std::getline(f, line)) {
            if (line.empty() || line[0] == '#') continue;
            size_t comma = line.find(',');
            if (comma == std::string::npos) continue;
            CalibPoint p;
            p.shift = std::stod(line.substr(0, comma));
            p.depth_mm = std::stod(line.substr(comma + 1));
            points.push_back(p);
        }
        if (points.size() < 2) {
            fprintf(stderr, "Need at least 2 points in file\n");
            return 1;
        }
        S2DParams params = fitStereoFormula(points);
        printf("--- Fitted Parameters from %s ---\n", argv[2]);
        printf("  focalBL      = %.1f\n", params.focalBL);
        printf("  fittingCoeff = %.1f\n", params.fittingCoeff);
        printf("  RMS error    = %.1f mm\n", params.rmsError);
        compareS2D(953451.8, 2445.3, params.focalBL, params.fittingCoeff);
    } else {
        printUsage(argv[0]);
        return 1;
    }

    return 0;
}
