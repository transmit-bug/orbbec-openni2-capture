/// Astra Pro 采集程序。
///
/// 两种模式：
///   ./astra_capture                              # 默认：读一帧深度 + 一帧彩色并落盘
///   ./astra_capture --record N --out DIR [...]   # 录一段静态深度序列，供 Python 分析
///
/// 序列模式只录深度，不录彩色：调研只需要几何，且 Astra Pro 的彩色是独立的 UVC
/// 设备，同时开启会增加 USB 带宽压力与丢帧风险。
///
/// 文件名必须零填充（``frame_%04d.pgm``）：Python 侧按字典序读入，
/// 否则 ``frame_10`` 会排到 ``frame_2`` 前面。

#include "camera_openni.h"
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cerrno>
#include <ctime>
#include <string>
#include <sys/stat.h>

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
    // PGM big-endian, 16-bit. Python 侧的读取器依赖这个格式逐字节一致。
    fprintf(f, "P5\n%d %d\n65535\n", w, h);
    for (int i = 0; i < w * h; ++i) {
        uint8_t be[2] = { (uint8_t)(data[i] >> 8), (uint8_t)(data[i] & 0xFF) };
        fwrite(be, 1, 2, f);
    }
    fclose(f);
    return true;
}

/// 递归创建目录（等价于 mkdir -p）。C++14 没有 std::filesystem。
static bool ensureDir(const std::string& path) {
    std::string current;
    for (size_t i = 0; i < path.size(); ++i) {
        current += path[i];
        if (path[i] != '/' && i + 1 != path.size()) continue;
        std::string dir = current;
        while (dir.size() > 1 && dir.back() == '/') dir.pop_back();
        if (dir.empty() || dir == "/") continue;
        if (mkdir(dir.c_str(), 0755) != 0 && errno != EEXIST) {
            fprintf(stderr, "ERROR mkdir %s: %s\n", dir.c_str(), strerror(errno));
            return false;
        }
    }
    return true;
}

static std::string isoUtcNow() {
    std::time_t now = std::time(nullptr);
    std::tm tm{};
    gmtime_r(&now, &tm);
    char buffer[32];
    std::strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%SZ", &tm);
    return std::string(buffer);
}

/// 把一次录制的旁路信息写成 metadata.json，让分析结论可以追溯到采集条件。
static bool writeMetadata(const std::string& path, int frameCount,
                          bool hasWorkingDistance, double workingDistanceMm) {
    FILE* f = fopen(path.c_str(), "wb");
    if (!f) {
        fprintf(stderr, "ERROR: 无法写入 %s\n", path.c_str());
        return false;
    }
    fprintf(f,
            "{\n"
            "  \"frame_count\": %d,\n"
            "  \"working_distance_mm\": %s,\n"
            "  \"registered\": false,\n"
            "  \"created\": \"%s\",\n"
            "  \"source\": \"astra_capture\",\n"
            "  \"depth_only\": true\n"
            "}\n",
            frameCount,
            hasWorkingDistance ? std::to_string(workingDistanceMm).c_str() : "null",
            isoUtcNow().c_str());
    fclose(f);
    return true;
}

// ---------------------------------------------------------------------------
// 默认模式：读一帧深度 + 一帧彩色
// ---------------------------------------------------------------------------
static int runOnce() {
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

// ---------------------------------------------------------------------------
// 序列模式：录 N 帧静态深度
// ---------------------------------------------------------------------------
static int runRecord(int frameCount, int warmup, const std::string& outDir,
                     bool hasWorkingDistance, double workingDistanceMm) {
    if (!ensureDir(outDir)) return 3;

    printf("[record] 开始：%d 帧，预热 %d 帧，输出 %s\n",
           frameCount, warmup, outDir.c_str());
    fflush(stdout);

    CameraOpenNI cam;
    if (!cam.open()) {
        fprintf(stderr, "ERROR: %s\n", cam.lastError().c_str());
        fprintf(stderr, "Hint: run 'sudo ./third_party/OpenNI_2.3*/rules/install.sh'\n");
        usleep(100000);
        return 1;
    }
    if (!cam.startDepth()) {
        fprintf(stderr, "ERROR startDepth: %s\n", cam.lastError().c_str());
        return 2;
    }

    // 预热：流刚启动的头几帧往往是坏的（自动曝光/增益未收敛）。
    // 把它们录进去会污染整个时间累积结果，而且从数据上很难看出来。
    std::vector<uint16_t> scratch;
    int scratchW = 0, scratchH = 0;
    for (int i = 0; i < warmup; ++i) {
        cam.readDepth(scratch, scratchW, scratchH, 3000);
    }
    printf("[record] 预热完成，开始录制...\n"); fflush(stdout);

    std::vector<uint16_t> depthData;
    int width = 0, height = 0;
    int saved = 0;

    for (int i = 0; i < frameCount; ++i) {
        if (!cam.readDepth(depthData, width, height, 3000)) {
            fprintf(stderr, "WARN 第 %d 帧读取失败: %s\n", i, cam.lastError().c_str());
            continue;
        }
        char path[512];
        snprintf(path, sizeof(path), "%s/frame_%04d.pgm", outDir.c_str(), i);
        if (!savePGM16(path, depthData.data(), width, height)) {
            fprintf(stderr, "ERROR 写入失败: %s\n", path);
            cam.close();
            return 4;
        }
        ++saved;
        if ((i + 1) % 30 == 0 || i + 1 == frameCount) {
            printf("[record] %d/%d\n", i + 1, frameCount);
            fflush(stdout);
        }
    }

    const std::string metadataPath = outDir + "/metadata.json";
    writeMetadata(metadataPath, saved, hasWorkingDistance, workingDistanceMm);

    cam.close();
    printf("[record] 完成：%d/%d 帧写入 %s (%dx%d)\n",
           saved, frameCount, outDir.c_str(), width, height);
    if (saved < frameCount) {
        fprintf(stderr, "WARN 有 %d 帧读取失败，实际录到 %d 帧\n", frameCount - saved, saved);
    }
    return saved > 0 ? 0 : 5;
}

static void printUsage(const char* program) {
    fprintf(stderr,
            "用法:\n"
            "  %s                                            读一帧深度+彩色\n"
            "  %s --record N --out DIR [--warmup M] \\\n"
            "       [--working-distance-mm D]                录 N 帧静态深度序列\n",
            program, program);
}

int main(int argc, char** argv) {
    // Required: old Astra Pro firmware 5.8 uses PrimeSense depth format
    setenv("ASTRA_DEPTH_FORMAT", "ps", 1);
    // Workaround for glibc malloc corruption in OpenNI2 beta
    setenv("MALLOC_CHECK_", "0", 1);

    int frameCount = 0;
    int warmup = 10;
    std::string outDir;
    bool hasWorkingDistance = false;
    double workingDistanceMm = 0.0;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        const bool hasValue = (i + 1 < argc);
        if (arg == "--record" && hasValue) {
            frameCount = std::atoi(argv[++i]);
        } else if (arg == "--out" && hasValue) {
            outDir = argv[++i];
        } else if (arg == "--warmup" && hasValue) {
            warmup = std::atoi(argv[++i]);
        } else if (arg == "--working-distance-mm" && hasValue) {
            workingDistanceMm = std::atof(argv[++i]);
            hasWorkingDistance = true;
        } else if (arg == "-h" || arg == "--help") {
            printUsage(argv[0]);
            return 0;
        } else {
            fprintf(stderr, "ERROR: 无法识别的参数 '%s'\n", arg.c_str());
            printUsage(argv[0]);
            return 64;
        }
    }

    if (frameCount > 0 || !outDir.empty()) {
        if (frameCount <= 0) {
            fprintf(stderr, "ERROR: --record 需要正整数帧数\n");
            return 64;
        }
        if (outDir.empty()) {
            fprintf(stderr, "ERROR: --record 需要 --out 指定输出目录\n");
            return 64;
        }
        return runRecord(frameCount, warmup, outDir, hasWorkingDistance, workingDistanceMm);
    }

    return runOnce();
}
