#include "SoftFilter.h"
#include <cstdio>
#include <cstring>
#include <cstdlib>

SoftFilter::Config SoftFilter::getConfig(int width)
{
    Config cfg;
    if (width == 640) {           // VGA
        cfg.maxDiff = 4;
        cfg.maxSpeckleSize = 240;
    } else if (width == 1280) {   // SXGA
        cfg.maxDiff = 5;
        cfg.maxSpeckleSize = 4000;
    } else {
        cfg.maxDiff = 5;
        cfg.maxSpeckleSize = 90;
    }
    return cfg;
}

// Flood-fill BFS speckle filter, matching official Softfilter @ 0x98849.
// Uses (col,row) packed as two int16 in one int32 on the stack — no divisions.
// Single allocation: labels(int32*pixels) + stack(int32*pixels) + flags(byte*pixels).
void SoftFilter::apply(uint16_t* depth, int width, int height,
                       uint16_t noDepthValue)
{
    int total = width * height;
    Config cfg = getConfig(width);
    int maxDiff = cfg.maxDiff;
    int maxSize = cfg.maxSpeckleSize;

    // Persistent allocation: labels + stack + flags = total * 9 bytes
    static int lastTotal = 0;
    static int32_t* baseBuf = nullptr;

    if (total != lastTotal) {
        free(baseBuf);
        baseBuf = static_cast<int32_t*>(malloc(static_cast<size_t>(total) * 9 + 1));
        lastTotal = total;
    }

    int32_t* labels = baseBuf;
    int32_t* stack  = labels + total;
    uint8_t* flags  = reinterpret_cast<uint8_t*>(stack + total);

    memset(labels, 0, static_cast<size_t>(total) * 4);
    memset(flags, 0, static_cast<size_t>(total) + 1);

    int labelCount = 0;

    for (int row = 0; row < height; row++) {
        int rowOff = row * width;
        for (int col = 0; col < width; col++) {
            int idx = rowOff + col;
            if (depth[idx] == noDepthValue) continue;

            int lbl = labels[idx];
            if (lbl != 0) {
                if (flags[lbl]) depth[idx] = noDepthValue;
                continue;
            }

            // Start flood fill
            labelCount++;
            labels[idx] = labelCount;
            int sp = 0;
            // Pack (col, row) into one int32 — col in low 16, row in high 16
            stack[sp++] = (row << 16) | (col & 0xFFFF);
            int regionSize = 0;

            while (sp > 0) {
                regionSize++;
                int packed = stack[--sp];
                int c = packed & 0xFFFF;
                int r = (packed >> 16) & 0xFFFF;
                int curIdx = r * width + c;
                int curDepth = depth[curIdx];

                // Right
                if (c < width - 1) {
                    int ni = curIdx + 1;
                    if (!labels[ni] && depth[ni] != noDepthValue &&
                        abs(curDepth - depth[ni]) <= maxDiff) {
                        labels[ni] = labelCount;
                        stack[sp++] = (r << 16) | ((c + 1) & 0xFFFF);
                    }
                }
                // Left
                if (c > 0) {
                    int ni = curIdx - 1;
                    if (!labels[ni] && depth[ni] != noDepthValue &&
                        abs(curDepth - depth[ni]) <= maxDiff) {
                        labels[ni] = labelCount;
                        stack[sp++] = (r << 16) | ((c - 1) & 0xFFFF);
                    }
                }
                // Down
                if (r < height - 1) {
                    int ni = curIdx + width;
                    if (!labels[ni] && depth[ni] != noDepthValue &&
                        abs(curDepth - depth[ni]) <= maxDiff) {
                        labels[ni] = labelCount;
                        stack[sp++] = (((r + 1) << 16) | (c & 0xFFFF));
                    }
                }
                // Up
                if (r > 0) {
                    int ni = curIdx - width;
                    if (!labels[ni] && depth[ni] != noDepthValue &&
                        abs(curDepth - depth[ni]) <= maxDiff) {
                        labels[ni] = labelCount;
                        stack[sp++] = (((r - 1) << 16) | (c & 0xFFFF));
                    }
                }
            }

            // Classify region
            if (regionSize <= maxSize) {
                flags[labelCount] = 1;   // speckle
                depth[idx] = noDepthValue;
            } else {
                flags[labelCount] = 0;   // valid
            }
        }
    }
}
