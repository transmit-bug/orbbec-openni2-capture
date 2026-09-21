#pragma once

#include <cstdint>

// SoftFilter: Flood-fill speckle removal (matching official liborbbec.so).
//
// Algorithm (from Softfilter @ 0x98849 in liborbbec.so.official):
//   For each non-zero, unlabeled pixel, flood-fill via 4-connectivity.
//   Neighbors are added if |depth_diff| <= maxDiff.
//   After flood-fill, if the connected region has <= maxSpeckleSize pixels,
//   it's a speckle — zero all pixels in it.
//
// Parameters by resolution (from disassembly):
//   VGA  (640):  maxDiff=4,   maxSpeckleSize=240
//   SXGA (1280): maxDiff=5,   maxSpeckleSize=4000
//   other:       maxDiff=5,   maxSpeckleSize=90

class SoftFilter {
public:
    struct Config {
        int maxDiff;
        int maxSpeckleSize;
    };

    static Config getConfig(int width);

    // Apply in-place on a depth buffer (uint16, 0 = no depth).
    static void apply(uint16_t* depth, int width, int height,
                      uint16_t noDepthValue = 0);
};
