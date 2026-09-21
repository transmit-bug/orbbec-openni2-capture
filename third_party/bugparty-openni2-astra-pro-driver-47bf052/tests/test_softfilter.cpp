// Unit test for SoftFilter::apply — BFS speckle removal
// Standalone: no hardware or OpenNI2 dependency.

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cassert>

#include "../src/SoftFilter.h"

static int g_pass = 0, g_fail = 0;

#define CHECK(cond, msg) do { \
    if (cond) { g_pass++; } \
    else { g_fail++; fprintf(stderr, "FAIL: %s (line %d)\n", msg, __LINE__); } \
} while(0)

// Count non-zero pixels in a depth buffer.
static int countValid(const uint16_t* buf, int n) {
    int c = 0;
    for (int i = 0; i < n; i++) c += (buf[i] != 0);
    return c;
}

// Count pixels with a specific value.
static int countValue(const uint16_t* buf, int n, uint16_t val) {
    int c = 0;
    for (int i = 0; i < n; i++) c += (buf[i] == val);
    return c;
}

static void test_config() {
    auto c640 = SoftFilter::getConfig(640);
    CHECK(c640.maxDiff == 4, "VGA maxDiff=4");
    CHECK(c640.maxSpeckleSize == 240, "VGA maxSpeckleSize=240");

    auto c1280 = SoftFilter::getConfig(1280);
    CHECK(c1280.maxDiff == 4 + 1, "SXGA maxDiff=5");
    CHECK(c1280.maxSpeckleSize == 4000, "SXGA maxSpeckleSize=4000");

    auto c320 = SoftFilter::getConfig(320);
    CHECK(c320.maxDiff == 5, "other maxDiff=5");
    CHECK(c320.maxSpeckleSize == 90, "other maxSpeckleSize=90");
}

static void test_all_zero() {
    // All-zero buffer — filter should be a no-op.
    uint16_t buf[100] = {};
    SoftFilter::apply(buf, 10, 10, 0);
    CHECK(countValid(buf, 100) == 0, "all-zero stays all-zero");
}

static void test_all_same_depth() {
    // All pixels same depth = one big region > maxSpeckleSize → keep all.
    const int W = 10, H = 10;
    uint16_t buf[W * H];
    for (int i = 0; i < W * H; i++) buf[i] = 500;

    SoftFilter::apply(buf, W, H, 0);
    CHECK(countValid(buf, W * H) == W * H, "uniform depth all kept");
    CHECK(countValue(buf, W * H, 500) == W * H, "uniform depth unchanged");
}

static void test_single_speckle() {
    // One isolated pixel surrounded by zeros → speckle (size 1 <= 240).
    const int W = 10, H = 10;
    uint16_t buf[W * H] = {};
    buf[45] = 800;

    SoftFilter::apply(buf, W, H, 0);
    CHECK(buf[45] == 0, "isolated pixel removed");
    CHECK(countValid(buf, W * H) == 0, "only zeros after filter");
}

static void test_small_cluster_below_threshold() {
    // 3x3 cluster of same depth (9 pixels) in a 10x10 image, rest zero.
    // 9 < 240 → speckle, should be removed.
    const int W = 10, H = 10;
    uint16_t buf[W * H] = {};
    // Place 3x3 cluster at (4,4)-(6,6)
    for (int r = 4; r <= 6; r++)
        for (int c = 4; c <= 6; c++)
            buf[r * W + c] = 600;

    SoftFilter::apply(buf, W, H, 0);
    CHECK(countValid(buf, W * H) == 0, "3x3 cluster (9px) removed");
}

static void test_large_region_kept() {
    // 20x20 region (400 pixels) in a 30x30 image → 400 > 240 → kept.
    const int W = 30, H = 30;
    uint16_t buf[W * H] = {};
    for (int r = 5; r < 25; r++)
        for (int c = 5; c < 25; c++)
            buf[r * W + c] = 500;

    SoftFilter::apply(buf, W, H, 0);
    int kept = countValid(buf, W * H);
    CHECK(kept == 400, "20x20 region (400px) kept");
    CHECK(countValue(buf, W * H, 500) == 400, "values unchanged");
}

static void test_depth_diff_boundary() {
    // Two adjacent pixels: depth=500 and depth=504 (diff=4 <= maxDiff=4).
    // They form one region of size 2 < 240 → speckle, both removed.
    const int W = 10, H = 10;
    uint16_t buf[W * H] = {};
    buf[0] = 500;
    buf[1] = 504;  // right neighbor, diff=4

    SoftFilter::apply(buf, W, H, 0);
    CHECK(buf[0] == 0 && buf[1] == 0, "diff=4 connected, speckle removed");
}

static void test_depth_diff_exceeds_max() {
    // Two adjacent pixels: depth=500 and depth=505 (diff=5 > maxDiff=4 for VGA).
    // Each is a separate region of size 1 → both speckles → both removed.
    const int W = 10, H = 10;
    uint16_t buf[W * H] = {};
    buf[0] = 500;
    buf[1] = 505;  // diff=5 > maxDiff=4, not connected

    SoftFilter::apply(buf, W, H, 0);
    CHECK(buf[0] == 0 && buf[1] == 0, "diff=5 not connected, both speckles");
}

static void test_diagonal_not_connected() {
    // Two pixels diagonal to each other: 4-connectivity means NOT connected.
    // Each is size 1 → both speckles.
    const int W = 10, H = 10;
    uint16_t buf[W * H] = {};
    buf[0] = 500;
    buf[W + 1] = 500;  // diagonal, not 4-connected

    SoftFilter::apply(buf, W, H, 0);
    CHECK(buf[0] == 0, "diagonal pixel A removed");
    CHECK(buf[W + 1] == 0, "diagonal pixel B removed");
}

static void test_no_depth_value_custom() {
    // Using noDepthValue=0xFFFF instead of 0.
    const int W = 10, H = 10;
    uint16_t buf[W * H];
    for (int i = 0; i < W * H; i++) buf[i] = 0xFFFF;
    buf[50] = 300;  // single isolated non-sentinel pixel

    SoftFilter::apply(buf, W, H, 0xFFFF);
    CHECK(buf[50] == 0xFFFF, "isolated pixel with custom sentinel removed");
}

static void test_edge_pixels() {
    // Single pixel at each corner and edge midpoint — all speckles.
    const int W = 10, H = 10;
    uint16_t buf[W * H] = {};
    buf[0] = 400;                          // top-left
    buf[W - 1] = 400;                      // top-right
    buf[(H - 1) * W] = 400;                // bottom-left
    buf[(H - 1) * W + W - 1] = 400;        // bottom-right
    buf[W / 2] = 400;                      // top edge midpoint
    buf[(H / 2) * W] = 400;                // left edge midpoint

    SoftFilter::apply(buf, W, H, 0);
    CHECK(countValid(buf, W * H) == 0, "all edge speckles removed");
}

static void test_two_regions_different_labels() {
    // Two separate 3x3 clusters at different positions, both below threshold.
    const int W = 20, H = 20;
    uint16_t buf[W * H] = {};
    // Cluster A at (2,2)-(4,4)
    for (int r = 2; r <= 4; r++)
        for (int c = 2; c <= 4; c++)
            buf[r * W + c] = 500;
    // Cluster B at (10,10)-(12,12)
    for (int r = 10; r <= 12; r++)
        for (int c = 10; c <= 12; c++)
            buf[r * W + c] = 500;

    SoftFilter::apply(buf, W, H, 0);
    CHECK(countValid(buf, W * H) == 0, "both separate speckle clusters removed");
}

static void test_mixed_speckle_and_valid() {
    // One large region (>240) and one small speckle.
    const int W = 30, H = 30;
    uint16_t buf[W * H] = {};
    // Large region: 16x16 = 256 > 240 → kept
    for (int r = 2; r < 18; r++)
        for (int c = 2; c < 18; c++)
            buf[r * W + c] = 500;
    // Speckle: 3 pixels at (25,25)-(27,25)
    buf[25 * W + 25] = 600;
    buf[26 * W + 25] = 600;
    buf[27 * W + 25] = 600;

    SoftFilter::apply(buf, W, H, 0);
    int kept = countValid(buf, W * H);
    CHECK(kept == 256, "large region kept, speckle removed");
    CHECK(buf[25 * W + 25] == 0, "speckle pixel A removed");
    CHECK(buf[5 * W + 5] == 500, "large region pixel unchanged");
}

static void test_vertical_connectivity() {
    // Two pixels stacked vertically — should be connected via 4-connectivity.
    const int W = 10, H = 10;
    uint16_t buf[W * H] = {};
    buf[5 * W + 3] = 500;
    buf[6 * W + 3] = 500;

    SoftFilter::apply(buf, W, H, 0);
    CHECK(buf[5 * W + 3] == 0 && buf[6 * W + 3] == 0,
          "vertically connected speckle removed");
}

static void test_rectangular_1d_width() {
    // Width=1: vertical strip, all same depth.
    // Height 300 > 240 → valid region.
    const int W = 1, H = 300;
    uint16_t buf[W * H];
    for (int i = 0; i < W * H; i++) buf[i] = 100;

    SoftFilter::apply(buf, W, H, 0);
    CHECK(countValid(buf, W * H) == 300, "1D width vertical strip kept");
}

static void test_idempotent() {
    // Running filter twice on the same data should give the same result.
    const int W = 30, H = 30;
    uint16_t buf1[W * H] = {};
    uint16_t buf2[W * H] = {};
    // One large region + speckle
    for (int r = 2; r < 18; r++)
        for (int c = 2; c < 18; c++)
            buf1[r * W + c] = buf2[r * W + c] = 500;
    buf1[25 * W + 25] = buf2[25 * W + 25] = 600;

    SoftFilter::apply(buf1, W, H, 0);
    SoftFilter::apply(buf2, W, H, 0);

    // buf1 and buf2 should be identical after one pass
    CHECK(memcmp(buf1, buf2, sizeof(buf1)) == 0, "both buffers match after one pass");

    // Apply again to buf1 — should be no-op since speckles already removed
    SoftFilter::apply(buf1, W, H, 0);
    CHECK(memcmp(buf1, buf2, sizeof(buf1)) == 0, "second pass is no-op (idempotent)");
}

int main() {
    fprintf(stderr, "=== SoftFilter unit tests ===\n\n");

    test_config();
    test_all_zero();
    test_all_same_depth();
    test_single_speckle();
    test_small_cluster_below_threshold();
    test_large_region_kept();
    test_depth_diff_boundary();
    test_depth_diff_exceeds_max();
    test_diagonal_not_connected();
    test_no_depth_value_custom();
    test_edge_pixels();
    test_two_regions_different_labels();
    test_mixed_speckle_and_valid();
    test_vertical_connectivity();
    test_rectangular_1d_width();
    test_idempotent();

    fprintf(stderr, "\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail > 0 ? 1 : 0;
}
