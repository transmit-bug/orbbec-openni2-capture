#!/bin/bash
# Automated A/B depth comparison: official driver vs our calibrated driver.
#
# Usage: ./ab_test.sh [scene_name]
#
# Steps:
#   1. Deploy official driver, capture reference frame
#   2. Deploy our driver, capture test frame
#   3. Run pixel-by-pixel comparison
#
# IMPORTANT: Always restores our driver as the active driver at the end,
# even if the script fails partway through.

DRIVER_DIR="/home/bowmanhan/Code/OrbbecSDK/AstraSDK-v2.1.3/lib/Plugins/openni2/OpenNI2/Drivers"
OFFICIAL_DRIVER="$DRIVER_DIR/libAstraDriver.so.orig"
OUR_DRIVER="/home/bowmanhan/Code/OrbbecSDK/openni2-astra-driver/build/oni_driver_astra.so"
DEPLOY_PATH="$DRIVER_DIR/libAstraDriver.so"
BUILD_DIR="/home/bowmanhan/Code/OrbbecSDK/openni2-astra-driver/build"
LIB_PATH="/home/bowmanhan/Code/OrbbecSDK/AstraSDK-v2.1.3/lib/Plugins/openni2"

SCENE_NAME="${1:-default}"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
REF_FILE="/tmp/ab_ref_official_${TIMESTAMP}.bin"
TEST_FILE="/tmp/ab_test_ours_${TIMESTAMP}.bin"

# Always restore our driver on exit (even on error)
restore_driver() {
    echo "Restoring our driver..."
    cp "$OUR_DRIVER" "$DEPLOY_PATH"
    echo "  Our driver restored."
}
trap restore_driver EXIT

echo "=== A/B Depth Comparison: scene=$SCENE_NAME ==="
echo "Timestamp: $TIMESTAMP"
echo ""

# Step 0: Build tools
echo "[0/4] Building dump_compare..."
cd "$BUILD_DIR"
cmake .. -DCMAKE_BUILD_TYPE=Release > /dev/null 2>&1
make -j4 dump_compare 2>&1 | tail -3
echo ""

# Step 1: Deploy official driver & capture reference
echo "[1/4] Deploying official driver..."
cp "$OFFICIAL_DRIVER" "$DEPLOY_PATH"
echo "  Official driver deployed."

echo "  Capturing reference frame (official driver)..."
if ! LD_LIBRARY_PATH="$LIB_PATH" "$BUILD_DIR/examples/dump_compare" "$REF_FILE" 2>&1 | grep -E "(Device:|Frame|Best|Saved)"; then
    echo "  WARNING: Official driver capture failed (this device may not be supported by .orig)"
    echo "  Will still proceed with our driver capture."
fi
echo ""

# Give camera a moment to settle
sleep 1

# Step 2: Deploy our driver & capture test
echo "[2/4] Deploying our driver..."
cp "$OUR_DRIVER" "$DEPLOY_PATH"
echo "  Our driver deployed."

echo "  Capturing test frame (our driver)..."
LD_LIBRARY_PATH="$LIB_PATH" "$BUILD_DIR/examples/dump_compare" "$TEST_FILE" 2>&1 | grep -E "(Device:|Frame|Best|Saved)"
echo ""

# Step 3: Compare (only if reference file exists and has data)
if [ -f "$REF_FILE" ] && [ $(stat -c%s "$REF_FILE") -gt 100 ]; then
    echo "[3/4] Running pixel-by-pixel comparison..."
    LD_LIBRARY_PATH="$LIB_PATH" "$BUILD_DIR/examples/dump_compare" "$REF_FILE" "$TEST_FILE" 2>/dev/null
    echo ""

    RESULT_FILE="/tmp/ab_result_${TIMESTAMP}.txt"
    LD_LIBRARY_PATH="$LIB_PATH" "$BUILD_DIR/examples/dump_compare" "$REF_FILE" "$TEST_FILE" > "$RESULT_FILE" 2>/dev/null
    echo "[4/4] Result saved to $RESULT_FILE"
else
    echo "[3/4] No valid reference frame, skipping comparison."
    echo "[4/4] Our driver frame saved to $TEST_FILE"
fi

echo ""
echo "Files:"
echo "  Reference: $REF_FILE"
echo "  Test:      $TEST_FILE"
echo ""
echo "=== Done (our driver is active) ==="
