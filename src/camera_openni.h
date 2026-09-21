#pragma once

#include <OpenNI.h>
#include <cstdint>
#include <string>
#include <vector>

/// RAII wrapper around OpenNI2 Device for Astra Pro.
/// Provides depth + color streams and simple frame access.
class CameraOpenNI {
public:
    CameraOpenNI() = default;
    ~CameraOpenNI();

    // Non-copyable
    CameraOpenNI(const CameraOpenNI&) = delete;
    CameraOpenNI& operator=(const CameraOpenNI&) = delete;

    /// Initialize OpenNI and open the first available Orbbec device.
    /// Returns false on failure (use lastError() for details).
    bool open();

    /// Stop streams and shutdown.
    void close();

    /// Create and start depth stream (640x480 @ 30fps, 1mm units).
    bool startDepth();

    /// Create and start color stream (640x480 @ 30fps, RGB888).
    bool startColor();

    /// Enable depth-to-color registration (call after both streams created).
    bool enableRegistration();

    /// Blocking read of the latest depth frame.
    /// Returns false on timeout or error.
    /// Output: width, height, and raw depth data (uint16, millimeters).
    bool readDepth(std::vector<uint16_t>& depthData,
                   int& width, int& height,
                   int timeoutMs = 2000);

    /// Blocking read of the latest color frame.
    /// Returns false on timeout or error.
    /// Output: width, height, and raw RGB data (uint8, 3 channels).
    bool readColor(std::vector<uint8_t>& colorData,
                   int& width, int& height,
                   int timeoutMs = 2000);

    /// Depth intrinsics for point cloud generation.
    struct Intrinsics {
        double fx, fy, cx, cy;
    };
    Intrinsics depthIntrinsics();

    bool isDepthRunning() const { return depthRunning_; }
    bool isColorRunning() const { return colorRunning_; }
    const std::string& lastError() const { return lastError_; }

private:
    openni::Device    device_;
    openni::VideoStream depthStream_;
    openni::VideoStream colorStream_;
    bool opened_       = false;
    bool depthRunning_ = false;
    bool colorRunning_ = false;
    bool niInitialized_ = false;
    std::string lastError_;
};
