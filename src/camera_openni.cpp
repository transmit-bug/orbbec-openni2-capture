#include "camera_openni.h"

CameraOpenNI::~CameraOpenNI() { close(); }

bool CameraOpenNI::open() {
    lastError_.clear();

    openni::Status rc = openni::OpenNI::initialize();
    if (rc != openni::STATUS_OK) {
        lastError_ = "OpenNI initialize failed: ";
        lastError_ += openni::OpenNI::getExtendedError();
        return false;
    }
    niInitialized_ = true;

    rc = device_.open(openni::ANY_DEVICE);
    if (rc != openni::STATUS_OK) {
        lastError_ = "Device open failed: ";
        lastError_ += openni::OpenNI::getExtendedError();
        openni::OpenNI::shutdown();
        return false;
    }

    opened_ = true;

    const openni::DeviceInfo& info = device_.getDeviceInfo();
    printf("[CameraOpenNI] Device: %s (URI: %s)\n",
           info.getName(), info.getUri());
    return true;
}

void CameraOpenNI::close() {
    if (depthRunning_) { depthStream_.stop(); depthStream_.destroy(); depthRunning_ = false; }
    if (colorRunning_) { colorStream_.stop(); colorStream_.destroy(); colorRunning_ = false; }
    if (opened_)       { device_.close(); opened_ = false; }
    if (niInitialized_) {
        // Small delay to let the driver's USB event thread finish pending work
        usleep(50000);
        openni::OpenNI::shutdown();
        niInitialized_ = false;
    }
}

bool CameraOpenNI::startDepth() {
    if (!opened_) { lastError_ = "Device not open"; return false; }
    lastError_.clear();

    printf("[startDepth] Checking sensor info...\n"); fflush(stdout);
    if (device_.getSensorInfo(openni::SENSOR_DEPTH) == nullptr) {
        lastError_ = "No depth sensor found";
        return false;
    }

    printf("[startDepth] Creating stream...\n"); fflush(stdout);
    openni::Status rc = depthStream_.create(device_, openni::SENSOR_DEPTH);
    if (rc != openni::STATUS_OK) {
        lastError_ = "Depth stream create failed: ";
        lastError_ += openni::OpenNI::getExtendedError();
        return false;
    }

    // Set preferred video mode: 640x480 @ 30fps, 1mm
    const openni::SensorInfo* sensorInfo = device_.getSensorInfo(openni::SENSOR_DEPTH);
    const openni::Array<openni::VideoMode>& modes = sensorInfo->getSupportedVideoModes();
    for (int i = 0; i < modes.getSize(); ++i) {
        const openni::VideoMode& m = modes[i];
        if (m.getResolutionX() == 640 && m.getResolutionY() == 480 &&
            m.getPixelFormat() == openni::PIXEL_FORMAT_DEPTH_1_MM &&
            m.getFps() == 30) {
            depthStream_.setVideoMode(m);
            break;
        }
    }

    printf("[startDepth] Stream created, calling start()...\n"); fflush(stdout);
    rc = depthStream_.start();
    if (rc != openni::STATUS_OK) {
        lastError_ = "Depth stream start failed: ";
        lastError_ += openni::OpenNI::getExtendedError();
        depthStream_.destroy();
        return false;
    }

    depthRunning_ = true;
    printf("[startDepth] Stream started OK\n"); fflush(stdout);
    const openni::VideoMode vm = depthStream_.getVideoMode();
    printf("[CameraOpenNI] Depth: %dx%d @ %dfps, format=%d\n",
           vm.getResolutionX(), vm.getResolutionY(), vm.getFps(), vm.getPixelFormat());
    return true;
}

bool CameraOpenNI::startColor() {
    if (!opened_) { lastError_ = "Device not open"; return false; }
    lastError_.clear();

    if (device_.getSensorInfo(openni::SENSOR_COLOR) == nullptr) {
        lastError_ = "No color sensor found";
        return false;
    }

    openni::Status rc = colorStream_.create(device_, openni::SENSOR_COLOR);
    if (rc != openni::STATUS_OK) {
        lastError_ = "Color stream create failed: ";
        lastError_ += openni::OpenNI::getExtendedError();
        return false;
    }

    // Set preferred video mode: 640x480 @ 30fps, RGB888
    const openni::SensorInfo* sensorInfo = device_.getSensorInfo(openni::SENSOR_COLOR);
    const openni::Array<openni::VideoMode>& modes = sensorInfo->getSupportedVideoModes();
    for (int i = 0; i < modes.getSize(); ++i) {
        const openni::VideoMode& m = modes[i];
        if (m.getResolutionX() == 640 && m.getResolutionY() == 480 &&
            m.getPixelFormat() == openni::PIXEL_FORMAT_RGB888 &&
            m.getFps() == 30) {
            colorStream_.setVideoMode(m);
            break;
        }
    }

    rc = colorStream_.start();
    if (rc != openni::STATUS_OK) {
        lastError_ = "Color stream start failed: ";
        lastError_ += openni::OpenNI::getExtendedError();
        colorStream_.destroy();
        return false;
    }

    colorRunning_ = true;
    const openni::VideoMode vm = colorStream_.getVideoMode();
    printf("[CameraOpenNI] Color: %dx%d @ %dfps, format=%d\n",
           vm.getResolutionX(), vm.getResolutionY(), vm.getFps(), vm.getPixelFormat());
    return true;
}

bool CameraOpenNI::enableRegistration() {
    if (!opened_) { lastError_ = "Device not open"; return false; }
    openni::Status rc = device_.setImageRegistrationMode(
        openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
    if (rc != openni::STATUS_OK) {
        lastError_ = "Registration enable failed: ";
        lastError_ += openni::OpenNI::getExtendedError();
        return false;
    }
    printf("[CameraOpenNI] Depth-to-color registration enabled\n");
    return true;
}

bool CameraOpenNI::readDepth(std::vector<uint16_t>& depthData,
                              int& width, int& height,
                              int timeoutMs) {
    if (!depthRunning_) { lastError_ = "Depth stream not running"; return false; }

    openni::VideoStream* streams[] = { &depthStream_ };
    int changedIndex = -1;
    openni::Status rc = openni::OpenNI::waitForAnyStream(streams, 1, &changedIndex, timeoutMs);
    if (rc != openni::STATUS_OK) {
        lastError_ = "Depth wait timeout";
        return false;
    }

    openni::VideoFrameRef frame;
    rc = depthStream_.readFrame(&frame);
    if (rc != openni::STATUS_OK) {
        lastError_ = "Depth readFrame failed: ";
        lastError_ += openni::OpenNI::getExtendedError();
        return false;
    }

    width  = frame.getWidth();
    height = frame.getHeight();
    depthData.resize(width * height);
    memcpy(depthData.data(), frame.getData(), width * height * sizeof(uint16_t));
    return true;
}

bool CameraOpenNI::readColor(std::vector<uint8_t>& colorData,
                              int& width, int& height,
                              int timeoutMs) {
    if (!colorRunning_) { lastError_ = "Color stream not running"; return false; }

    openni::VideoStream* streams[] = { &colorStream_ };
    int changedIndex = -1;
    openni::Status rc = openni::OpenNI::waitForAnyStream(streams, 1, &changedIndex, timeoutMs);
    if (rc != openni::STATUS_OK) {
        lastError_ = "Color wait timeout";
        return false;
    }

    openni::VideoFrameRef frame;
    rc = colorStream_.readFrame(&frame);
    if (rc != openni::STATUS_OK) {
        lastError_ = "Color readFrame failed: ";
        lastError_ += openni::OpenNI::getExtendedError();
        return false;
    }

    width  = frame.getWidth();
    height = frame.getHeight();
    colorData.resize(width * height * 3);
    memcpy(colorData.data(), frame.getData(), width * height * 3);
    return true;
}

CameraOpenNI::Intrinsics CameraOpenNI::depthIntrinsics() {
    if (!depthRunning_) return {0, 0, 0, 0};
    const openni::CameraSettings* settings = depthStream_.getCameraSettings();
    if (!settings) return {0, 0, 0, 0};
    Intrinsics intr;
    intr.fx = settings->getAutoExposureEnabled() ? 570.3 : 570.3; // placeholder
    intr.fy = 570.3;
    intr.cx = 320.0;
    intr.cy = 240.0;
    // Try to get real values
    openni::VideoMode vm = depthStream_.getVideoMode();
    // OpenNI doesn't expose fx/fy directly in all versions;
    // for Astra Pro at 640x480, typical values are approximately:
    intr.fx = 570.3; intr.fy = 570.3;
    intr.cx = vm.getResolutionX() / 2.0;
    intr.cy = vm.getResolutionY() / 2.0;
    return intr;
}
