/**
 * Astra Pro RGB-D Viewer - GTK3 Version
 * Uses GTK3 for better Linux/Wayland performance
 */

#include <gtk/gtk.h>
#include <opencv2/opencv.hpp>
#include <OpenNI.h>
#include <libobsensor/ObSensor.hpp>
#include <stdio.h>
#include <signal.h>
#include <atomic>
#include <mutex>
#include <thread>
#include <chrono>

static std::atomic<bool> running{true};
void signal_handler(int) { running = false; }

// Shared frames
std::mutex colorMutex, depthMutex;
cv::Mat latestColor, latestDepth;
bool newColorReady = false, newDepthReady = false;

// GTK widgets
GtkWidget *window, *colorImage, *depthImage, *statusLabel;

// OpenNI2 depth thread
void depthThread() {
    OniStatus rc = oniInitialize(ONI_API_VERSION);
    if(rc != ONI_STATUS_OK) {
        fprintf(stderr, "OpenNI2 initialize failed: %d\n", rc);
        return;
    }

    OniDeviceInfo* deviceInfos;
    int deviceCount;
    rc = oniGetDeviceList(&deviceInfos, &deviceCount);
    if(rc != ONI_STATUS_OK || deviceCount == 0) {
        fprintf(stderr, "No OpenNI2 devices found\n");
        oniShutdown();
        return;
    }

    OniDeviceHandle device;
    rc = oniDeviceOpen(deviceInfos[0].uri, &device);
    if(rc != ONI_STATUS_OK) {
        fprintf(stderr, "OpenNI2 device open failed\n");
        oniShutdown();
        return;
    }
    printf("OpenNI2: Device opened - %s\n", deviceInfos[0].name);

    OniStreamHandle depthStream;
    rc = oniDeviceCreateStream(device, ONI_SENSOR_DEPTH, &depthStream);
    if(rc != ONI_STATUS_OK) {
        fprintf(stderr, "OpenNI2: Create depth stream failed: %d\n", rc);
        oniDeviceClose(device);
        oniShutdown();
        return;
    }
    printf("OpenNI2: Depth stream created\n");

    rc = oniStreamStart(depthStream);
    if(rc != ONI_STATUS_OK) {
        fprintf(stderr, "OpenNI2: Start depth stream failed: %d\n", rc);
        oniStreamDestroy(depthStream);
        oniDeviceClose(device);
        oniShutdown();
        return;
    }

    printf("OpenNI2: Depth streaming started\n");
    int frameCount = 0;
    int fpsFrameCount = 0;
    float currentFps = 0.0f;
    auto lastTime = std::chrono::steady_clock::now();

    // Create color bar for depth visualization (0m=blue at top, 5m=red at bottom)
    cv::Mat colorBar(200, 30, CV_8UC3);
    for(int y = 0; y < 200; y++) {
        for(int x = 0; x < 30; x++) {
            // Invert y so blue (near) is at top, red (far) at bottom
            int yInv = 199 - y;
            cv::Vec3b color;
            if(yInv < 25) color = cv::Vec3b(255, 0, 0);      // Blue
            else if(yInv < 50) color = cv::Vec3b(255 - (yInv-25)*4, 0, (yInv-25)*4);
            else if(yInv < 75) color = cv::Vec3b(0, (yInv-50)*4, 255 - (yInv-50)*4);
            else if(yInv < 100) color = cv::Vec3b(0, 255, 0); // Green
            else if(yInv < 125) color = cv::Vec3b((yInv-100)*4, 255 - (yInv-100)*4, 0);
            else if(yInv < 150) color = cv::Vec3b(255, 255 - (yInv-125)*4, 0);
            else if(yInv < 175) color = cv::Vec3b(255, (yInv-150)*4, (yInv-150)*4);
            else color = cv::Vec3b(255, 0, 255);             // Red
            colorBar.at<cv::Vec3b>(y, x) = color;
        }
    }

    while(running) {
        OniFrame* frame;
        rc = oniStreamReadFrame(depthStream, &frame);
        if(rc == ONI_STATUS_OK && frame && frame->dataSize > 0) {
            frameCount++;
            fpsFrameCount++;

            uint16_t* depthData = (uint16_t*)frame->data;
            cv::Mat depthRaw(frame->height, frame->width, CV_16UC1, depthData);

            double minVal, maxVal;
            cv::minMaxLoc(depthRaw, &minVal, &maxVal, nullptr, nullptr, depthRaw > 0);

            cv::Mat depthVisual;
            if(maxVal > 0) {
                depthRaw.convertTo(depthVisual, CV_8U, 255.0 / maxVal);
                cv::applyColorMap(depthVisual, depthVisual, cv::COLORMAP_JET);
            } else {
                depthVisual = cv::Mat::zeros(frame->height, frame->width, CV_8UC3);
            }

            // Add FPS
            auto now = std::chrono::steady_clock::now();
            float elapsed = std::chrono::duration<float>(now - lastTime).count();
            if(elapsed >= 1.0f) {
                currentFps = fpsFrameCount / elapsed;
                fpsFrameCount = 0;
                lastTime = now;
            }
            char text[64];
            snprintf(text, sizeof(text), "FPS: %.1f", currentFps);
            cv::putText(depthVisual, text, cv::Point(10, 25),
                       cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);

            // Add max depth
            snprintf(text, sizeof(text), "Max: %.1fm", maxVal / 1000.0f);
            cv::putText(depthVisual, text, cv::Point(10, 50),
                       cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);

            // Add color bar on the right side
            // Use fixed depth range 0-5m for consistent scale
            const float MAX_DEPTH_SCALE = 5.0f;  // Fixed max depth for scale
            int barX = depthVisual.cols - 40;
            int barY = 80;
            int barHeight = depthVisual.rows - barY - 10;
            cv::Mat resizedBar;
            cv::resize(colorBar, resizedBar, cv::Size(30, barHeight));
            resizedBar.copyTo(depthVisual(cv::Rect(barX, barY, 30, barHeight)));

            // Add depth tick marks with fixed labels (0.5m, 1m, 1.5m, 2m, 3m, 4m)
            float tickDepths[] = {0.5, 1.0, 1.5, 2.0, 3.0, 4.0};
            int numTicks = 6;

            for(int i = 0; i < numTicks; i++) {
                float tickDepth = tickDepths[i];
                // Calculate y position (0m at top, MAX_DEPTH_SCALE at bottom)
                float ratio = tickDepth / MAX_DEPTH_SCALE;
                int tickY = barY + barHeight - (int)(barHeight * ratio);

                // Draw tick line
                cv::line(depthVisual, cv::Point(barX - 5, tickY), cv::Point(barX, tickY),
                        cv::Scalar(255, 255, 255), 1);

                // Draw label
                snprintf(text, sizeof(text), "%.1f", tickDepth);
                cv::putText(depthVisual, text, cv::Point(barX - 35, tickY + 5),
                           cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(255, 255, 255), 1);
            }

            // Add "0" at top of scale
            cv::putText(depthVisual, "0", cv::Point(barX - 18, barY + 18),
                       cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(255, 255, 255), 1);

            // Add "5" at bottom of scale
            cv::putText(depthVisual, "5", cv::Point(barX - 18, depthVisual.rows - 10),
                       cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(255, 255, 255), 1);

            // Add "m" unit label centered
            cv::putText(depthVisual, "m", cv::Point(barX - 8, (barY + depthVisual.rows) / 2),
                       cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(255, 255, 255), 1);

            // Add current max depth in top-left corner
            snprintf(text, sizeof(text), "Max: %.2fm", maxVal / 1000.0f);
            cv::putText(depthVisual, text, cv::Point(10, 50),
                       cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);

            {
                std::lock_guard<std::mutex> lock(depthMutex);
                latestDepth = depthVisual.clone();
                newDepthReady = true;
            }
        }
        oniFrameRelease(frame);
    }

    oniStreamStop(depthStream);
    oniStreamDestroy(depthStream);
    oniDeviceClose(device);
    oniShutdown();
    printf("OpenNI2: Depth thread stopped\n");
}

// OrbbecSDK v2 color thread
void colorThread() {
    try {
        ob::Context context;
        auto deviceList = context.queryDeviceList();

        std::shared_ptr<ob::Device> astraPro;
        for(uint32_t i = 0; i < deviceList->getCount(); i++) {
            try {
                auto dev = deviceList->getDevice(i);
                auto info = dev->getDeviceInfo();
                if(info->getPid() == 0x0403) {
                    astraPro = dev;
                    break;
                }
            } catch(...) {}
        }

        if(!astraPro) {
            fprintf(stderr, "OrbbecSDK: Astra Pro not found\n");
            return;
        }

        auto sensorList = astraPro->getSensorList();
        auto sensor = sensorList->getSensor(0);
        auto profiles = sensor->getStreamProfileList();

        std::shared_ptr<ob::VideoStreamProfile> selected;
        for(uint32_t i = 0; i < profiles->getCount(); i++) {
            auto p = profiles->getProfile(i)->as<ob::VideoStreamProfile>();
            if(p->getWidth() == 640 && p->getHeight() == 480 && p->getFormat() == OB_FORMAT_YUYV) {
                selected = p;
                break;
            }
        }
        if(!selected) {
            fprintf(stderr, "OrbbecSDK: 640x480 YUYV not found\n");
            return;
        }

        printf("OrbbecSDK: Color streaming - %dx%d @ %dfps\n",
               selected->getWidth(), selected->getHeight(), selected->getFps());

        int frameCount = 0;
        int fpsFrameCount = 0;
        float currentFps = 0.0f;
        auto lastTime = std::chrono::steady_clock::now();

        sensor->start(selected, [&](std::shared_ptr<ob::Frame> frame) {
            if(!running) return;

            frameCount++;
            fpsFrameCount++;
            auto vf = frame->as<ob::VideoFrame>();

            cv::Mat yuyv(vf->getHeight(), vf->getWidth(), CV_8UC2, vf->getData());
            cv::Mat bgr;
            cv::cvtColor(yuyv, bgr, cv::COLOR_YUV2BGR_YUYV);

            auto now = std::chrono::steady_clock::now();
            float elapsed = std::chrono::duration<float>(now - lastTime).count();
            if(elapsed >= 1.0f) {
                currentFps = fpsFrameCount / elapsed;
                fpsFrameCount = 0;
                lastTime = now;
            }
            char text[64];
            snprintf(text, sizeof(text), "Color: %.1f FPS", currentFps);
            cv::putText(bgr, text, cv::Point(10, 30),
                       cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

            {
                std::lock_guard<std::mutex> lock(colorMutex);
                latestColor = bgr.clone();
                newColorReady = true;
            }
        });

        while(running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        sensor->stop();
        printf("OrbbecSDK: Color thread stopped\n");

    } catch(ob::Error &e) {
        fprintf(stderr, "OrbbecSDK Error: %s\n", e.getMessage());
    } catch(std::exception &e) {
        fprintf(stderr, "OrbbecSDK Exception: %s\n", e.what());
    }
}

// GTK timeout callback to update images
gboolean updateImages(gpointer data) {
    if(!running) return G_SOURCE_REMOVE;

    cv::Mat color, depth;
    bool hasColor = false, hasDepth = false;

    {
        std::lock_guard<std::mutex> lock(colorMutex);
        if(newColorReady && !latestColor.empty()) {
            color = latestColor.clone();
            hasColor = true;
            newColorReady = false;
        }
    }

    {
        std::lock_guard<std::mutex> lock(depthMutex);
        if(newDepthReady && !latestDepth.empty()) {
            depth = latestDepth.clone();
            hasDepth = true;
            newDepthReady = false;
        }
    }

    if(hasColor && colorImage) {
        cv::Mat rgb;
        cv::cvtColor(color, rgb, cv::COLOR_BGR2RGB);
        // Make a copy to ensure data stays valid
        void* dataCopy = malloc(rgb.total() * rgb.elemSize());
        memcpy(dataCopy, rgb.data, rgb.total() * rgb.elemSize());
        GdkPixbuf* pixbuf = gdk_pixbuf_new_from_data(
            (const guchar*)dataCopy, GDK_COLORSPACE_RGB, FALSE, 8, rgb.cols, rgb.rows, rgb.step,
            (GdkPixbufDestroyNotify)g_free, dataCopy);
        gtk_image_set_from_pixbuf(GTK_IMAGE(colorImage), pixbuf);
        g_object_unref(pixbuf);
    }

    if(hasDepth && depthImage) {
        cv::Mat rgb;
        cv::cvtColor(depth, rgb, cv::COLOR_BGR2RGB);
        // Make a copy to ensure data stays valid
        void* dataCopy = malloc(rgb.total() * rgb.elemSize());
        memcpy(dataCopy, rgb.data, rgb.total() * rgb.elemSize());
        GdkPixbuf* pixbuf = gdk_pixbuf_new_from_data(
            (const guchar*)dataCopy, GDK_COLORSPACE_RGB, FALSE, 8, rgb.cols, rgb.rows, rgb.step,
            (GdkPixbufDestroyNotify)g_free, dataCopy);
        gtk_image_set_from_pixbuf(GTK_IMAGE(depthImage), pixbuf);
        g_object_unref(pixbuf);
    }

    return G_SOURCE_CONTINUE;
}

int main(int argc, char** argv) {
    signal(SIGINT, signal_handler);

    // Set environment for OpenNI2
    setenv("OPENNI2_REPO", "/tmp/OpenNI_2.3.0.86_202210111154_4c8f5aa4_beta6_linux/sdk/libs/OpenNI2/Drivers", 1);

    // Start capture threads
    std::thread depthT(depthThread);
    std::thread colorT(colorThread);

    // Init GTK
    gtk_init(&argc, &argv);

    window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_window_set_title(GTK_WINDOW(window), "Astra Pro RGB-D Viewer");
    gtk_window_set_default_size(GTK_WINDOW(window), 1280, 480);
    g_signal_connect(window, "destroy", G_CALLBACK(gtk_main_quit), NULL);

    GtkWidget *vbox = gtk_box_new(GTK_ORIENTATION_VERTICAL, 0);
    gtk_container_add(GTK_CONTAINER(window), vbox);

    GtkWidget *hbox = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 0);
    gtk_box_pack_start(GTK_BOX(vbox), hbox, TRUE, TRUE, 0);

    colorImage = gtk_image_new();
    depthImage = gtk_image_new();
    gtk_box_pack_start(GTK_BOX(hbox), colorImage, TRUE, TRUE, 0);
    gtk_box_pack_start(GTK_BOX(hbox), depthImage, TRUE, TRUE, 0);

    statusLabel = gtk_label_new("Press ESC or close window to exit");
    gtk_box_pack_start(GTK_BOX(vbox), statusLabel, FALSE, FALSE, 5);

    gtk_widget_show_all(window);

    printf("GTK viewer started. Close window to exit.\n");

    // Setup timeout for image updates (30 FPS)
    g_timeout_add(33, updateImages, NULL);

    gtk_main();

    running = false;
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    if(colorT.joinable()) colorT.join();
    if(depthT.joinable()) depthT.join();

    printf("Done!\n");
    return 0;
}
