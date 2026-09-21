/**
 * Astra Pro Depth + IR Viewer - GTK3 + OpenNI2 C API
 * Single device handle, single frame-reading thread, GTK main loop for display.
 */

#include <gtk/gtk.h>
#include <opencv2/opencv.hpp>
#include <OpenNI.h>
#include <OniCProperties.h>
#include <stdio.h>
#include <signal.h>
#include <atomic>
#include <mutex>
#include <thread>
#include <chrono>

static std::atomic<bool> running{true};
void signal_handler(int) { running = false; }

std::mutex depthMutex, irMutex;
cv::Mat latestDepth, latestDepthRaw, latestIR, latestIRRaw;
bool newDepthReady = false, newIRReady = false;

GtkWidget *window, *depthImage, *irImage, *statusLabel;

cv::Mat depthColorBar;
const float MAX_DEPTH_SCALE = 5.0f;
const int MAX_DEPTH_MM = (int)(MAX_DEPTH_SCALE * 1000);

void initDepthColorBar() {
    depthColorBar = cv::Mat(256, 30, CV_8UC3);
    cv::Mat ramp(1, 256, CV_8U);
    for (int i = 0; i < 256; i++) ramp.at<uchar>(0, i) = i;
    cv::Mat rampColor;
    cv::applyColorMap(ramp, rampColor, cv::COLORMAP_JET);
    for (int y = 0; y < 256; y++)
        for (int x = 0; x < 30; x++)
            depthColorBar.at<cv::Vec3b>(y, x) = rampColor.at<cv::Vec3b>(0, y);
}

void streamThread() {
    setenv("OPENNI2_REPO",
           "/home/bowmanhan/Code/OrbbecSDK/AstraSDK-v2.1.3/lib/Plugins/openni2/Drivers", 1);

    OniStatus rc = oniInitialize(ONI_API_VERSION);
    if (rc != ONI_STATUS_OK) {
        fprintf(stderr, "OpenNI2 init failed: %d\n", rc);
        return;
    }

    OniDeviceInfo* deviceInfos;
    int deviceCount;
    rc = oniGetDeviceList(&deviceInfos, &deviceCount);
    if (rc != ONI_STATUS_OK || deviceCount == 0) {
        fprintf(stderr, "No devices found\n");
        oniShutdown();
        return;
    }
    printf("Found %d device(s)\n", deviceCount);

    OniDeviceHandle device;
    rc = oniDeviceOpen(deviceInfos[0].uri, &device);
    if (rc != ONI_STATUS_OK) {
        fprintf(stderr, "Device open failed: %d\n", rc);
        oniShutdown();
        return;
    }
    printf("Device opened\n");

    // Depth stream
    OniStreamHandle depthStream = NULL;
    rc = oniDeviceCreateStream(device, ONI_SENSOR_DEPTH, &depthStream);
    if (rc != ONI_STATUS_OK) {
        fprintf(stderr, "Create depth stream failed: %d\n", rc);
    } else {
        OniVideoMode mode = {ONI_PIXEL_FORMAT_DEPTH_1_MM, 640, 480, 30};
        oniStreamSetProperty(depthStream, ONI_STREAM_PROPERTY_VIDEO_MODE, &mode, sizeof(mode));
        rc = oniStreamStart(depthStream);
        if (rc != ONI_STATUS_OK) {
            fprintf(stderr, "Start depth stream failed: %d\n", rc);
            oniStreamDestroy(depthStream);
            depthStream = NULL;
        } else {
            printf("Depth streaming started\n");
        }
    }

    // IR stream
    OniStreamHandle irStream = NULL;
    rc = oniDeviceCreateStream(device, ONI_SENSOR_IR, &irStream);
    if (rc != ONI_STATUS_OK) {
        fprintf(stderr, "Create IR stream failed: %d\n", rc);
    } else {
        OniVideoMode mode = {ONI_PIXEL_FORMAT_GRAY16, 640, 480, 30};
        oniStreamSetProperty(irStream, ONI_STREAM_PROPERTY_VIDEO_MODE, &mode, sizeof(mode));
        rc = oniStreamStart(irStream);
        if (rc != ONI_STATUS_OK) {
            fprintf(stderr, "Start IR stream failed: %d\n", rc);
            oniStreamDestroy(irStream);
            irStream = NULL;
        } else {
            printf("IR streaming started\n");
        }
    }

    int depthFrames = 0, irFrames = 0;
    float depthFps = 0, irFps = 0;
    int depthFpsCount = 0, irFpsCount = 0;
    auto lastTime = std::chrono::steady_clock::now();

    while (running) {
        bool didWork = false;

        if (depthStream) {
            OniFrame* frame;
            rc = oniStreamReadFrame(depthStream, &frame);
            if (rc == ONI_STATUS_OK && frame && frame->dataSize > 0) {
                depthFrames++;
                depthFpsCount++;
                didWork = true;

                uint16_t* data = (uint16_t*)frame->data;
                cv::Mat raw(frame->height, frame->width, CV_16UC1, data);

                double minVal, maxVal;
                cv::minMaxLoc(raw, &minVal, &maxVal, nullptr, nullptr, raw > 0);

                cv::Mat vis;
                if (maxVal > 0) {
                    raw.convertTo(vis, CV_8U, 255.0 / MAX_DEPTH_MM, 0);
                    cv::applyColorMap(vis, vis, cv::COLORMAP_JET);
                    cv::Mat overRange;
                    cv::threshold(raw, overRange, MAX_DEPTH_MM, 255, cv::THRESH_BINARY);
                    overRange.convertTo(overRange, CV_8U);
                    vis.setTo(cv::Vec3b(40, 40, 40), overRange > 0);
                } else {
                    vis = cv::Mat::zeros(frame->height, frame->width, CV_8UC3);
                }

                char text[64];
                snprintf(text, sizeof(text), "Depth: %.1f FPS", depthFps);
                cv::putText(vis, text, cv::Point(10, 25),
                           cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
                snprintf(text, sizeof(text), "Max: %.2fm", maxVal / 1000.0);
                cv::putText(vis, text, cv::Point(10, 50),
                           cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

                // Color bar
                int barX = vis.cols - 40, barY = 80;
                int barH = vis.rows - barY - 10;
                cv::Mat resizedBar;
                cv::resize(depthColorBar, resizedBar, cv::Size(30, barH));
                resizedBar.copyTo(vis(cv::Rect(barX, barY, 30, barH)));

                float ticks[] = {0.5, 1.0, 1.5, 2.0, 3.0, 4.0};
                for (int i = 0; i < 6; i++) {
                    int ty = barY + (int)(barH * ticks[i] / MAX_DEPTH_SCALE);
                    cv::line(vis, cv::Point(barX - 5, ty), cv::Point(barX, ty),
                            cv::Scalar(255, 255, 255), 1);
                    snprintf(text, sizeof(text), "%.1f", ticks[i]);
                    cv::putText(vis, text, cv::Point(barX - 35, ty + 5),
                               cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(255, 255, 255), 1);
                }

                {
                    std::lock_guard<std::mutex> lk(depthMutex);
                    latestDepth = vis.clone();
                    latestDepthRaw = raw.clone();
                    newDepthReady = true;
                }
                oniFrameRelease(frame);
            }
        }

        if (irStream) {
            OniFrame* frame;
            rc = oniStreamReadFrame(irStream, &frame);
            if (rc == ONI_STATUS_OK && frame && frame->dataSize > 0) {
                irFrames++;
                irFpsCount++;
                didWork = true;

                uint16_t* data = (uint16_t*)frame->data;
                cv::Mat raw(frame->height, frame->width, CV_16UC1, data);

                double minVal, maxVal;
                cv::minMaxLoc(raw, &minVal, &maxVal, nullptr, nullptr, raw > 0);

                cv::Mat vis;
                if (maxVal > 0) {
                    double normMax = std::min(maxVal, 4000.0);
                    raw.convertTo(vis, CV_8U, 255.0 / normMax, 0);
                    cv::applyColorMap(vis, vis, cv::COLORMAP_HOT);
                    cv::Mat noData;
                    cv::threshold(raw, noData, 0, 255, cv::THRESH_BINARY_INV);
                    noData.convertTo(noData, CV_8U);
                    vis.setTo(cv::Vec3b(20, 20, 20), noData > 0);
                } else {
                    vis = cv::Mat::zeros(frame->height, frame->width, CV_8UC3);
                }

                char text[64];
                snprintf(text, sizeof(text), "IR: %.1f FPS", irFps);
                cv::putText(vis, text, cv::Point(10, 25),
                           cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
                snprintf(text, sizeof(text), "Max: %d", (int)maxVal);
                cv::putText(vis, text, cv::Point(10, 50),
                           cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

                {
                    std::lock_guard<std::mutex> lk(irMutex);
                    latestIR = vis.clone();
                    latestIRRaw = raw.clone();
                    newIRReady = true;
                }
                oniFrameRelease(frame);
            }
        }

        auto now = std::chrono::steady_clock::now();
        float elapsed = std::chrono::duration<float>(now - lastTime).count();
        if (elapsed >= 1.0f) {
            depthFps = depthFpsCount / elapsed;
            irFps = irFpsCount / elapsed;
            depthFpsCount = 0;
            irFpsCount = 0;
            lastTime = now;
            printf("Depth: %d frames (%.1f fps), IR: %d frames (%.1f fps)\n",
                   depthFrames, depthFps, irFrames, irFps);
        }
    }

    if (depthStream) { oniStreamStop(depthStream); oniStreamDestroy(depthStream); }
    if (irStream) { oniStreamStop(irStream); oniStreamDestroy(irStream); }
    oniDeviceClose(device);
    oniShutdown();
    printf("Stream thread stopped\n");
}

gboolean updateImages(gpointer) {
    if (!running) return G_SOURCE_REMOVE;

    cv::Mat depth, ir;
    bool hasDepth = false, hasIR = false;

    {
        std::lock_guard<std::mutex> lk(depthMutex);
        if (newDepthReady && !latestDepth.empty()) {
            depth = latestDepth.clone();
            hasDepth = true;
            newDepthReady = false;
        }
    }
    {
        std::lock_guard<std::mutex> lk(irMutex);
        if (newIRReady && !latestIR.empty()) {
            ir = latestIR.clone();
            hasIR = true;
            newIRReady = false;
        }
    }

    if (hasDepth && depthImage) {
        cv::Mat rgb;
        cv::cvtColor(depth, rgb, cv::COLOR_BGR2RGB);
        void* dataCopy = malloc(rgb.total() * rgb.elemSize());
        memcpy(dataCopy, rgb.data, rgb.total() * rgb.elemSize());
        GdkPixbuf* pixbuf = gdk_pixbuf_new_from_data(
            (const guchar*)dataCopy, GDK_COLORSPACE_RGB, FALSE, 8,
            rgb.cols, rgb.rows, rgb.step,
            (GdkPixbufDestroyNotify)g_free, dataCopy);
        gtk_image_set_from_pixbuf(GTK_IMAGE(depthImage), pixbuf);
        g_object_unref(pixbuf);
    }

    if (hasIR && irImage) {
        cv::Mat rgb;
        cv::cvtColor(ir, rgb, cv::COLOR_BGR2RGB);
        void* dataCopy = malloc(rgb.total() * rgb.elemSize());
        memcpy(dataCopy, rgb.data, rgb.total() * rgb.elemSize());
        GdkPixbuf* pixbuf = gdk_pixbuf_new_from_data(
            (const guchar*)dataCopy, GDK_COLORSPACE_RGB, FALSE, 8,
            rgb.cols, rgb.rows, rgb.step,
            (GdkPixbufDestroyNotify)g_free, dataCopy);
        gtk_image_set_from_pixbuf(GTK_IMAGE(irImage), pixbuf);
        g_object_unref(pixbuf);
    }

    return G_SOURCE_CONTINUE;
}

gboolean onDepthMotion(GtkWidget*, GdkEventMotion* event, gpointer) {
    std::lock_guard<std::mutex> lk(depthMutex);
    if (latestDepthRaw.empty()) return FALSE;
    int imgX = (int)event->x * latestDepthRaw.cols / 640;
    int imgY = (int)event->y * latestDepthRaw.rows / 480;
    if (imgX < 0 || imgX >= latestDepthRaw.cols || imgY < 0 || imgY >= latestDepthRaw.rows)
        return FALSE;
    uint16_t d = latestDepthRaw.at<uint16_t>(imgY, imgX);
    char buf[128];
    if (d > 0)
        snprintf(buf, sizeof(buf), "Depth (%d,%d): %dmm (%.3fm)", imgX, imgY, d, d / 1000.0f);
    else
        snprintf(buf, sizeof(buf), "Depth (%d,%d): no data", imgX, imgY);
    gtk_label_set_text(GTK_LABEL(statusLabel), buf);
    return TRUE;
}

gboolean onIRMotion(GtkWidget*, GdkEventMotion* event, gpointer) {
    std::lock_guard<std::mutex> lk(irMutex);
    if (latestIRRaw.empty()) return FALSE;
    int imgX = (int)event->x * latestIRRaw.cols / 640;
    int imgY = (int)event->y * latestIRRaw.rows / 480;
    if (imgX < 0 || imgX >= latestIRRaw.cols || imgY < 0 || imgY >= latestIRRaw.rows)
        return FALSE;
    uint16_t v = latestIRRaw.at<uint16_t>(imgY, imgX);
    char buf[128];
    if (v > 0)
        snprintf(buf, sizeof(buf), "IR (%d,%d): %d", imgX, imgY, v);
    else
        snprintf(buf, sizeof(buf), "IR (%d,%d): no data", imgX, imgY);
    gtk_label_set_text(GTK_LABEL(statusLabel), buf);
    return TRUE;
}

gboolean onLeave(GtkWidget*, GdkEventCrossing*, gpointer) {
    gtk_label_set_text(GTK_LABEL(statusLabel), "Hover depth or IR image for values");
    return TRUE;
}

int main(int argc, char** argv) {
    signal(SIGINT, signal_handler);
    setvbuf(stdout, NULL, _IONBF, 0);

    initDepthColorBar();

    std::thread t(streamThread);

    gtk_init(&argc, &argv);

    window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_window_set_title(GTK_WINDOW(window), "Astra Pro Depth + IR Viewer");
    gtk_window_set_default_size(GTK_WINDOW(window), 1280, 520);
    g_signal_connect(window, "destroy", G_CALLBACK(gtk_main_quit), NULL);

    GtkWidget* vbox = gtk_box_new(GTK_ORIENTATION_VERTICAL, 0);
    gtk_container_add(GTK_CONTAINER(window), vbox);

    GtkWidget* hbox = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 0);
    gtk_box_pack_start(GTK_BOX(vbox), hbox, TRUE, TRUE, 0);

    depthImage = gtk_image_new();
    irImage = gtk_image_new();

    GtkWidget* depthEventBox = gtk_event_box_new();
    gtk_container_add(GTK_CONTAINER(depthEventBox), depthImage);
    gtk_widget_add_events(depthEventBox, GDK_POINTER_MOTION_MASK | GDK_LEAVE_NOTIFY_MASK);
    g_signal_connect(depthEventBox, "motion-notify-event", G_CALLBACK(onDepthMotion), NULL);
    g_signal_connect(depthEventBox, "leave-notify-event", G_CALLBACK(onLeave), NULL);

    GtkWidget* irEventBox = gtk_event_box_new();
    gtk_container_add(GTK_CONTAINER(irEventBox), irImage);
    gtk_widget_add_events(irEventBox, GDK_POINTER_MOTION_MASK | GDK_LEAVE_NOTIFY_MASK);
    g_signal_connect(irEventBox, "motion-notify-event", G_CALLBACK(onIRMotion), NULL);
    g_signal_connect(irEventBox, "leave-notify-event", G_CALLBACK(onLeave), NULL);

    gtk_box_pack_start(GTK_BOX(hbox), depthEventBox, TRUE, TRUE, 0);
    gtk_box_pack_start(GTK_BOX(hbox), irEventBox, TRUE, TRUE, 0);

    statusLabel = gtk_label_new("Hover depth or IR image for values");
    gtk_box_pack_start(GTK_BOX(vbox), statusLabel, FALSE, FALSE, 5);

    gtk_widget_show_all(window);

    g_timeout_add(33, updateImages, NULL);

    gtk_main();

    running = false;
    if (t.joinable()) t.join();

    printf("Done!\n");
    return 0;
}
