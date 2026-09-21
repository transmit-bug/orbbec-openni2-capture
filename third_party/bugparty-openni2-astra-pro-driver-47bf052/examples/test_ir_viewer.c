/**
 * Astra Pro IR Viewer - GTK3 + OpenNI2
 * Reads IR stream, displays with OpenCV colormap
 * Laser on/off toggle via OBEXTENSION_ID_LASER_EN (0x0F)
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

std::mutex irMutex;
cv::Mat latestIR, latestIRRaw;
bool newIRReady = false;

GtkWidget *window, *irImage, *statusLabel, *laserButton;
OniDeviceHandle g_device = NULL;
std::atomic<bool> laserOn{true};

void setLaser(bool on) {
    if(!g_device) return;
    uint8_t val = on ? 1 : 0;
    OniStatus rc = oniDeviceSetProperty(g_device, OBEXTENSION_ID_LASER_EN, &val, sizeof(val));
    if(rc == ONI_STATUS_OK) {
        laserOn = on;
        printf("Laser: %s (rc=0 OK)\n", on ? "ON" : "OFF");
    } else {
        fprintf(stderr, "Set laser failed: rc=%d\n", rc);
    }
}

void onLaserToggle(GtkButton *btn, gpointer) {
    bool newState = !laserOn;
    setLaser(newState);
    gtk_button_set_label(btn, laserOn ? "Laser: ON" : "Laser: OFF");
}

void irThread() {
    OniStatus rc = oniInitialize(ONI_API_VERSION);
    if(rc != ONI_STATUS_OK) {
        fprintf(stderr, "OpenNI2 init failed: %d\n", rc);
        return;
    }

    OniDeviceInfo* deviceInfos;
    int deviceCount;
    rc = oniGetDeviceList(&deviceInfos, &deviceCount);
    if(rc != ONI_STATUS_OK || deviceCount == 0) {
        fprintf(stderr, "No devices found\n");
        oniShutdown();
        return;
    }

    rc = oniDeviceOpen(deviceInfos[0].uri, &g_device);
    if(rc != ONI_STATUS_OK) {
        fprintf(stderr, "Device open failed\n");
        oniShutdown();
        return;
    }
    printf("Device: %s\n", deviceInfos[0].name);

    // Set laser ON by default (getProperty doesn't work, but setProperty does)
    setLaser(true);

    const OniSensorInfo* irInfo = oniDeviceGetSensorInfo(g_device, ONI_SENSOR_IR);
    if(!irInfo) {
        fprintf(stderr, "No IR sensor\n");
        oniDeviceClose(g_device);
        oniShutdown();
        return;
    }
    printf("IR modes: %d\n", irInfo->numSupportedVideoModes);
    for(int i = 0; i < irInfo->numSupportedVideoModes; i++) {
        const OniVideoMode* m = &irInfo->pSupportedVideoModes[i];
        printf("  [%d] %dx%d @ %dfps pixel=%d\n", i, m->resolutionX, m->resolutionY, m->fps, m->pixelFormat);
    }

    OniStreamHandle irStream;
    rc = oniDeviceCreateStream(g_device, ONI_SENSOR_IR, &irStream);
    if(rc != ONI_STATUS_OK) {
        fprintf(stderr, "Create IR stream failed: %d\n", rc);
        oniDeviceClose(g_device);
        oniShutdown();
        return;
    }

    OniVideoMode mode;
    mode.resolutionX = 640;
    mode.resolutionY = 480;
    mode.fps = 30;
    mode.pixelFormat = ONI_PIXEL_FORMAT_GRAY16;
    rc = oniStreamSetProperty(irStream, ONI_STREAM_PROPERTY_VIDEO_MODE, &mode, sizeof(mode));
    if(rc != ONI_STATUS_OK)
        fprintf(stderr, "Set IR mode failed: %d (using default)\n", rc);

    OniVideoMode curMode;
    int curModeSize = sizeof(curMode);
    rc = oniStreamGetProperty(irStream, ONI_STREAM_PROPERTY_VIDEO_MODE, &curMode, &curModeSize);
    if(rc == ONI_STATUS_OK)
        printf("IR: %dx%d @ %dfps fmt=%d\n", curMode.resolutionX, curMode.resolutionY, curMode.fps, curMode.pixelFormat);

    rc = oniStreamStart(irStream);
    if(rc != ONI_STATUS_OK) {
        fprintf(stderr, "Start IR failed: %d\n", rc);
        oniStreamDestroy(irStream);
        oniDeviceClose(g_device);
        oniShutdown();
        return;
    }

    printf("IR streaming\n");
    int frameCount = 0, fpsCount = 0;
    float fps = 0;
    auto t0 = std::chrono::steady_clock::now();

    while(running) {
        OniFrame* frame;
        rc = oniStreamReadFrame(irStream, &frame);
        if(rc != ONI_STATUS_OK || !frame || frame->dataSize == 0)
            continue;

        frameCount++;
        fpsCount++;

        cv::Mat raw(frame->height, frame->width, CV_16UC1, (uint16_t*)frame->data);

        double minV, maxV;
        cv::minMaxLoc(raw, &minV, &maxV, nullptr, nullptr, raw > 0);

        cv::Mat vis;
        if(maxV > 0) {
            double normMax = std::min(maxV, 4000.0);
            raw.convertTo(vis, CV_8U, 255.0 / normMax, 0);
            cv::applyColorMap(vis, vis, cv::COLORMAP_HOT);
            cv::Mat noData;
            cv::threshold(raw, noData, 0, 255, cv::THRESH_BINARY_INV);
            noData.convertTo(noData, CV_8U);
            vis.setTo(cv::Vec3b(20, 20, 20), noData > 0);
        } else {
            vis = cv::Mat::zeros(frame->height, frame->width, CV_8UC3);
        }

        auto now = std::chrono::steady_clock::now();
        float dt = std::chrono::duration<float>(now - t0).count();
        if(dt >= 1.0f) { fps = fpsCount / dt; fpsCount = 0; t0 = now; }

        char txt[64];
        snprintf(txt, sizeof(txt), "IR: %.1f FPS", fps);
        cv::putText(vis, txt, cv::Point(10,25), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255,255,255), 2);
        snprintf(txt, sizeof(txt), "Max: %d (10-bit: %d)", (int)maxV, (int)maxV>>2);
        cv::putText(vis, txt, cv::Point(10,50), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,255), 1);

        const char *laserStr = laserOn ? "LASER ON" : "LASER OFF";
        cv::putText(vis, laserStr, cv::Point(10, vis.rows-10), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                    laserOn ? cv::Scalar(0,0,255) : cv::Scalar(100,100,100), 2);

        {
            std::lock_guard<std::mutex> lk(irMutex);
            latestIR = vis.clone();
            latestIRRaw = raw.clone();
            newIRReady = true;
        }

        oniFrameRelease(frame);
    }

    oniStreamStop(irStream);
    oniStreamDestroy(irStream);
    oniDeviceClose(g_device);
    g_device = NULL;
    oniShutdown();
    printf("IR thread stopped\n");
}

gboolean updateImage(gpointer) {
    if(!running) return G_SOURCE_REMOVE;
    cv::Mat img;
    {
        std::lock_guard<std::mutex> lk(irMutex);
        if(newIRReady && !latestIR.empty()) {
            img = latestIR.clone();
            newIRReady = false;
        }
    }
    if(!img.empty() && irImage) {
        cv::Mat rgb;
        cv::cvtColor(img, rgb, cv::COLOR_BGR2RGB);
        void* d = malloc(rgb.total() * rgb.elemSize());
        memcpy(d, rgb.data, rgb.total() * rgb.elemSize());
        GdkPixbuf* pb = gdk_pixbuf_new_from_data(
            (const guchar*)d, GDK_COLORSPACE_RGB, FALSE, 8, rgb.cols, rgb.rows, rgb.step,
            (GdkPixbufDestroyNotify)g_free, d);
        gtk_image_set_from_pixbuf(GTK_IMAGE(irImage), pb);
        g_object_unref(pb);
    }
    return G_SOURCE_CONTINUE;
}

gboolean onMotion(GtkWidget *w, GdkEventMotion *e, gpointer) {
    if(!gtk_widget_get_realized(w)) return FALSE;
    GtkAllocation a; gtk_widget_get_allocation(w, &a);
    int ix = (int)e->x * latestIRRaw.cols / a.width;
    int iy = (int)e->y * latestIRRaw.rows / a.height;
    std::lock_guard<std::mutex> lk(irMutex);
    if(latestIRRaw.empty() || ix<0 || ix>=latestIRRaw.cols || iy<0 || iy>=latestIRRaw.rows)
        return FALSE;
    uint16_t v = latestIRRaw.at<uint16_t>(iy, ix);
    char buf[128];
    if(v>0) snprintf(buf, sizeof(buf), "(%d,%d): IR=%d (10-bit: %d)", ix, iy, v, v>>2);
    else snprintf(buf, sizeof(buf), "(%d,%d): no data", ix, iy);
    gtk_label_set_text(GTK_LABEL(statusLabel), buf);
    return TRUE;
}

gboolean onLeave(GtkWidget*, GdkEventCrossing*, gpointer) {
    gtk_label_set_text(GTK_LABEL(statusLabel), "Hover for IR value");
    return TRUE;
}

int main(int argc, char** argv) {
    signal(SIGINT, signal_handler);
    setvbuf(stdout, NULL, _IONBF, 0);
    setenv("OPENNI2_REPO", "/home/bowmanhan/Code/OrbbecSDK/AstraSDK-v2.1.3/lib/Plugins/openni2/Drivers", 1);

    std::thread irT(irThread);

    gtk_init(&argc, &argv);
    window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_window_set_title(GTK_WINDOW(window), "Astra Pro IR Viewer");
    gtk_window_set_default_size(GTK_WINDOW(window), 700, 560);
    g_signal_connect(window, "destroy", G_CALLBACK(gtk_main_quit), NULL);

    GtkWidget *vbox = gtk_box_new(GTK_ORIENTATION_VERTICAL, 0);
    gtk_container_add(GTK_CONTAINER(window), vbox);

    // Toolbar
    GtkWidget *toolbar = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 5);
    gtk_box_pack_start(GTK_BOX(vbox), toolbar, FALSE, FALSE, 2);

    laserButton = gtk_button_new_with_label("Laser: ON");
    g_signal_connect(laserButton, "clicked", G_CALLBACK(onLaserToggle), NULL);
    gtk_box_pack_start(GTK_BOX(toolbar), laserButton, FALSE, FALSE, 5);

    // IR image
    irImage = gtk_image_new();
    GtkWidget *eb = gtk_event_box_new();
    gtk_container_add(GTK_CONTAINER(eb), irImage);
    gtk_widget_add_events(eb, GDK_POINTER_MOTION_MASK | GDK_LEAVE_NOTIFY_MASK);
    g_signal_connect(eb, "motion-notify-event", G_CALLBACK(onMotion), NULL);
    g_signal_connect(eb, "leave-notify-event", G_CALLBACK(onLeave), NULL);
    gtk_box_pack_start(GTK_BOX(vbox), eb, TRUE, TRUE, 0);

    statusLabel = gtk_label_new("Hover for IR value");
    gtk_box_pack_start(GTK_BOX(vbox), statusLabel, FALSE, FALSE, 5);

    gtk_widget_show_all(window);
    printf("IR viewer started\n");
    g_timeout_add(33, updateImage, NULL);
    gtk_main();

    running = false;
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    if(irT.joinable()) irT.join();
    printf("Done\n");
    return 0;
}
