/**
 * Astra Pro IR Laser Control — GTK3 + OpenNI2
 *
 * Toggle laser on/off with a button, see live IR stream.
 * Uses OBEXTENSION_ID_LASER_EN (property 15) via our driver.
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
cv::Mat latestIr;
bool newIrReady = false;

GtkWidget *window, *irImage, *statusLabel, *laserBtn;
OniDeviceHandle g_device = NULL;
std::atomic<bool> g_laserOn{false};

void setLaser(bool on) {
    if (!g_device) return;
    uint8_t val = on ? 1 : 0;
    OniStatus rc = oniDeviceSetProperty(g_device, OBEXTENSION_ID_LASER_EN, &val, sizeof(val));
    if (rc == ONI_STATUS_OK) {
        g_laserOn.store(on);
        gtk_button_set_label(GTK_BUTTON(laserBtn), on ? "Laser ON (click to turn off)" : "Laser OFF (click to turn on)");
    } else {
        fprintf(stderr, "setLaser(%d) failed: %d\n", on, rc);
    }
}

void onLaserBtn(GtkWidget*, gpointer) {
    setLaser(!g_laserOn.load());
}

void irThread() {
    OniStatus rc = oniInitialize(ONI_API_VERSION);
    if (rc != ONI_STATUS_OK) {
        fprintf(stderr, "OpenNI2 init failed: %d\n", rc);
        return;
    }

    OniDeviceInfo* devs;
    int n;
    rc = oniGetDeviceList(&devs, &n);
    if (rc != ONI_STATUS_OK || n == 0) {
        fprintf(stderr, "No devices found\n");
        oniShutdown();
        return;
    }

    OniDeviceHandle device;
    rc = oniDeviceOpen(devs[0].uri, &device);
    if (rc != ONI_STATUS_OK) {
        fprintf(stderr, "Device open failed\n");
        oniShutdown();
        return;
    }
    g_device = device;
    printf("Device: %s\n", devs[0].name);

    OniStreamHandle irStream;
    rc = oniDeviceCreateStream(device, ONI_SENSOR_IR, &irStream);
    if (rc != ONI_STATUS_OK) {
        fprintf(stderr, "Create IR stream failed: %d\n", rc);
        oniDeviceClose(device);
        oniShutdown();
        return;
    }

    OniVideoMode mode;
    mode.resolutionX = 640;
    mode.resolutionY = 480;
    mode.fps = 30;
    mode.pixelFormat = ONI_PIXEL_FORMAT_GRAY16;
    rc = oniStreamSetProperty(irStream, ONI_STREAM_PROPERTY_VIDEO_MODE, &mode, sizeof(mode));
    if (rc != ONI_STATUS_OK)
        fprintf(stderr, "Set IR mode failed: %d (using default)\n", rc);

    rc = oniStreamStart(irStream);
    if (rc != ONI_STATUS_OK) {
        fprintf(stderr, "Start IR failed: %d\n", rc);
        oniStreamDestroy(irStream);
        oniDeviceClose(device);
        oniShutdown();
        return;
    }

    // Turn laser on by default for IR visibility
    setLaser(true);

    printf("IR streaming\n");
    int frameCount = 0, fpsCount = 0;
    float fps = 0;
    auto t0 = std::chrono::steady_clock::now();

    while (running) {
        OniFrame* frame;
        rc = oniStreamReadFrame(irStream, &frame);
        if (rc != ONI_STATUS_OK || !frame || frame->dataSize == 0)
            continue;

        frameCount++;
        fpsCount++;

        cv::Mat raw(frame->height, frame->width, CV_16UC1, (uint16_t*)frame->data);

        double minV, maxV;
        cv::minMaxLoc(raw, &minV, &maxV, nullptr, nullptr, raw > 0);

        cv::Mat vis;
        if (maxV > 0) {
            double displayMax = (maxV < 1024.0) ? 1024.0 : maxV;
            raw.convertTo(vis, CV_8U, 255.0 / displayMax, 0);
            cv::applyColorMap(vis, vis, cv::COLORMAP_JET);
            vis.setTo(cv::Vec3b(0, 0, 0), raw == 0);
        } else {
            vis = cv::Mat::zeros(frame->height, frame->width, CV_8UC3);
        }

        auto now = std::chrono::steady_clock::now();
        float dt = std::chrono::duration<float>(now - t0).count();
        if (dt >= 1.0f) { fps = fpsCount / dt; fpsCount = 0; t0 = now; }

        char txt[64];
        snprintf(txt, sizeof(txt), "IR: %.1f FPS", fps);
        cv::putText(vis, txt, cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255,255,255), 2);
        snprintf(txt, sizeof(txt), "Laser: %s", g_laserOn.load() ? "ON" : "OFF");
        cv::putText(vis, txt, cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    g_laserOn.load() ? cv::Scalar(0,255,0) : cv::Scalar(0,0,255), 1);

        {
            std::lock_guard<std::mutex> lk(irMutex);
            latestIr = vis.clone();
            newIrReady = true;
        }

        oniFrameRelease(frame);
    }

    setLaser(false);
    oniStreamStop(irStream);
    oniStreamDestroy(irStream);
    g_device = NULL;
    oniDeviceClose(device);
    oniShutdown();
    printf("IR thread stopped\n");
}

gboolean updateImage(gpointer) {
    if (!running) return G_SOURCE_REMOVE;
    cv::Mat img;
    {
        std::lock_guard<std::mutex> lk(irMutex);
        if (newIrReady && !latestIr.empty()) {
            img = latestIr.clone();
            newIrReady = false;
        }
    }
    if (!img.empty() && irImage) {
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

int main(int argc, char** argv) {
    signal(SIGINT, signal_handler);
    setvbuf(stdout, NULL, _IONBF, 0);
    setenv("OPENNI2_REPO", "/home/bowmanhan/Code/OrbbecSDK/AstraSDK-v2.1.3/lib/Plugins/openni2/OpenNI2/Drivers", 1);

    std::thread t(irThread);

    gtk_init(&argc, &argv);
    window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_window_set_title(GTK_WINDOW(window), "Astra Pro IR + Laser Control");
    gtk_window_set_default_size(GTK_WINDOW(window), 700, 560);
    g_signal_connect(window, "destroy", G_CALLBACK(gtk_main_quit), NULL);

    GtkWidget *vbox = gtk_box_new(GTK_ORIENTATION_VERTICAL, 4);
    gtk_container_add(GTK_CONTAINER(window), vbox);

    irImage = gtk_image_new();
    gtk_box_pack_start(GTK_BOX(vbox), irImage, TRUE, TRUE, 0);

    laserBtn = gtk_button_new_with_label("Laser OFF (click to turn on)");
    g_signal_connect(laserBtn, "clicked", G_CALLBACK(onLaserBtn), NULL);
    gtk_box_pack_start(GTK_BOX(vbox), laserBtn, FALSE, FALSE, 4);

    statusLabel = gtk_label_new("IR + Laser Control");
    gtk_box_pack_start(GTK_BOX(vbox), statusLabel, FALSE, FALSE, 2);

    gtk_widget_show_all(window);
    g_timeout_add(33, updateImage, NULL);
    gtk_main();

    running = false;
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    if (t.joinable()) t.join();
    return 0;
}
