/**
 * Astra Pro Depth Viewer - GTK3 + OpenNI2 C API
 *
 * Reverse-engineered depth data flow:
 *   USB Bulk (Depth Endpoint, pSpecificDepthUsb)
 *     → XnDeviceSensorProtocolUsbEpCb (state machine: MAGIC→HEADER→DATA)
 *     → XnFirmwareStreams::ProcessPacketChunk (dispatch by nType)
 *     → XnDepthProcessor (base: SOF=0x7100, EOF=0x7500)
 *       ├── XnUncompressedDepthProcessor: raw shift → ShiftToDepth LUT
 *       ├── XnPacked11DepthProcessor: 11-bit packed → 16-bit (11B→8px) → LUT
 *       └── XnPSCompressedDepthProcessor: 4-bit RLE/diff → LUT
 *     → TripleBuffer → oniStreamReadFrame()
 *
 * Depth opens: Stream1Mode = DEPTH, USB read on pSpecificDepthUsb
 * Depth values: shift → ShiftToDepthTable[] → millimeters (uint16)
 * Supported formats: PSCompressed, Uncompressed11, Uncompressed16
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

std::mutex depthMutex;
cv::Mat latestDepth, latestDepthRaw;
bool newDepthReady = false;

GtkWidget *window, *depthImage, *statusLabel;

const int MAX_DEPTH_MM = 9470;    // Astra Pro max depth from S2D table
const int NEAR_DEPTH_MM = 400;   // Near cutoff for display scaling (below this is likely sensor artifact)
const int DISPLAY_MAX_MM = 5000; // Display range: near objects get more color resolution

GtkWidget *colorBarArea;

// JET colormap: map t∈[0,1] → RGB (matching OpenCV COLORMAP_JET)
static void jetColor(float t, double &r, double &g, double &b) {
    t = t < 0 ? 0 : t > 1 ? 1 : t;
    if (t < 0.125)      { r=0;           g=0;           b=0.5+t*4; }
    else if (t < 0.375) { r=0;           g=(t-0.125)*4; b=1; }
    else if (t < 0.625) { r=(t-0.375)*4; g=1;           b=1-(t-0.375)*4; }
    else if (t < 0.875) { r=1;           g=1-(t-0.625)*4; b=0; }
    else                 { r=1-(t-0.875)*4; g=0;           b=(t-0.875)*4; }
}

gboolean onDrawColorBar(GtkWidget *widget, cairo_t *cr, gpointer) {
    GtkAllocation a; gtk_widget_get_allocation(widget, &a);
    int w = a.width, h = a.height;
    int barX = 10, barW = 24;
    int barY = 20, barH = h - 40;

    // Draw gradient bar
    for (int y = 0; y < barH; y++) {
        float t = 1.0f - (float)y / barH; // top=1.0 (far), bottom=0.0 (near)
        double r, g, b;
        jetColor(t, r, g, b);
        cairo_set_source_rgb(cr, r, g, b);
        cairo_rectangle(cr, barX, barY + y, barW, 1);
        cairo_fill(cr);
    }

    // Border
    cairo_set_source_rgb(cr, 0.6, 0.6, 0.6);
    cairo_rectangle(cr, barX, barY, barW, barH);
    cairo_stroke(cr);

    // Tick marks and labels
    cairo_set_source_rgb(cr, 0, 0, 0);
    cairo_select_font_face(cr, "monospace", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_NORMAL);
    cairo_set_font_size(cr, 11);
    float ticks[] = {0.3f, 0.5f, 1.0f, 1.5f, 2.0f, 3.0f, 5.0f};
    for (float d : ticks) {
        float meters = d;
        int depthMM = (int)(meters * 1000);
        // Map depth to position using same scale as display: [NEAR_DEPTH_MM, DISPLAY_MAX_MM] → [0, 1]
        float t = (float)(depthMM - NEAR_DEPTH_MM) / (float)(DISPLAY_MAX_MM - NEAR_DEPTH_MM);
        t = t < 0 ? 0 : t > 1 ? 1 : t;
        int ty = barY + (int)(barH * (1.0f - t));
        cairo_move_to(cr, barX + barW + 3, ty + 4);
        char lbl[16]; snprintf(lbl, sizeof(lbl), "%.2fm", d);
        cairo_show_text(cr, lbl);
        cairo_move_to(cr, barX + barW, ty);
        cairo_line_to(cr, barX + barW + 2, ty);
        cairo_stroke(cr);
    }
    return TRUE;
}

void depthThread() {
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

    OniDeviceHandle device;
    rc = oniDeviceOpen(deviceInfos[0].uri, &device);
    if(rc != ONI_STATUS_OK) {
        fprintf(stderr, "Device open failed\n");
        oniShutdown();
        return;
    }
    printf("Device: %s\n", deviceInfos[0].name);

    const OniSensorInfo* depthInfo = oniDeviceGetSensorInfo(device, ONI_SENSOR_DEPTH);
    if(!depthInfo) {
        fprintf(stderr, "No depth sensor\n");
        oniDeviceClose(device);
        oniShutdown();
        return;
    }
    printf("Depth modes: %d\n", depthInfo->numSupportedVideoModes);
    for(int i = 0; i < depthInfo->numSupportedVideoModes; i++) {
        const OniVideoMode* m = &depthInfo->pSupportedVideoModes[i];
        printf("  [%d] %dx%d @ %dfps pixel=%d\n", i, m->resolutionX, m->resolutionY, m->fps, m->pixelFormat);
    }

    OniStreamHandle depthStream;
    rc = oniDeviceCreateStream(device, ONI_SENSOR_DEPTH, &depthStream);
    if(rc != ONI_STATUS_OK) {
        fprintf(stderr, "Create depth stream failed: %d\n", rc);
        oniDeviceClose(device);
        oniShutdown();
        return;
    }

    OniVideoMode mode;
    mode.resolutionX = 640;
    mode.resolutionY = 480;
    mode.fps = 30;
    mode.pixelFormat = ONI_PIXEL_FORMAT_DEPTH_1_MM;
    rc = oniStreamSetProperty(depthStream, ONI_STREAM_PROPERTY_VIDEO_MODE, &mode, sizeof(mode));
    if(rc != ONI_STATUS_OK)
        fprintf(stderr, "Set depth mode failed: %d (using default)\n", rc);

    OniVideoMode curMode;
    int sz = sizeof(curMode);
    rc = oniStreamGetProperty(depthStream, ONI_STREAM_PROPERTY_VIDEO_MODE, &curMode, &sz);
    if(rc == ONI_STATUS_OK)
        printf("Depth: %dx%d @ %dfps fmt=%d\n", curMode.resolutionX, curMode.resolutionY, curMode.fps, curMode.pixelFormat);

    rc = oniStreamStart(depthStream);
    if(rc != ONI_STATUS_OK) {
        fprintf(stderr, "Start depth failed: %d\n", rc);
        oniStreamDestroy(depthStream);
        oniDeviceClose(device);
        oniShutdown();
        return;
    }

    printf("Depth streaming\n");
    int frameCount = 0, fpsCount = 0;
    float fps = 0;
    auto t0 = std::chrono::steady_clock::now();

    while(running) {
        OniFrame* frame;
        rc = oniStreamReadFrame(depthStream, &frame);
        if(rc != ONI_STATUS_OK || !frame || frame->dataSize == 0)
            continue;

        frameCount++;
        fpsCount++;

        cv::Mat raw(frame->height, frame->width, CV_16UC1, (uint16_t*)frame->data);

        // Debug: print raw pixel data for first 5 frames
        if(frameCount <= 3) {
            uint16_t* p = (uint16_t*)frame->data;
            fprintf(stderr, "VIEWER frame%d: %dx%d stride=%d dataSize=%d\n",
                    frameCount, frame->width, frame->height, frame->stride, frame->dataSize);
            fprintf(stderr, "  row0[0..9]:");
            for(int i=0;i<10;i++) fprintf(stderr," %u", p[i]);
            fprintf(stderr, "\n  row1[0..9]:");
            for(int i=frame->width;i<frame->width+10;i++) fprintf(stderr," %u", p[i]);
            fprintf(stderr, "\n  row479[0..9]:");
            for(int i=frame->width*479;i<frame->width*479+10;i++) fprintf(stderr," %u", p[i]);
            fprintf(stderr, "\n");

            // Save raw depth as PNG for offline analysis
            cv::Mat rawCopy = raw.clone();
            if(frameCount == 2) {
                // Save both raw 16-bit and 8-bit visual
                cv::imwrite("/tmp/depth_raw_16bit.png", rawCopy);
                cv::Mat disp;
                rawCopy.convertTo(disp, CV_8U, 255.0 / 5000.0, 0);
                cv::applyColorMap(disp, disp, cv::COLORMAP_JET);
                cv::imwrite("/tmp/depth_visual.png", disp);
                fprintf(stderr, "  Saved /tmp/depth_raw_16bit.png and /tmp/depth_visual.png\n");
            }
        }

        double minV, maxV;
        cv::minMaxLoc(raw, &minV, &maxV, nullptr, nullptr, raw > 0);

        cv::Mat vis;
        if(maxV > 0) {
            // Log-scale normalization: spread near objects across more of the color range
            // Maps [NEAR_DEPTH_MM, DISPLAY_MAX_MM] → [0, 255]
            // Values below NEAR_DEPTH_MM → 0, above DISPLAY_MAX_MM → 255
            cv::Mat normed = cv::Mat::zeros(raw.size(), CV_8U);
            for (int y = 0; y < raw.rows; y++) {
                const uint16_t* src = raw.ptr<uint16_t>(y);
                uint8_t* dst = normed.ptr<uint8_t>(y);
                for (int x = 0; x < raw.cols; x++) {
                    uint16_t d = src[x];
                    if (d >= NEAR_DEPTH_MM && d <= DISPLAY_MAX_MM) {
                        dst[x] = static_cast<uint8_t>(255.0 * (d - NEAR_DEPTH_MM) / (DISPLAY_MAX_MM - NEAR_DEPTH_MM));
                    } else if (d > DISPLAY_MAX_MM) {
                        dst[x] = 255;
                    }
                    // else d < NEAR_DEPTH_MM or d == 0 → stays 0
                }
            }
            cv::applyColorMap(normed, vis, cv::COLORMAP_JET);
            // Zero-depth pixels → black (not JET blue)
            vis.setTo(cv::Vec3b(0, 0, 0), raw == 0);
        } else {
            vis = cv::Mat::zeros(frame->height, frame->width, CV_8UC3);
        }

        auto now = std::chrono::steady_clock::now();
        float dt = std::chrono::duration<float>(now - t0).count();
        if(dt >= 1.0f) { fps = fpsCount / dt; fpsCount = 0; t0 = now; }

        char txt[64];
        snprintf(txt, sizeof(txt), "Depth: %.1f FPS", fps);
        cv::putText(vis, txt, cv::Point(10,25), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255,255,255), 2);
        snprintf(txt, sizeof(txt), "Max: %.2fm Range: %.1f-%.1fm", maxV / 1000.0f, minV / 1000.0f, maxV / 1000.0f);
        cv::putText(vis, txt, cv::Point(10,50), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,255), 1);
        snprintf(txt, sizeof(txt), "Frame: %d", frameCount);
        cv::putText(vis, txt, cv::Point(10,75), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(200,200,200), 1);

        {
            std::lock_guard<std::mutex> lk(depthMutex);
            latestDepth = vis.clone();
            latestDepthRaw = raw.clone();
            newDepthReady = true;
        }

        oniFrameRelease(frame);
    }

    oniStreamStop(depthStream);
    oniStreamDestroy(depthStream);
    oniDeviceClose(device);
    oniShutdown();
    printf("Depth thread stopped\n");
}

gboolean updateImage(gpointer) {
    if(!running) return G_SOURCE_REMOVE;
    cv::Mat img;
    {
        std::lock_guard<std::mutex> lk(depthMutex);
        if(newDepthReady && !latestDepth.empty()) {
            img = latestDepth.clone();
            newDepthReady = false;
        }
    }
    if(!img.empty() && depthImage) {
        cv::Mat rgb;
        cv::cvtColor(img, rgb, cv::COLOR_BGR2RGB);
        void* d = malloc(rgb.total() * rgb.elemSize());
        memcpy(d, rgb.data, rgb.total() * rgb.elemSize());
        GdkPixbuf* pb = gdk_pixbuf_new_from_data(
            (const guchar*)d, GDK_COLORSPACE_RGB, FALSE, 8, rgb.cols, rgb.rows, rgb.step,
            (GdkPixbufDestroyNotify)g_free, d);
        gtk_image_set_from_pixbuf(GTK_IMAGE(depthImage), pb);
        g_object_unref(pb);
    }
    return G_SOURCE_CONTINUE;
}

gboolean onMotion(GtkWidget *w, GdkEventMotion *e, gpointer) {
    if(!gtk_widget_get_realized(w)) return FALSE;
    GtkAllocation a; gtk_widget_get_allocation(w, &a);
    int ix = (int)e->x * latestDepthRaw.cols / a.width;
    int iy = (int)e->y * latestDepthRaw.rows / a.height;
    std::lock_guard<std::mutex> lk(depthMutex);
    if(latestDepthRaw.empty() || ix < 0 || ix >= latestDepthRaw.cols || iy < 0 || iy >= latestDepthRaw.rows)
        return FALSE;
    uint16_t d = latestDepthRaw.at<uint16_t>(iy, ix);
    char buf[128];
    if(d > 0) snprintf(buf, sizeof(buf), "(%d,%d): %dmm (%.3fm)", ix, iy, d, d/1000.0f);
    else snprintf(buf, sizeof(buf), "(%d,%d): no depth", ix, iy);
    gtk_label_set_text(GTK_LABEL(statusLabel), buf);
    return TRUE;
}

gboolean onLeave(GtkWidget*, GdkEventCrossing*, gpointer) {
    gtk_label_set_text(GTK_LABEL(statusLabel), "Hover for depth");
    return TRUE;
}

int main(int argc, char** argv) {
    signal(SIGINT, signal_handler);
    setvbuf(stdout, NULL, _IONBF, 0);
    setenv("OPENNI2_REPO", "/home/bowmanhan/Code/OrbbecSDK/AstraSDK-v2.1.3/lib/Plugins/openni2/Drivers", 1);

    // initDepthColorBar removed — color bar is now a GTK/Cairo widget
    std::thread t(depthThread);

    gtk_init(&argc, &argv);
    window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_window_set_title(GTK_WINDOW(window), "Astra Pro Depth Viewer (OpenNI2)");
    gtk_window_set_default_size(GTK_WINDOW(window), 700, 560);
    g_signal_connect(window, "destroy", G_CALLBACK(gtk_main_quit), NULL);

    GtkWidget *vbox = gtk_box_new(GTK_ORIENTATION_VERTICAL, 0);
    gtk_container_add(GTK_CONTAINER(window), vbox);

    GtkWidget *hbox = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 0);
    gtk_box_pack_start(GTK_BOX(vbox), hbox, TRUE, TRUE, 0);

    depthImage = gtk_image_new();
    GtkWidget *eb = gtk_event_box_new();
    gtk_container_add(GTK_CONTAINER(eb), depthImage);
    gtk_widget_add_events(eb, GDK_POINTER_MOTION_MASK | GDK_LEAVE_NOTIFY_MASK);
    g_signal_connect(eb, "motion-notify-event", G_CALLBACK(onMotion), NULL);
    g_signal_connect(eb, "leave-notify-event", G_CALLBACK(onLeave), NULL);
    gtk_box_pack_start(GTK_BOX(hbox), eb, TRUE, TRUE, 0);

    colorBarArea = gtk_drawing_area_new();
    gtk_widget_set_size_request(colorBarArea, 80, -1);
    g_signal_connect(colorBarArea, "draw", G_CALLBACK(onDrawColorBar), NULL);
    gtk_box_pack_start(GTK_BOX(hbox), colorBarArea, FALSE, FALSE, 0);

    statusLabel = gtk_label_new("Hover for depth");
    gtk_box_pack_start(GTK_BOX(vbox), statusLabel, FALSE, FALSE, 5);

    gtk_widget_show_all(window);
    printf("Depth viewer started\n");
    g_timeout_add(33, updateImage, NULL);
    gtk_main();

    running = false;
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    if(t.joinable()) t.join();
    printf("Done\n");
    return 0;
}
