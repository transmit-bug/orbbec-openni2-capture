# Orbbec Astra Pro 摄像头接入规划（OpenNI2）

> 目标：在当前项目下接入 Orbbec Astra Pro 摄像头，利用 OpenNI2 SDK 读取
> 深度图 / 彩色图 / 点云数据，为后续 3D 感知应用提供数据源。

---

## 一、现状诊断

### 系统环境

| 项目 | 当前状态 |
|------|----------|
| 操作系统 | Omarchy（Arch Linux） |
| 项目目录 | `/home/pony/codehub/weimou/Sim/orbbec`（当前为空） |
| USB 枚举 | **暂未检测到 Orbbec 设备**（`lsusb` 中无 `2bc5` VID） |
| OpenNI2 | 未安装 |

### 硬件检查（优先解决）

Astra Pro 的 USB VID 通常为 `2bc5`。当前 `lsusb` 未枚举到该设备，可能原因：

1. USB 供电不足（Astra Pro 为 USB 2.0，但启动时需要约 500mA）。
2. 插在 USB 3.0 hub / 部分主板端口兼容性差。
3. 线缆质量差或接触不良。

**排查步骤：**

```bash
# 换一个 USB 口（优先直连主板 USB 2.0 口），重新插拔后执行：
lsusb | grep -i 2bc5
# 或
lsusb
# 应看到类似 "2bc5:0xxx Orbbec" 的条目
```

如果 `lsusb` 始终无法看到设备，请先换线/换口/换电脑排除硬件问题。

---

## 二、参考：OpenNI SDK 官方说明摘要

来源：[orbbec/OpenNI_SDK README](https://github.com/orbbec/OpenNI_SDK/blob/main/README.md)

- **支持的设备**：Astra Pro、Astra Mini(S)、Astra+(S)、Astra(S)、Astra Pro Plus 等。
- **API**：C / C++11 为主，另有 Java / Android 绑定。
- **Linux 依赖**：freeglut3、build-essential、gcc >= 4.9。
- **关键库文件**：`libOpenNI2.so`、`OpenNI2/Drivers/liborbbec.so`。
- **关键配置**：`OpenNI.ini`、`OpenNI2/Drivers/orbbec.ini`。
- **安装**：解压 SDK → 运行 `install.sh`（安装 udev 规则）→ `source OpenNIDevEnvironment`。

---

## 三、分阶段实施规划

### 阶段 0：环境准备（预计 0.5 天）

**目标：** SDK 安装完成，NiViewer 能看到深度图。

1. **确认硬件枚举**（见上文诊断）。
2. **安装系统依赖**（Arch Linux）：
   ```bash
   sudo pacman -S --needed base-devel cmake freeglut libusb
   ```
3. **下载 OpenNI SDK for Linux x64**：
   - 官方下载页：https://orbbec3d.com/
   - 下载 `OpenNI-Linux-x64-2.3.x.x.zip`，解压到项目下：
   ```bash
   cd /home/pony/codehub/weimou/Sim/orbbec
   mkdir -p third_party
   cd third_party
   unzip ~/Downloads/OpenNI-Linux-x64-2.3.*.zip
   ```
4. **安装 udev 规则 + 环境变量**：
   ```bash
   cd third_party/OpenNI-Linux-x64-2.3.*
   chmod +x install.sh
   sudo ./install.sh          # 安装 udev 规则（避免每次 sudo）
   source OpenNIDevEnvironment # 设置 OPENNI2_INCLUDE / OPENNI2_REDIST
   ```
5. **验证**：运行 SDK 自带 NiViewer：
   ```bash
   cd third_party/OpenNI-Linux-x64-2.3.*/Tools
   ./NiViewer
   ```
   应能看到实时深度图窗口。

### 阶段 1：项目骨架（预计 0.5 天）

**目标：** 建立可编译、可运行的 C++ CMake 工程，能读取一帧深度图并保存。

**目录结构：**

```
orbbec/
├── third_party/
│   └── OpenNI-Linux-x64-2.3.x.x/   # SDK 解压后
├── src/
│   ├── main.cpp                    # 入口：初始化→读一帧→保存
│   ├── camera_openni.cpp           # CameraOpenNI 封装类
│   └── camera_openni.h
├── scripts/
│   └── setup_env.sh               # source SDK 环境变量
├── CMakeLists.txt
└── README.md
```

**CMake 要点：**

```cmake
set(OPENNI_DIR ${CMAKE_SOURCE_DIR}/third_party/OpenNI-Linux-x64-2.3.x.x)
include_directories(${OPENNI_DIR}/Include)
link_directories(${OPENNI_DIR}/Redist)
target_link_libraries(app OpenNI2)
```

**运行时注意：** `libOpenNI2.so` 依赖 `OpenNI2/Drivers/liborbbec.so` 插件，
需保证可执行文件运行时能找到 `OpenNI2/Drivers/` 目录（通过 `LD_LIBRARY_PATH`
或 `OPENNI2_DRIVERS_PATH`，或在可执行文件旁建立符号链接）。

**验收：** 编译通过，运行后输出一帧深度 PNG（16-bit mm 值）到 `output/depth_0.png`。

### 阶段 2：多流采集封装（预计 1 天）

**目标：** 封装一个可复用的 `CameraOpenNI` 类，同时读取深度 + 彩色。

**Astra Pro 能力（参考 SDK 文档）：**

| 流 | 格式 | 分辨率 | 帧率 |
|----|------|--------|------|
| Depth | `PIXEL_FORMAT_DEPTH_1_MM` | 640×480 / 320×240 | 30 fps |
| Color | `PIXEL_FORMAT_RGB888` | 640×480 / 1280×720 | 30 fps |

**CameraOpenNI 接口草案：**

```cpp
class CameraOpenNI {
public:
    bool open(int deviceId = 0);
    void close();
    bool readDepth(cv::Mat& depth);     // CV_16UC1, 单位 mm
    bool readColor(cv::Mat& color);     // CV_8UC3, RGB
    void enableDepthToColorRegistration();
private:
    openni::Device device_;
    openni::VideoStream depthStream_, colorStream_;
};
```

**验收：** 同时显示深度图伪彩色窗口和彩色图窗口，30 fps 无丢帧。

### 阶段 3：点云生成（预计 1 天）

**目标：** 深度图 + 内参 → XYZ 点云，可选保存为 PCD/Ply。

**步骤：**
1. 从 `depthStream` 获取内参：
   ```cpp
   openni::CameraSettings* settings = depthStream_.getCameraSettings();
   // fx, fy, cx, cy
   ```
2. 逐像素反投影：
   ```
   X = (u - cx) * Z / fx
   Y = (v - cy) * Z / fy
   Z = depth(v, u)  // mm → m
   ```
3. 如开启 depth-to-color 配准，可生成彩色点云。

**验收：** 用 CloudCompare / Open3D 打开保存的 PCD 文件，点云形状合理。

### 阶段 4：应用层集成（预计 1-2 天，视需求而定）

根据具体应用选择：

- **ROS 集成**：将 CameraOpenNI 封装为 ROS node，发布
  `sensor_msgs/Image` + `sensor_msgs/CameraInfo` + `sensor_msgs/PointCloud2`。
  （也可直接使用社区 `openni2_camera` ROS 包，但对 Astra Pro 兼容性需验证。）
- **手眼标定**：采集棋盘格数据，配合彩色图做相机标定。
- **3D 重建 / SLAM**：将点云接入现有 SLAM 管线。

### 阶段 5：稳定性与部署（预计 0.5 天）

1. **udev 规则**：确认 `install.sh` 已安装，普通用户可免 sudo 访问设备。
2. **异常处理**：USB 掉线重连、流打开失败、帧超时。
3. **配置文件**：分辨率 / fps / 是否开启配准可配置。
4. **日志**：关键操作 + 帧率统计。

---

## 四、风险与注意事项

| 风险 | 缓解措施 |
|------|----------|
| USB 枚举失败 | 换直连 USB 2.0 口；使用带供电的 hub |
| Arch 上 SDK 兼容性 | SDK 是预编译 .so，理论兼容；如遇 glibc 冲突用 ldd 检查 |
| 深度/彩色不同步 | 使用 OpenNI 的 depth-to-color registration + timestamp 对齐 |
| 精度受限 | Astra Pro 深度最优范围约 0.6–8 m，近距离会有噪声 |

---

## 五、时间线汇总

| 阶段 | 内容 | 预计时间 |
|------|------|----------|
| 0 | 环境准备 + NiViewer 验证 | 0.5 天 |
| 1 | CMake 骨架 + 读一帧深度 | 0.5 天 |
| 2 | 多流封装（深度 + 彩色） | 1 天 |
| 3 | 点云生成 | 1 天 |
| 4 | 应用层集成 | 1-2 天 |
| 5 | 稳定性 + 部署 | 0.5 天 |
| **合计** | | **约 4.5-5.5 天** |

