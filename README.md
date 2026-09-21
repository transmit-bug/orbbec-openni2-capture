# Orbbec Astra Pro 采集示例（OpenNI2）

基于 OpenNI2 读取 Orbbec Astra Pro 的深度图与彩色图的最小示例。
原理与分阶段规划见 [PLAN.md](PLAN.md)。

## 目录结构

```
orbbec/
├── CMakeLists.txt                              # 构建入口
├── src/
│   ├── main.cpp                                # 读一帧深度 + 一帧彩色并落盘
│   ├── camera_openni.h / .cpp                  # CameraOpenNI 封装
├── scripts/
│   ├── run.sh                                  # 设置运行环境并启动程序
│   └── setup_env.sh                            # 导出 OpenNI2 环境变量（可选）
├── output/                                     # 采集结果输出目录
└── third_party/
    ├── openni2_runtime/                        # 精简 OpenNI2 运行时 + 头文件（构建依赖）
    ├── bugparty-openni2-astra-pro-driver-*/    # 开源 Astra Pro 驱动源码（构建依赖）
    └── OpenNI_2.3.0.86_*/rules/                # udev 规则（一次性系统安装）
```

## 依赖

```bash
sudo pacman -S --needed base-devel cmake libusb
```

## udev 规则（首次使用，需 root）

```bash
sudo ./third_party/OpenNI_2.3.0.86_*/rules/install.sh
```

## 构建

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j"$(nproc)"
```

自定义 Astra 驱动由 CMake 的 `ExternalProject` 自动从源码构建，
产物为 `build/OpenNI2/Drivers/libAstraDriver.so`，无需手动处理。

## 运行

```bash
./scripts/run.sh
```

或手动指定环境：

```bash
export ASTRA_DEPTH_FORMAT=ps          # 旧固件 (5.8) 使用 PrimeSense 深度格式
export MALLOC_CHECK_=0                # 规避 OpenNI2 beta 的 malloc 问题
export LD_LIBRARY_PATH="$PWD/build:${LD_LIBRARY_PATH:-}"
export OPENNI2_DRIVERS_PATH="$PWD/build/OpenNI2/Drivers"
./build/astra_capture
```

输出：

| 文件 | 内容 |
|------|------|
| `output/depth_<W>x<H>.pgm` | 16-bit 深度图，单位 mm |
| `output/color_<W>x<H>.ppm` | RGB 彩色图 |

## 维护说明

为保证仓库精简，以下内容不会被提交（见 [.gitignore](.gitignore)）：
`build/` 等构建目录、`output/` 中采集到的图像，以及 OpenNI SDK 的
`samples/`、`tools/` 与各类 `.zip` / `.tar.gz` 原始压缩包。
这些均可从上游重新获取或重新构建，不属于源码内容。
