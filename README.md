# Orbbec Astra Pro 采集示例（OpenNI2）

基于 OpenNI2 读取 Orbbec Astra Pro 的深度图与彩色图的最小示例。
原理与分阶段规划见 [PLAN.md](PLAN.md)。

## 目录结构

```
orbbec/
├── CMakeLists.txt                              # 构建入口
├── src/
│   ├── main.cpp                                # 采集：一帧模式 + 序列录制模式
│   ├── camera_openni.h / .cpp                  # CameraOpenNI 封装
├── python/                                     # 风团高度分析（uv 项目，见 python/README.md）
├── docs/agents/                                # 工程技能的仓库配置
├── scripts/
│   ├── run.sh                                  # 设置运行环境并启动程序
│   ├── setup_env.sh                            # 导出 OpenNI2 环境变量（可选）
│   └── view_in_blender.py                      # 在 Blender 里打开 PLY（打开就能看）
├── output/                                     # 采集结果输出目录（含录制的序列）
└── third_party/
    ├── openni2_runtime/                        # 精简 OpenNI2 运行时 + 头文件（构建依赖）
    ├── bugparty-openni2-astra-pro-driver-*/    # 开源 Astra Pro 驱动源码（构建依赖）
    └── OpenNI_2.3.0.86_*/rules/                # udev 规则（一次性系统安装）
```

## output/ 里有什么

```
output/
├── depth_640x480.pgm        单帧采集（C++ 一帧模式的输出）
├── seq_indoor/              录制的原始序列 —— 120 帧，是唯一不可再生的东西
└── scene_indoor/            2.5D 建模产物（可从 seq_indoor 重新生成）
│   ├── points.ply             点云（带顶点色）
│   ├── mesh.ply               网格（顶点色 + 法向）
│   ├── overview.png           四联图：深度 / 浮雕 / 掩码 / 逐行剖面
│   └── scene.json             覆盖率、内参、包围盒
└── preview/                 深度图的“能直接看”版本（可从 seq_indoor 重新生成）
    ├── sequence.png / frame_0000.png / fused.png
    └── fused.pgm            16-bit 累积结果，可继续下游
```

`output/` 整个在 `.gitignore` 里（`/output/*`）。除了 `seq_indoor/`，其余都可由
上面两条命令重生。所以在清理磁盘时：**只删生成物，别删录制**。


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

### 录制深度序列

供 `python/` 侧做多帧累积分析使用（深度专用，不含彩色）：

```bash
./build/astra_capture --record 300 --out output/seq_demo --working-distance-mm 600
```

写出 `frame_%04d.pgm` 与 `metadata.json`。文件名零填充不是美观问题——
Python 侧按字典序读入，否则 `frame_10` 会排到 `frame_2` 前面。

## 风团 2.5D 高度图分析（调研）

基于深度图像测量过敏原风团隆起量的**可行性调研**，见
[issue #1](https://github.com/transmit-bug/orbbec-openni2-capture/issues/1)
与 [python/README.md](python/README.md)。

分工是：**C++ 负责采集，Python 负责分析，语言边界正好落在帧来源（Seam 2）上**，
通过磁盘上的 PGM 序列交接——没有 pybind11、没有 FFI、没有构建耦合。

```bash
cd python
uv sync
uv run pytest
uv run wheal synthetic --out /tmp/seq --frames 64   # 无相机也能跑通全链路
uv run wheal analyze /tmp/seq --out /tmp/report
```

拿到真实录制后的两条主命令：

```bash
uv run wheal preview ../output/seq_indoor --out ../output/preview --fuse
uv run wheal scene   ../output/seq_indoor --out ../output/scene_indoor --color
```

### 看产物：两个必须知道的坑

产物打开是黑的是**正常现象**，不是文件坏了，也不是流程没跑完——两个坑都在
“少做了一步归一化”上：

1. **PGM 深度图**里存的是毫米原值（460..630），而 header 声明 `maxval=65535`。
   任何看图程序只显示高字节，`460 >> 8 == 1` → 整幅图落在 0..2/255。
   用 `uv run wheal preview` 重生一份归一化过、带色标的 PNG。
2. **PLY** 没有材质，在带光照的渲染器里就是黑的。用
   `scripts/view_in_blender.py` 打开（已配好着色模式、材质和相机）：

```bash
blender --factory-startup --python scripts/view_in_blender.py -- \
    output/scene_indoor/mesh.ply
```

细节、以及 **Blender 5.2.2 二进制 PLY 读取器本身有 bug**（所以默认导出 ascii）
的记录，见 [python/README.md](python/README.md)。

### ⚠️ 当前最优先的未解决问题

采集到的数据在**上半幅存在确定性的行相关伪影**（相邻行差 p90 41 mm，而下半幅
仅 1.5 mm；同一录制前后半段相关系数 0.87，时间累积消不掉）。现有证据指向驱动的
S2D 标定参数而非硬件，但还未定论。**在这条结论落定前，上半幅的深度不要用于
任何测量。** 完整的判据、数据与下一步的定性实验见
[python/README.md](python/README.md) 的“上半幅的确定性行相关伪影”。

## 维护说明

为保证仓库精简，以下内容不会被提交（见 [.gitignore](.gitignore)）：
`build/` 等构建目录、`output/` 中采集到的图像，以及 OpenNI SDK 的
`samples/`、`tools/` 与各类 `.zip` / `.tar.gz` 原始压缩包。
这些均可从上游重新获取或重新构建，不属于源码内容。

**`output/` 里只有 `seq_indoor/`（原始录制）是不可再生的**，其余都是生成物。
清理磁盘时只删生成物：

```bash
cd python
rm -rf ../output/preview ../output/scene_indoor
uv run wheal preview ../output/seq_indoor --out ../output/preview --fuse
uv run wheal scene   ../output/seq_indoor --out ../output/scene_indoor --color
```
