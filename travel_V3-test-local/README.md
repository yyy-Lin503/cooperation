# TravelMate

毫米波雷达与视觉辅助的随行工具车系统。雷达负责点云、位置与目标跟踪，相机提供人物检测、人工关联及可选的视觉连续性保护，ROS 2 负责里程计与底盘速度控制。主工作区只展示“雷视协同”。两路通过人工核对关联，不提供自动身份匹配或视觉深度定位。

## 构建与启动

Linux 使用 C++17、CMake 3.18+。GUI 需要 Qt 5.12+，视觉需要 OpenCV 4.5+（DNN）；Jetson CUDA 构建需要 CUDA/cuFFT。ROS 运行需要 ROS 2 Humble、rclpy、sysv_ipc 和底盘驱动 turn_on_wheeltec_robot。

在项目根目录运行，无硬件 CPU 构建及测试：

```bash
bash scripts/build/build.sh cpu
# WSL GUI 构建及测试：
bash scripts/build/build.sh wsl
```

Windows PowerShell 启动 WSLg 界面：

```powershell
.\scripts\run\run_gui_wsl.ps1
# 其他发行版：-Distribution Ubuntu-24.04
# WSLg COPY MODE、窗口不可见时：-RepairWslg
# 使用 Windows 本机摄像头（默认编号 0）：
.\scripts\run\run_gui_wsl.ps1 -LocalCamera
# 指定其他摄像头：追加 -CameraIndex 1
```

本机摄像头模式需要 Windows 的 `python.exe` 已安装 OpenCV，并已启用摄像头与桌面应用访问权限。
启动器运行 Windows 采集桥接，经 TCP 向 WSL 按需发送最新 JPEG；退出时清理桥接进程。
若 WSL 无法连接桥接，请检查 Windows 防火墙是否允许该 Python 进程通过 WSL 网络接口访问。
在“视觉”页选择存在的 COCO YOLO11 detection ONNX 模型，点击“开启摄像头”即可观察原有人物框和 V 编号，无需雷达。
若数据根下存在 `models/yolo11n.onnx`（默认 `var/models/yolo11n.onnx`），界面会优先采用它；`RADAR_CAMERA_MODEL` 仍可覆盖。
首次性能测试可取消“录制相机供回放”；模型推理仍使用现有配置，接收时间标记为 `receive_estimate`。
使用本机模式时同样支持 `-Distribution` 和 `-RepairWslg`，原有 Linux 摄像头启动方式保持不变。

Jetson 编译和启动：

```bash
bash scripts/build/build_ros2.sh
bash scripts/build/build.sh jetson
bash scripts/run/run_gui.sh
```

输出统一位于 build/cpu、build/wsl、build/jetson。CMake 3.20+ 也可使用 `cmake --preset cpu`、`cmake --build --preset cpu`、`ctest --preset cpu`，将 cpu 替换为其他配置即可。脚本支持 RADAR_BUILD_JOBS 和额外 -D 参数。

保留 CMake 3.18 的直接入口：

```bash
cmake -S . -B build/cpu -DBUILD_RADAR_APP=OFF -DRADAR_ENABLE_CUDA=OFF -DRADAR_ENABLE_CAMERA=OFF
cmake --build build/cpu --parallel 2
ctest --test-dir build/cpu --output-on-failure
```

ROS 包源码位于 ros2_ws/src/my_radar_follower，产物位于 ros2_ws/build、install、log。迁移项目后重新构建 overlay，来源校验会拒绝其他工程的包和安装目录。设备部署与验收见 [部署与验收](docs/部署与验收.md)。

## 操作

1. 连接 IWR6843ISK / DCA1000，加载 config/radar/ 下对应配置，开始采集。
2. 点击雷达候选预选，核对后确认锁定。
3. 开启摄像头，点击对应人物框，人工关联视觉轨迹。P/V/T 分别表示雷达轨迹、视觉轨迹和任务编号。
4. 在“ROS/底盘”页核对驱动、话题和速度参数，点击“开始追踪”。未采集或未锁定时停车等待；相机选人不会直接启动底盘。
5. “停止追踪”停止随行；“关闭底盘与 ROS”清理由本界面管理的进程。停止采集、进入相机回放或退出时同步停止随行。

目标、配置、视觉、ROS/底盘与日志通过侧栏访问。严格视觉保护默认关闭，启用后的许可和失效行为沿用原实现。

相机需另行准备 COCO YOLO11 detection ONNX，输入 1×3×640×640、输出 1×84×N。通过视觉设置或 RADAR_CAMERA_DEVICE、RADAR_CAMERA_MODEL 指定设备和模型。模型不随源码分发。

## 运行路径与数据迁移

默认数据根为 `<project>/var/`。RADAR_DATA_ROOT 可覆盖数据根，RADAR_TRACK_DIR 可单独覆盖跟踪 TXT 目录；相对路径均相对项目根解析，GUI 和启动脚本保留用户覆盖值。

| 内容 | 默认路径 |
| --- | --- |
| 本地 GUI 设置 | var/config/gui_runtime.ini |
| 雷达 BIN 与 .bin.frames.csv | var/recordings/radar/ |
| 相机 MJPG、JSONL、事件及 summary | var/recordings/camera/ |
| 点云 / RDM | var/pointcloud/ |
| 控制 TXT、原子 metadata、runtime 会话 | var/tracking/txt/ |
| 轨迹 CSV | var/tracking/csv/ |
| 诊断及分析报告 | var/diagnostics/、var/reports/ |

GUI 首次启动只在新设置不存在时复制旧根目录 gui_runtime.ini；已有设置不被示例覆盖。示例位于 config/gui_runtime.example.ini。本地设备参数、录制和模型不进入 Git。

旧数据目录可先预览，再迁移：

```bash
python3 -B scripts/runtime/migrate_data.py
python3 -B scripts/runtime/migrate_data.py --apply
```

迁移先校验全部冲突，再复制并核验哈希，全部完成后移除源文件。新旧配置不同时保留新设置，并将旧值保存为 gui_runtime.legacy.ini。雷达、轨迹和相机会话成套保留，清单写入数据根的 reports/data-migration.json。请在停止应用和录制后迁移。

## 离线复现与工具

雷达离线处理和跟踪使用独立工具；相机“回放记录…”保留暂停、逐帧、拖动和变速，使用录制时的检测结果。

```bash
# 输出当前跟踪结果，可选 --points-output、--candidates-output、--clusters-output
build/cpu/replay_radar capture.bin config/radar/IWR6843_260714-15m-10Hz-9.8cm.cfg motion.csv selections.csv frames.csv
# 选择事件 CSV 的前三列为 after_frame,x,y；无事件时仍需表头
build/cpu/bin2pointcloud capture.bin config/radar/IWR6843_260714-15m-10Hz-9.8cm.cfg --diagnostic-csv motion.csv --output points.csv
python3 tools/analysis/audit_recordings.py --data-root var --output var/reports/audit
python3 tools/analysis/summarize_replays.py --frames frames.csv --output var/reports/summary.json
python3 tools/analysis/plot_replays.py --frames frames.csv --points replay_points.csv --output-dir var/reports/figures
python3 tools/analysis/validate_converter.py --manifest var/reports/audit/recordings.json --converter build/cpu/bin2pointcloud
python3 tools/analysis/export_full_recording_videos.py --manifest var/reports/audit/recordings.json --replay-dir var/reports/replay --output-dir var/reports/videos
```

绘图需要 matplotlib 或 Pillow；完整视频导出需要 Pillow 和 FFmpeg（通过 PATH 或 --ffmpeg 指定，也可安装 imageio-ffmpeg）。工具读取显式文件或 manifest，不再查找相邻历史工程。完整视频的回放输入名称为 `<BIN stem>_frames.csv`、`<BIN stem>_points.csv`。

`tools/diagnostics/` 提供相机 probe、DCA 以太网及 odom 检查。probe 显式关闭录制，设备验证需真实摄像头及模型。

## 目录与回退

| 路径 | 内容 |
| --- | --- |
| src/app、src/ui | 启动、协同界面和共享绘图控件 |
| src/radar、src/tracking、src/vision | 雷达设备/采集/DSP、当前跟踪和视觉实现 |
| src/common、src/replay | 路径、时间/运动协议、帧索引和离线里程计 |
| cmake、scripts/build、scripts/run、scripts/runtime | 构建定义、启动、ROS 与迁移支持 |
| tools/replay、tools/analysis、tools/diagnostics | 转换回放、分析和诊断工具 |
| tests | 分类回归测试及 fixtures |
| docs | 数据协议、原审阅报告、部署与实施记录 |

Qt 表单位于 src/ui/forms，AUTOUIC 在构建目录生成头文件。历史版本保留在本地 Git 标签 baseline-before-cleanup，恢复和文件处置说明见 [实施记录](docs/实施记录.md)。V8 标记、控制协议和数据字段保持原样。
