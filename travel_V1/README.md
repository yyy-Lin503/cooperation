# TravelMate

基于毫米波雷达和视觉辅助的随行工具车系统。雷达负责点云处理、目标跟踪与距离估计，相机提供人物检测、人工目标关联及可选的视觉连续性保护，ROS 2 负责里程计接收和底盘速度控制。

## 运行环境

- NVIDIA Jetson / Linux，C++17、CMake 3.18 及以上。
- Qt 5.12 及以上、CUDA / cuFFT、OpenCV 4.5 及以上（含 DNN 模块）。
- ROS 2 Humble、Python 3、`rclpy`、`sysv_ipc`，以及底盘驱动 `turn_on_wheeltec_robot`。
- IWR6843ISK 雷达、DCA1000 采集板及 USB 摄像头。
- 视觉功能使用 COCO YOLO11 detection ONNX 模型，输入为 `1×3×640×640`，输出为 `1×84×N`；模型需在部署设备上准备。

## 编译与启动

在项目根目录执行：

```bash
source /opt/ros/humble/setup.bash
bash scripts/build_ros2.sh
bash scripts/build.sh -DBUILD_RADAR_APP=ON -DRADAR_ENABLE_CUDA=ON -DRADAR_ENABLE_CAMERA=ON
bash scripts/run_gui.sh
```

GUI 构建目录为 `build-v8-jetson/`，ROS 工作空间安装目录为 `ros2_ws/v8-install/`。部署到新路径后应重新编译。

在“ROS/底盘”页设置底盘驱动的 `setup.bash`、启动包、启动文件、里程计话题、速度话题及速度上限，设置保存在 `gui_runtime.ini`。默认话题为 `/odom` 和 `/cmd_vel`。相机设备与模型可在界面中选择，也可通过 `RADAR_CAMERA_DEVICE` 和 `RADAR_CAMERA_MODEL` 指定。

## 操作流程

1. 连接雷达，加载 `config/` 中的配置文件，开始实时采集。
2. 在点云图或目标列表中预选人员对应的雷达候选，核对后确认锁定。
3. 开启摄像头，点击对应人物框，将视觉轨迹与当前雷达目标关联。未锁定时可先保存视觉预选，再选择雷达目标。
4. 核对“ROS/底盘”页参数，点击“开始追踪”。GUI 管理底盘、里程计接收和随行控制进程；未采集或未锁定时保持停车等待。
5. 点击“停止追踪”结束随行控制；“关闭底盘与 ROS”关闭由 GUI 启动的相关进程。停止采集、切换离线模式或退出界面时同步停止随行。

P 为雷达轨迹编号，V 为视觉轨迹编号，T 为选定任务编号。相机与雷达通过人工核对建立关联。视觉连续性保护默认关闭，可在视觉设置中启用。目标丢失、关联歧义及数据过期时的运动许可由跟踪状态和控制协议共同决定。

## 录制与回放

雷达录制使用 BIN 文件及配套的 `.bin.frames.csv` 帧索引，移动录制时应保留配套索引。GUI 提供雷达离线处理与跟踪入口。

相机录制开关位于视觉页，记录保存在 `camera_recordings/`，包含 MJPEG 图像、帧索引和事件记录。“回放记录…”支持暂停、逐帧、拖动与变速播放，检测框使用录制时保存的结果。

## 测试

无需 Qt、CUDA 或 ROS 的 CPU 构建与测试：

```bash
cmake -S . -B build-cpu -DBUILD_RADAR_APP=OFF -DRADAR_ENABLE_CUDA=OFF -DRADAR_ENABLE_CAMERA=OFF
cmake --build build-cpu --parallel 2
ctest --test-dir build-cpu --output-on-failure
```

Qt、CUDA、相机和 Linux IPC 相关测试需要在相应环境中运行。底盘闭环运行需连接实际设备。

## 目录

| 路径 | 内容 |
| --- | --- |
| 根目录 `.cpp` / `.hpp` / `.cu` | 信号处理、跟踪、采集与 GUI |
| `config/` | 雷达配置 |
| `功能包/my_radar_follower/` | ROS 2 功能包 |
| `scripts/` | 编译、启动与检查脚本 |
| `tools/` | 数据转换、离线回放与分析工具 |
| `tests/` | 回归测试和测试数据 |
| `reference/` | 离线版本对比所需的跟踪源码 |
| `docs/数据协议.md` | 坐标、里程计与跟踪输出协议 |

运行时生成点云、轨迹、诊断日志和录制目录。
