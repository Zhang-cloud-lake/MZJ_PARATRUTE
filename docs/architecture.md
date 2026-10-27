# 架构与接口约定

## 包与数据流

```text
mission/goal → mzj_planning → planning/trajectory → mzj_control
                      ↑                              ↓ control/request
                 state/odom                     mzj_system
                      ↑                              ↓ actuator/command
             mzj_localization ← sensors/odom ← mzj_simulator
                                                     或
                                             mzj_mcu_bridge → MCU

demo_camera 或真实相机 → camera/image_raw → mzj_perception
                                           ↓
                                   perception/detections
```

mzj_interfaces 是消息依赖底层；mzj_model 是 ROS 无关的 C++ 头文件库。
mzj_description 提供 TF 固定部分，mzj_bringup 统一参数和启动。
当前没有空壳 mapping/drivers 包；接入真实设备、地图和定位算法时再增加实际实现。

## 运动模型

局部坐标右手系，x/y 水平、z 向上，地面 z=0；航向从 +x 向 +y 为正。
机体系 x 前、y 左、z 上。此局部 odom 不直接代表经纬度，需要另建 GNSS 原点转换。
模型采用：

- uL、uR 在 [0,1]；0 为松绳配平。
- common = min(uL,uR)。
- V = V0 × (1 - 0.3 × common)。
- sink = sink0 × (1 + 0.5 × common)。
- yaw_rate = max_yaw_rate × (uL-uR)。
- x_dot = V cos(yaw) + wind_x；y_dot = V sin(yaw) + wind_y。
- z_dot = -sink。

这些响应系数只是接口演示。无滚转、俯仰、摆动、执行器迟滞或真实拉平模型。
积分使用恒定转弯率解析更新，并将最后一个步长裁到触地时刻。
触地后状态冻结，速度归零。

规划器每 0.5 s 从最新测量重新生成轨迹；转弯率取 31 个离散值，转弯时长以
0.25 s 增长，最多 12 s，之后配平滑翔至地面。最大预测飞行时长 120 s。
碰撞检测检查线段到圆柱的距离，默认膨胀 1 m。
该搜索不覆盖全部可行轨迹，不支持复杂地形或自动盘旋消高。

轨迹时间起点为状态测量时间，控制器按当前时刻选段，插值航向并进行反馈。
轨迹需在 1.5 s 内刷新；状态需在 0.3 s 内刷新。轨迹不可行时输出配平，
而非在空中“停止”。末端落点误差阈值默认 8 m，不是精度承诺。
规划和仿真共享模型，默认闭环只能检验集成正确性，不能证明真实飞行性能。
可分别改变仿真风场/参数测试模型失配。

## 主要接口

| 话题 | 类型 | 发布者 → 订阅者 | QoS |
|---|---|---|---|
| sensors/odom | nav_msgs/Odometry | 仿真 → 理想定位适配 | Reliable，深度1 |
| state/odom | nav_msgs/Odometry | 定位 → 规划/控制/系统 | Reliable，深度1 |
| sim/truth | nav_msgs/Odometry | 仿真 → 观察工具 | Reliable，深度1 |
| mission/goal | geometry_msgs/PoseStamped | 外部 → 规划 | Reliable，深度1 |
| planning/trajectory | mzj_interfaces/Trajectory | 规划 → 控制 | Reliable，深度1 |
| planning/path | nav_msgs/Path | 规划 → RViz | Reliable，深度1 |
| control/request | mzj_interfaces/ControlCommand | 控制 → 系统 | Reliable，深度1 |
| actuator/command | mzj_interfaces/ControlCommand | 系统 → 执行后端 | Reliable，深度1 |
| camera/image_raw | sensor_msgs/Image | 相机 → 视觉 | SensorData QoS |
| perception/detections | mzj_interfaces/SemanticDetections | 视觉 → 观察工具 | Reliable，深度1 |
| mcu/tx_frame | std_msgs/UInt8MultiArray | 桥接 → 调试工具 | Reliable，深度1 |
| mcu/connected | std_msgs/Bool | 桥接 → 系统 | Reliable，深度1 |

以上数据默认 Volatile。模拟真值与测量分别发布，但当前测量无噪声。
时间戳来自 ROS 系统时钟；通信失活和本地接收超时同时使用单调时钟。
多机部署需配置时间同步和 DDS 网络；不要在同一 ROS domain 同时运行两套同名执行链。

## TF 与视觉

固定关系为 base_link → canopy_link / imu_link / camera_link → camera_optical_frame。
odom → base_link 只由定位适配器发布；真实模式由外部定位器负责。
当前没有 map 帧及 map → odom 估计。

视觉结果保留原图时间戳与相机坐标系标记，框的单位是像素。color_demo 的 confidence=1
仅表示规则匹配，不是学习模型概率。YOLO 后端输出检测框，尚无掩码或地面投影。
需要深度/地形、CameraInfo、时间匹配 TF 后才能构建候选着陆区域。

## 进程与部署

规划、控制、仿真、视觉、定位、系统、串口桥接分别运行。
视觉接收线程只保存最新帧，后台线程执行推理，避免旧图排队。
桥接使用非阻塞串口，短写保留剩余字节，写入阻塞过久会关闭连接。
尚未使用 Lifecycle、组件容器或 ros2_control；这些不影响当前闭环接口。

实机模式须外部定位有效、MCU 回执正常、人工通过使能服务开启控制，才转发控制请求。
停止主机进程不能保证发送末条指令，MCU 必须自行实现看门狗。
