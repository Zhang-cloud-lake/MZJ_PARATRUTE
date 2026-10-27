# MZJ_PARATRUTE：ROS 2 翼伞工程

Ubuntu 22.04 / ROS 2 Humble，C++17 + Python 3。第一版是可运行的翼伞运动学与通信验证基线，参数不是实机标定结果。

## 已实现与边界

| 模块 | 当前实现 |
|---|---|
| 运动学仿真 | 空速、持续下降、水平恒定风、左右拉绳影响；触地后停止积分 |
| 轨迹规划 | 搜索“恒定转弯 + 配平滑翔”候选，预测落点，检查圆柱障碍物；2 Hz 重规划 |
| 控制 | 50 Hz 按轨迹时间进行转弯前馈和航向误差反馈 |
| 状态接口 | 仿真理想里程计适配和 TF；不包含 GNSS/IMU 融合 |
| 视觉 | 10 Hz 合成测试图、绿色标记检测、可选本地 Ultralytics YOLO 模型 |
| MCU | 示例串口帧、CRC、拆包、序号回执、超时检测；默认 dry-run |
| 系统管理 | 状态与命令有效性检查、使能服务、配平回退 |
| 可视化 | URDF、RViz 配置、预测轨迹和里程计 |

尚未实现：真实相机驱动/标定、语义结果三维投影与着陆区评估、融合定位、复杂地形地图、
完整气动模型、盘旋消高/拉平着陆、飞行任务状态机、Lifecycle 管理、硬件在环动力学反馈、
真实 MCU 固件和实机标定。YOLO 分支需要用户提供模型及兼容依赖，默认不下载权重。

视觉演示与导航闭环独立：识别到绿色标记不表示可着陆，也不会自动修改目标。
规划失败表示有限候选集合未找到满足条件的轨迹，不证明目标在物理上不可达。

## 构建和运行

在只加载 ROS 2 Humble 的终端中：

```bash
cd /home/zhangsan/TestProjects/MZJ_PARATRUTE
bash scripts/build.sh
source install/setup.bash
ros2 launch mzj_bringup system.launch.py
```

依赖由各包 package.xml 声明。新环境可在安装 Humble、colcon、rosdep 后运行：

```bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
```

默认从局部地面上方 25 m 开始，以 5 m/s 空速和 1 m/s 下降率飞行，地面目标为
odom 坐标系下 (120, 30, 0)。约 25 秒后落地；重新启动仿真可复位。
全部使用系统时钟，仿真约按墙钟 1 倍运行，不发布 /clock，不支持暂停/倍速；
负载过高时积分追赶被限制，不能当作硬实时仿真。

```bash
# 图形环境下显示 RViz
ros2 launch mzj_bringup system.launch.py rviz:=true

# 不启用视觉演示
ros2 launch mzj_bringup system.launch.py perception:=false

# 覆盖参数文件；规划器与仿真器模型参数需一致
ros2 launch mzj_bringup system.launch.py config:=/absolute/path/system.yaml

# 使用已准备好的本地模型，需另行安装兼容的 ultralytics / torch
ros2 launch mzj_bringup system.launch.py perception_backend:=yolo weights:=/absolute/path/model.pt
```

## 操作

```bash
# 修改地面目标；必须在 odom 下且 z=0
ros2 topic pub --once /mission/goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: odom}, pose: {position: {x: 120.0, y: 30.0, z: 0.0}, orientation: {w: 1.0}}}"

# 禁止轨迹控制，输出配平指令；翼伞仍会前飞和下降
ros2 service call /system/enable std_srvs/srv/SetBool "{data: false}"

ros2 service call /system/enable std_srvs/srv/SetBool "{data: true}"
ros2 topic echo /system/status
ros2 topic echo /perception/detections
ros2 topic echo /mcu/status
```

## 测试

```bash
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
python3 scripts/smoke_test.py
python3 scripts/serial_integration_test.py
```

单元测试覆盖解析错误、CRC、指令超时、运动学和规划约束。
smoke_test 使用独立 ROS domain，启动完整系统，检查着陆误差、视觉输出、协议输出和使能门控。
serial_integration_test 使用伪终端模拟 MCU，不连接物理设备。
运行这些集成测试需要本机 DDS 通信权限。

## 实机接入

`mode:=real` 不启动仿真器、理想定位适配器或合成相机，系统控制默认禁用，
通信默认仍是 dry-run。外部定位器必须发布 state/odom 及 odom → base_link TF。
实际相机发布 camera/image_raw；URDF 中传感器外参必须替换为实测值。

仅在 MCU 实现本项目示例协议后，才可使用
`mode:=real hardware_output:=true serial_device:=/dev/...` 启用串口。
这不是已有 MCU 的协议；不要直接将归一化指令视作实机舵机 PWM。
真实飞行前必须替换并验证模型、执行器标定、断链策略和飞行控制逻辑。

更多说明见 [架构](docs/architecture.md) 和 [MCU 示例协议](docs/mcu_protocol.md)。
