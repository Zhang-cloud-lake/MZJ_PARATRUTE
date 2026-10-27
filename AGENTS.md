# MZJ_PARATRUTE 项目约定

- ROS 2 Humble / Ubuntu 22.04，翼伞平台；所有构建使用 colcon。
- 工作空间为 /home/zhangsan/TestProjects/MZJ_PARATRUTE。
- C++17；Python包为 ament_python，控制/规划/仿真为 ament_cmake。
- 默认使用 sim 模式和 MCU dry-run；开发测试不得连接物理执行器。
- 归一化左右拉绳范围 [0,1]，零指令表示配平滑翔，不表示停止。
- 状态、目标、轨迹均使用 odom，地面 z=0；姿态/角速度为弧度。
- 控制量 header.frame_id 为 base_link，须验证时间戳、有效期和有限数值。
- 不把图像检测框直接当作世界坐标着陆点；不将当前气动系数描述为实机标定。
- 运行前 source install/setup.bash；接口修改后重新构建。
- 验证：colcon test、colcon test-result --verbose，以及 scripts 下两个集成测试。
- 修改包布局、依赖或启动行为时同步 README、docs 和 /home/zhangsan/AGENTS.md。
