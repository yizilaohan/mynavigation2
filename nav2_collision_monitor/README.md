# Nav2 Collision Monitor

## 本地源码接口（2026-11-16）

当前源码同时提供 Nav2 速度过滤节点与独立状态工具。use_collision_monitor 在项目入口表示状态检测链总开关，use_collision_velocity_gate 才表示速度门；排除区和速度分档多边形属于本包 C++ 输入处理，UInt8 状态门控由规控／控制／平滑链消费。

- 构建声明：[package.xml](package.xml) 、[CMakeLists.txt](CMakeLists.txt) 。
- CMake 声明的运行目标：`collision_monitor`；具体安装和启动条件见构建文件。
- 安装脚本：[car_op_status_sim.py](tools/car_op_status_sim.py) 、[car_op_zones.py](tools/car_op_zones.py) 、[obstacle_status_translator.py](tools/obstacle_status_translator.py) 。脚本安装不等于入口自动执行。
- 实现定位：[source.cpp](src/source.cpp) 、[exclusion_zone.cpp](src/exclusion_zone.cpp) 、[obstacle_status_translator.py](tools/obstacle_status_translator.py) 。
- 启动资源：[collision_monitor_node.launch.py](launch/collision_monitor_node.launch.py) 、[obstacle_status.launch.py](launch/obstacle_status.launch.py) ；启动参数由对应文件声明。

## 当前项目用法

依据 LaserScan、PointCloud2、Range 等传感器与配置区域，对输入速度实施停车、减速或接近保护。

包类型：速度过滤节点。包名和构建依赖见 [package.xml](package.xml) ，安装目标见 [CMakeLists.txt](CMakeLists.txt) 。

主要目录：[src/](src/) 、[include/](include/) 、[launch/](launch/) 、[params/](params/) 、[test/](test/) 。

直接依赖（清单节选，完整约束见 manifest）：`geometry_msgs`、`libpcl-all`、`libpcl-all-dev`、`nav2_common`、`nav2_costmap_2d`、`nav2_util`、`rclcpp`、`rclcpp_components`、`sensor_msgs`、`tf2`、`tf2_geometry_msgs`、`tf2_ros`。

启动资源：[collision_monitor_node.launch.py](launch/collision_monitor_node.launch.py) 。执行前先加载工作空间环境，具体依赖由所选入口决定。

### 本项目接入说明

本包依据传感器过滤速度，与外部 `/collision_state` UInt8 门控独立。`regulated_modules.launch.py`、`myworld_bringup/entry.launch.py` 和 [launch/obstacle_status.launch.py](launch/obstacle_status.launch.py) 现在是同一套语义：`use_collision_monitor` 是碰撞检测模块（状态枚举链）总开关，Nav2 速度门由独立的 `use_collision_velocity_gate` 控制，两者均默认 false，需显式启用。区域可视化不会产生碰撞判断。碰撞区默认使用 `velocity_polygon`（按速度档切换），自研 box / 动态订阅框仍保留为 `type: "polygon"`。

避障状态链（`car_op_zones.py`、`obstacle_status_translator.py`、`car_op_status_sim.py`）自 2026-10 起也放在本包，默认行为是只发布 `/collision_state` 与 `/obstacle_status_int`，不发布 `/cmd_vel`。ooii: 2026-11 起状态输入由 `/car_op_status`（`std_msgs/Int32`）换成 `/robot_move_state`（`byd_custom_msgs/RobotMoveState`，`uint8 robot_move_state` 九值 0–8，QoS RELIABLE），话题名可用 launch 参数 `car_op_status_topic` 覆盖；九值到设计状态的映射见 [doc/OBSTACLE_STATUS.md](doc/OBSTACLE_STATUS.md)。`use_obstacle_status`（默认 true）只控制 translator 是否输出枚举。实车既可以跑 `regulated_modules.launch.py`，也可以单独跑 [launch/obstacle_status.launch.py](launch/obstacle_status.launch.py) ，但不要同时开。说明见 [doc/OBSTACLE_STATUS.md](doc/OBSTACLE_STATUS.md) 。

构建命令统一从 `nav2_ws` 执行，详见[仓库构建与包索引](../README.md) ；不在包源码目录运行 colcon。本文依据当前源码描述职责，包存在、可构建或包含测试文件均不代表完成运行验收。

## 当前扩展与边界

- `Polygon`、`Circle` 和 `VelocityPolygon` 分别提供静态／动态多边形、圆形及速度分档区域，构造选择见 [collision_monitor_node.cpp](src/collision_monitor_node.cpp) 。配置 `velocity_polygon` 不表示状态翻译器也采用同一套 C++ 几何判定。
- 传感器 `Source` 可配置排除区，运行时服务由 [source.cpp](src/source.cpp) 按 `<source_name>/add_exclusion_zone`、`<source_name>/remove_exclusion_zone` 创建；当前代码只有在初始 `exclusion_zones` 列表非空时才创建服务。只生成了接口、却没有实际服务时，不能靠调用接口自动启用。
- [ExclusionZone](src/exclusion_zone.cpp) 当前处理二维点，消息保留的 `min_height/max_height` 不代表实现了高度筛选。传感器排除区用于剔除点，和 Costmap 的禁止机器人进入区域是不同语义。
- `TwistPublisher`／`TwistSubscriber` 构造代码的 `enable_stamped_cmd_vel` 回退为 true。项目三个整车 YAML 显式设 false，以匹配最终裸 `Twist` 链路；独立使用通用参数文件时须核对输入输出类型。
- 状态脚本 [obstacle_status_translator.py](tools/obstacle_status_translator.py) 的当前多边形点内判断使用射线相交 `any`，并非交点奇偶判定；已知存在圆形区域外、包围盒内角点误判停止的情形。源码保留该分支行为，不将已有常规场景通过描述为完整几何正确性验收。

## 原有说明与上游资料

以下保留上游原理、配置链接和历史性能数字；当前本地实现与项目入口以本页源码接口及上述边界为准，未在本轮复测这些性能数据。

The Collision Monitor is a node providing an additional level of robot safety.
It performs several collision avoidance related tasks using incoming data from the sensors, bypassing the costmap and trajectory planners, to monitor for and prevent potential collisions at the emergency-stop level.

This is analogous to safety sensor and hardware features; take in laser scans from a real-time certified safety scanner, detect if there is to be an imminent collision in a configurable bounding box, and either emergency-stop the certified robot controller or slow the robot to avoid such collision.
However, this node is done at the CPU level with any form of sensor.
As such, this does not provide hard real-time safety certifications, but uses the same types of techniques with the same types of data for users that do not have safety-rated laser sensors, safety-rated controllers, or wish to use any type of data input (e.g. pointclouds from depth or stereo or range sensors).

This is a useful and integral part of large heavy industrial robots, or robots moving with high velocities, around people or other dynamic agents (e.g. other robots) as a safety mechanism for high-response emergency stopping.
The costmaps / trajectory planners will handle most situations, but this is to handle obstacles that virtually appear out of no where (from the robot's perspective) or approach the robot at such high speed it needs to immediately stop to prevent collision.

![polygons.png](doc/polygons.png)

## Features

The Collision Monitor uses polygons relative the robot's base frame origin to define "zones".
Data that fall into these zones trigger an operation depending on the model being used.
A given instance of the Collision Monitor can have many zones with different models at the same time.
When multiple zones trigger at once, the most aggressive one is used (e.g. stop > slow 50% > slow 10%).

The following models of safety behaviors are employed by Collision Monitor:

* **Stop model**: Define a zone and a point threshold. If more that `N` obstacle points appear inside this area, stop the robot until the obstacles will disappear.
* **Slowdown model**: Define a zone around the robot and slow the maximum speed for a `%S` percent, if more than `N` points will appear inside the area.
* **Approach model**: Using the current robot speed, estimate the time to collision to sensor data. If the time is less than `M` seconds (0.5, 2, 5, etc...), the robot will slow such that it is now at least `M` seconds to collision. The effect here would be to keep the robot always `M` seconds from any collision.

The zones around the robot can take the following shapes:

* Arbitrary user-defined polygon relative to the robot base frame.
* Circle: is made for the best performance and could be used in the cases where the zone or robot could be approximated by round shape.
* Robot footprint polygon, which is used in the approach behavior model only. Will use the footprint topic to allow it to be dynamically adjusted over time.

The data may be obtained from different data sources:

* Laser scanners (`sensor_msgs::msg::LaserScan` messages)
* PointClouds (`sensor_msgs::msg::PointCloud2` messages)
* IR/Sonars (`sensor_msgs::msg::Range` messages)

## Design

The Collision Monitor is designed to operate below Nav2 as an independent safety node.
This acts as a filter on the `cmd_vel` topic coming out of the Controller Server. If no such zone is triggered, then the Controller's `cmd_vel` is used. Else, it is scaled or set to stop as appropriate.

The following diagram is showing the high-level design of Collision Monitor module. All shapes (Polygons and Circles) are derived from base `Polygon` class, so without loss of generality we can call them as polygons. Subscribed footprint is also having the same properties as other polygons, but it is being obtained a footprint topic for the Approach Model.
![HDL.png](doc/HLD.png)

## Configuration

Detailed configuration parameters, their description and how to setup a Collision Monitor could be found at its [Configuration Guide](https://navigation.ros.org/configuration/packages/configuring-collision-monitor.html) and [Using Collision Monitor tutorial](https://navigation.ros.org/tutorials/docs/using_collision_monitor.html) pages.


## Metrics

Designed to be used in wide variety of robots (incl. moving fast) and have a high level of reliability, Collision Monitor node should operate at fast rates.
Typical one frame processing time is ~4-5ms for laser scanner (with 360 points) and ~4-20ms for PointClouds (having 24K points).
The table below represents the details of operating times for different behavior models and shapes:

| | Stop/Slowdown model, Polygon area | Stop/Slowdown model, Circle area | Approach model, Polygon footprint | Approach model, Circle footprint |
|-|-----------------------------------|----------------------------------|-----------------------------------|----------------------------------|
| LaserScan (360 points) processing time, ms  | 4.45 | 4.45 | 4.93  | 4.86  |
| PointCloud (24K points) processing time, ms | 4.94 | 4.06 | 20.67 | 10.87 |

The following notes could be made:

 * Due to sheer speed, circle shapes are preferred for the approach behavior models if you can approximately model your robot as circular.
 * More points mean lower performance. Pointclouds could be culled or filtered before the Collision Monitor to improve performance.

## 中文翻译

### Nav2 Collision Monitor

Collision Monitor 是 Nav2 的速度安全监控节点。它订阅 LaserScan、PointCloud2、Range 等传感器数据，把障碍物投影到配置的 Polygon 或 Circle 区域，再依据速度限制、减速或停车规则保护机器人。

节点支持多个传感器 Source、多个几何区域和按优先级组合的行为。常用安全动作包括 STOP 停车、SLOWDOWN 按比例减速、APPROACH 前向预测以及其他速度限制。传感器数据会在机器人坐标系中处理，并通过 cmd_vel 输入与安全后的速度输出形成独立速度门。

参数用于选择源 Topic、数据类型、坐标系、区域顶点、动作优先级、减速比例和停止时间。可启用调试多边形、源数据和状态指标。区域点越多，计算性能越低；在送入 Collision Monitor 前对点云裁剪或滤波可以降低开销。


        ros2 bag record -o ~/bags/obstacle_real \
  --qos-profile-overrides /tf_static:transient_local \
  /c200_lidar_node/scan /tf /tf_static /robot_move_state /motion_state /cmd_vel /odometry
