# yahboomcar_nav

Yahboomcar 的 **建图 (SLAM) 与导航 (Nav2)** 总包，集成多种 SLAM 后端与规划器。

## 功能概述

### 建图 (Mapping)
- `map_gmapping_launch.py` / `map_gmapping_a1_launch.py` / `map_gmapping_ld19_launch.py` / `map_gmapping_4ros_s2_launch.py`：GMapping 建图（适配不同雷达）。
- `map_cartographer_launch.py` + `params/lds_2d.lua`：Cartographer 2D 建图。
- `map_rtabmap_launch.py`、`rtabmap_*_launch.py`：RTAB-Map 视觉 / 激光 SLAM。
- `save_map_launch.py`：保存生成的地图。
- `occupancy_grid_launch.py`：栅格地图生成。

### 导航 (Navigation)
- `navigation_dwa_launch.py` + `params/dwa_nav_params.yaml`：Nav2 + DWA 规划器。
- `navigation_teb_launch.py` + `params/teb_nav_params.yaml`：Nav2 + TEB 规划器。
- `navigation_rtabmap_launch.py` + `params/rtabmap_nav_params.yaml`：基于 RTAB-Map 的导航。
- `display_nav_launch.py` / `display_map_launch.py`：可视化建图 / 导航。

### 工具节点
- `scan_filter.py`：对 `/scan` 做过滤（裁剪角度、去噪等）。

## 目录结构

```
yahboomcar_nav/
├── yahboomcar_nav/         # Python 节点
│   └── scan_filter.py
├── launch/                 # 大量 SLAM / Nav2 启动文件
├── params/                 # DWA / TEB / RTAB-Map / Cartographer 参数
├── maps/                   # 已保存的地图
├── rviz/                   # RViz 配置
└── resource/
```

## 依赖

- `rclpy`, `std_msgs`, `geometry_msgs`
- Nav2 栈：`nav2_bringup`, `nav2_*`
- SLAM 后端（按需安装）：`slam_gmapping`、`cartographer_ros`、`rtabmap_ros`

## 使用

```bash
# 建图（GMapping，配 A1 雷达）
ros2 launch yahboomcar_nav map_gmapping_a1_launch.py

# 保存地图
ros2 launch yahboomcar_nav save_map_launch.py

# 导航 (DWA)
ros2 launch yahboomcar_nav navigation_dwa_launch.py

# 扫描过滤
ros2 run yahboomcar_nav scan_filter
```
