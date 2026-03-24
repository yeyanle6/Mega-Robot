# map_repair_qt

一个用于手动修复 2D SLAM 地图的 PyQt5 小工具，目标是像画板一样直接编辑 ROS 占据栅格地图。

## 设计目标

- 加载 ROS 地图 `yaml + pgm/png`
- 用固定网格显示每个 occupancy cell
- 支持三种编辑值
  - 黑色: 障碍物
  - 白色: 可通行
  - 灰色: 未知
- 支持笔刷、缩放、平移、撤销、重做
- 保存修复后的地图图像和对应 YAML

## 建议的软件结构

- `scripts/map_repair_qt.py`
  - 启动入口，兼容源码运行和 `ros2 run`
- `map_repair_qt/document.py`
  - 地图数据模型
  - 负责 yaml/image 读取、保存、像素修改、撤销重做
- `map_repair_qt/canvas.py`
  - 可视化编辑画布
  - 负责网格绘制、缩放平移、鼠标涂刷
- `map_repair_qt/main_window.py`
  - 主窗口、菜单、工具栏、状态栏

## 交互方案

- 左键: 涂刷当前模式
- 中键拖拽: 平移视图
- 滚轮: 缩放
- `1 / 2 / 3`: 切换黑 / 白 / 灰
- `Ctrl+Z / Ctrl+Y`: 撤销 / 重做
- `G`: 显示或隐藏网格
- `F`: 适配窗口

## 适合后续继续扩展的功能

- 矩形填充、直线工具、橡皮擦
- 连通域删除小噪点
- 与 `nav_msgs/OccupancyGrid` 实时互转
- 叠加激光点云或轨迹用于辅助修图
- 差异高亮和修订记录导出

## 运行

快速启动:

```bash
bash src/map_repair_qt/run_map_repair_qt.sh
```

源码直接运行:

```bash
python3 src/map_repair_qt/scripts/map_repair_qt.py
```

或构建后:

```bash
colcon build --packages-select map_repair_qt
source install/setup.bash
ros2 run map_repair_qt map_repair_qt.py
```
