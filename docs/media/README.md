# 展示图片

README 中的图片全部由本仓库已有场景配置与实验记录绘制。

| 图片 | 内容 | 数据来源 |
|---|---|---|
| [world-layout.png](world-layout.png) | 障碍物三维几何与俯视布局 | [实验 world.sdf](../results/five_planners_2026-10-28/world.sdf) |
| [planner-comparison.png](planner-comparison.png) | 完整任务成功次数、成功任务墙钟时间均值 | [summary.csv](../results/five_planners_2026-10-28/summary.csv) |

场景图不是 Gazebo 截图，机器人本体未绘制；蓝点为基座位置。它使用 SDF 的模型、link、visual 位姿变换与 box 尺寸绘制，保留模型配色。

实验图对应五算法、每算法三个种子的同一批实验，共 15 次完整任务尝试。0/3 表示没有完整成功任务，不代表算法理论上无法解决该问题。耗时只统计完整成功的任务，因此各组时间样本数不同；不把失败任务记作零耗时。此图不包含后续桌面连续搬运实验。

当前没有附加录屏；图片不表示存在未采集的视频或新的实验结果。
