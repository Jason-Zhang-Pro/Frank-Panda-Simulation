# 三算法统一 Gazebo 对比：30 秒预算与优化抓放时间

本组重新运行 RRT、RRT-Connect、RRT*，每种算法三个种子，共 **9 次全新有效任务**。全部使用当前 `bounded` 抓放时间分配，不复用此前 RRT-Connect 的三次时间优化验证，也不混入旧保守时间组。

## 统一条件

同一机器人、场景、PD 配置、shortcut、B 样条和碰撞检查；七轴速度比例 0.3、参考加速度上限 0.9 rad/s²。抓放段采用 IK 直线路径与约束 B 样条；时间分配以实际控制 Hermite 插值的 Bernstein 包络复检，最多较保守估计提速两倍。全局接近、搬运、归位的时间分配未在本次调整。

每次全局搜索预算 30 秒，整套任务上限 240 秒；按种子 1/2/3 交错运行三种算法。每次新建隔离的无 GUI Gazebo 世界，真实模拟接触、夹持与方块运动。完整成功只认 `pick_place_succeeded`，要求抓取、放置、松开及归位七段到位。失败保留，不重新抽取成功样本。

启动前有一次控制器列表服务响应超时，任务尚未提交，单独保留日志。随后为只读列表查询增加最多三次有限重试，重试期间保持暂停；九次有效任务全部使用修正后的同一版本。激活和步进操作没有隐式重试。

环境为 i7-14650HX、ROS 2 Humble、OMPL 1.7、MoveIt Core 2.5.10、FCL 0.7。原有 GUI 实例保留；未锁定 CPU 频率，物理初始化与接触并非确定性回放，同种子不保证相同状态。此处是小样本场景结果，不能当作通用算法成功率。

## 完整任务与携物规划

| 算法 | 携物轨迹生成成功 | 完整任务成功 | 成功任务墙钟均值 / s |
|---|---:|---:|---:|
| RRT | 1/3 | 1/3 | 102.77 |
| RRT-Connect | 3/3 | 2/3 | 102.29 |
| RRT* | 0/3 | 0/3 | — |

携物列要求搜索及后处理轨迹复检通过；完整任务列还要求实际执行完成。成功任务时间仅对成功样本求均值，失败任务前缀时间不参与效率排名。

## 结果分析

本场景中 RRT-Connect 的三次携物轨迹均通过复检，搜索约 0.016～0.065 秒；两次完整成功，另一次在放置、松爪之后的撤离 IK 失败。它仍是当前默认方案，但不能据此宣称任务已稳定完成。

RRT 种子 1 的携物搜索约 7.13 秒并完整执行；种子 2 搜索找到解后，最终轨迹因 link5 与上方横梁碰撞被拒绝；种子 3 未找到精确解。RRT* 的种子 1、3 未找到精确解，种子 2 则在最终轨迹碰撞复检失败。这些是不同失败来源，不能全部称为搜索超时。

与此前仅对 RRT-Connect 的三次时间优化验证相比，本轮仍出现撤离 IK 失败，说明接触与物理状态变化会影响后续可达性。后续优先排查撤离起点和 IK 鲁棒性，以及平滑后碰撞失败的处理策略，再扩大样本。

## 同类初始接近段

以下均为每种算法三个初始接近请求的平均值。长度为七维关节空间欧氏累计长度，单位 rad。

| 算法 | solve / s | 首解 / s | 原始长度 / rad | shortcut 后 / rad |
|---|---:|---:|---:|---:|
| RRT | 0.3921 | 0.3923 | 5.2033 | 2.2212 |
| RRT-Connect | 0.0040 | 0.0037 | 2.3497 | 2.2212 |
| RRT* | 30.0706 | 8.8483 | 5.3061 | 2.2213 |

RRT* 的 solve 包含预算内持续优化，首解来自中间解回调，无回调时采用返回时间近似。最终路径受到共同 shortcut 影响，不能把简化后的长度完全归因于搜索算法。没有测量 jerk 或振荡 RMS，也没有关闭平滑的物理消融，不提供抖动改善百分比。

## 逐任务记录

墙钟时间从提交任务到终止，不含冷启动。失败任务的路径、耗时与段数只覆盖前缀。

| 算法 | 种子 | 完整成功 | 墙钟 / s | 生成 / 到位段数 | 终止原因 |
|---|---:|---|---:|---:|---|
| RRT | 1 | 是 | 102.77 | 7 / 7 | pick_place_succeeded: 真实抓取、放置、松开和避障归位全部完成 |
| RRT-Connect | 1 | 是 | 114.31 | 7 / 7 | pick_place_succeeded: 真实抓取、放置、松开和避障归位全部完成 |
| RRT* | 1 | 否 | 86.00 | 3 / 3 | pick_place_failed: failed: rrt_star 未找到精确路径：Approximate solution；保持夹爪指令 |
| RRT | 2 | 否 | 26.44 | 3 / 3 | pick_place_failed: failed: 轨迹碰撞复检失败：碰撞: overhead_bridge <-> panda_link5；保持夹爪指令 |
| RRT-Connect | 2 | 否 | 69.58 | 5 / 5 | pick_place_failed: failed: 未找到满足末端误差和碰撞约束的 IK；保持夹爪指令 |
| RRT* | 2 | 否 | 86.58 | 3 / 3 | pick_place_failed: failed: 轨迹碰撞复检失败：碰撞: overhead_bridge <-> panda_link5；保持夹爪指令 |
| RRT | 3 | 否 | 55.89 | 3 / 3 | pick_place_failed: failed: rrt 未找到精确路径：Approximate solution；保持夹爪指令 |
| RRT-Connect | 3 | 是 | 90.27 | 7 / 7 | pick_place_succeeded: 真实抓取、放置、松开和避障归位全部完成 |
| RRT* | 3 | 否 | 85.93 | 3 / 3 | pick_place_failed: failed: rrt_star 未找到精确路径：Approximate solution；保持夹爪指令 |

## 携物阶段

| 算法 | 种子 | 轨迹生成 | solve / s | 结果 |
|---|---:|---|---:|---|
| RRT | 1 | 通过 | 7.1304 | 轨迹复检通过 |
| RRT-Connect | 1 | 通过 | 0.0159 | 轨迹复检通过 |
| RRT* | 1 | 失败 | 30.0613 | failed: rrt_star 未找到精确路径：Approximate solution |
| RRT | 2 | 失败 | 0.1825 | failed: 轨迹碰撞复检失败：碰撞: overhead_bridge <-> panda_link5 |
| RRT-Connect | 2 | 通过 | 0.0572 | 轨迹复检通过 |
| RRT* | 2 | 失败 | 30.0714 | failed: 轨迹碰撞复检失败：碰撞: overhead_bridge <-> panda_link5 |
| RRT | 3 | 失败 | 30.0848 | failed: rrt 未找到精确路径：Approximate solution |
| RRT-Connect | 3 | 通过 | 0.0645 | 轨迹复检通过 |
| RRT* | 3 | 失败 | 30.0573 | failed: rrt_star 未找到精确路径：Approximate solution |

## 数据与复现

[逐任务](results/unified_bounded_30s_2026-10-25/trials.csv) · [逐阶段](results/unified_bounded_30s_2026-10-25/phases.csv) · [汇总](results/unified_bounded_30s_2026-10-25/summary.csv) · [日志与输入快照](results/unified_bounded_30s_2026-10-25/README.md)

```bash
source /opt/ros/humble/setup.bash
cd ~/panda_control_ws
source install/setup.bash
CARTESIAN_TIMING=bounded RUNS=3 TIMEOUT=30.0 \
  ALGORITHMS='rrt rrt_connect rrt_star' \
  BENCHMARK_ROS_DOMAIN_ID=40 \
  BENCHMARK_IGN_PARTITION=panda_unified ./tools/compare_gazebo.sh
```

[旧保守时间的 5/30 秒对照](planner_comparison_conservative.md)与[此前单独的时间优化验证](cartesian_timing.md)作为独立历史组保留。当前默认搜索预算仍为 15 秒，此组通过启动参数覆盖为 30 秒。下一步扩大样本、定位失败阶段，并探索 A*。
