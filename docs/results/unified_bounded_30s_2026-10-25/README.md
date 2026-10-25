# 最新统一三算法原始数据

9 次全新有效 Gazebo 任务：RRT / RRT-Connect / RRT*，种子 1/2/3，每次全局搜索预算 30 秒，cartesian_timing=bounded。完整任务要求七段全部完成。raw 保留任务 CSV、事件、启动与采集日志，包括任务提交前的一次基础设施失败；未剔除或重跑任何有效失败。

summary.csv / trials.csv / phases.csv 从原始任务和日志汇总。robot.urdf、world.urdf、world.sdf、controller.yaml 是输入快照，source_sha256.txt 记录有效试验源码。environment.txt 在恢复批次时采集，trial_domains.csv 包含首次无效启动及恢复后的隔离域。前次启动失败不计入九次任务。

各次物理接触和初始状态并非确定性回放。成功样本为7/7到位；失败路径和时间仅覆盖前缀。首次控制器查询失败后增加只读查询有限重试，所有有效任务使用同一修正版本。
