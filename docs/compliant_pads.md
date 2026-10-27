# 可选分块柔顺指垫

## 模型范围

这是集总弹簧阻尼近似：每侧四个独立接触片，共八个被动法向滑动关节。刚性片分别压缩，以近似局部贴合；没有有限元连续体、剪切变形或粘弹性材料本构，也没有把物体固定到夹爪。物体仍可能滑移和转动。原刚性指垫模式保留。

每片尺寸 8.5×2×9 mm，质量 0.5 g。每片初始刚度 10000 N/m、阻尼 1.4 N·s/m、最大压缩 1.5 mm，均为未标定的试验参数。在每侧四片均匀分担10 N的理想静态条件下，估计压缩为 10/(4×10000)=0.25 mm；实际接触片数和法向力会改变该数值，触及行程限位后会变硬。增加的指垫总质量为4 g，重力补偿计入其质量，并以零压缩姿态近似质心位置。

DART 的 SDF 加载器读取关节 spring_stiffness、spring_reference 与 damping。没有使用 ODE kp/kd/contact 参数来假设 DART 中存在同样的软接触，也没有用 slip1/slip2 代替橡胶柔顺性。参考：
- https://github.com/gazebosim/gz-physics/blob/ign-physics5/dartsim/src/SDFFeatures.cc
- https://sdformat.org/tutorials/specification/sdformat_urdf_extensions/

## 启动与恢复

关闭旧仿真后：

```bash
cd ~/panda_control_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
./start.sh scene:=world planner_algorithm:=rrt_connect planning_timeout:=30.0 compliant_pads:=true
```

抓放服务仍为 /pick_and_place。关闭仿真、重新启动并设置 compliant_pads:=false 即恢复原刚性指垫，不需要改源码。默认 false。

七轴机械臂的控制增益和关节阻尼未改。夹爪沿用当前恒力＋中心稳定配置，输出力上限仍为每指10 N。指垫阻尼只作用于新增的八个被动微行程关节。

## 参数试调

```bash
./start.sh scene:=world planner_algorithm:=rrt_connect planning_timeout:=30.0 \
  compliant_pads:=true pad_stiffness:=10000 pad_damping:=1.4 pad_travel:=0.0015
```

参数在启动时读取，修改需要重启。首先比较刚性和柔顺两组，其余场景、控制参数和任务种子相同。随后每次仅改变一个参数：刚度影响承载压缩量，阻尼影响回弹；不要把降低刚度当作必然提高抗滑。质量、刚度、阻尼和仿真步长共同影响数值稳定性。

此版本仍保留摩擦参数1.5作为待标定值，不把它称为PLA—橡胶实测系数。标定需要实际指垫与PLA试件的载荷—压缩曲线，以及滑动/抗扭实验；没有这些数据时只能做敏感性试验。每次关注夹持后的相对旋转、位移、脱落、接触状态和完整任务结果，不能用参考轨迹是否平滑代替物理夹持结果。

## 规划与反馈边界

规划节点使用单独的刚性未压缩包络URDF，包络覆盖接触片在法向行程内的几何范围；碰撞裕量和携物碰撞继续保留，不依赖被动关节的状态广播。仿真使用带被动关节的真实模型。ros2_control 仍只命令七轴和两根主手指，被动接触片不加入控制器命令数组。当前 /joint_states 和 robot_state_publisher 不提供这些被动关节的实测压缩量，不能将其TF零值当作测量数据；必要时需额外接入Gazebo被动关节状态采集。

已完成编译与URDF→SDF转换检查，确认八个关节的弹簧、阻尼、行程参数存在。已用于本次15次Gazebo任务：RRT-Connect与PRM＋A*各3/3完整成功，未完成任务均为搜索失败。未进行材料参数标定、柔顺模型消融或完整数值稳定性测试，不能推广为所有夹持工况的保证。
