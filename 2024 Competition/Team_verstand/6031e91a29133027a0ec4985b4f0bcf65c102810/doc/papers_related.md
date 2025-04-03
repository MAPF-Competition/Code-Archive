# Paper related to LoRR 2024

从出题人的角度考虑，阅读出题人近年来的论文也是把握题目思路的一种方法。

仿真在BaseSystem::simulate中进行。计算在Entry::compute中进行。

套壳：driver.cpp中的main 

调用了 CompetitionSystem.cpp中的BaseSystem::simulate 

调用了 CompetitionSystem.cpp中的BaseSystem::plan

调用了 CompetitionSystem.cpp中的BaseSystem::plan_wrapper

调用了 Entry.cpp中的Entry::compute

调用了 TaskScheduler.cpp中的plan (调用了scheduler.cpp中的DefaultPlanner::schedule_plan) 和 MAPFPlanner.cpp中的plan (调用了planner.cpp中的plan)

如果从布线角度去理解MAPD, 任务分配是从agent当前位置到目标位置连一条线，路径规划是把这些线尽可能理顺。

吾人猜测，RAMDOM-1到RANDOM-5的地图可能小而拥挤，不然怎么5000s才完成751个任务。

分数是不断变化的，以不断刷新的最优解为分母。所以统计分数不如统计任务数量。

## Task assignment

默认给的任务分配算法比较简单。估计优化task assignment 比 优化path planning更容易突破。

把whoenig算法以及whoenig提到的网络流算法测试一下效果。然后再去阅读其他论文。

算例random_32_32_20_100.json初始free agent num: 100。free task num: 150。而且没完成一个旧工作会冒出一个新工作。这说明free task始终比free agent多出50个。

如果num_agents 约等于 num_tasks, 抢单的效果比较好；如果num_agents >> num_tasks 或 num_tasks >> num_agents，吾人猜测效果有限。或许考虑交通拥堵和用户均衡是更优的选择。

目前的启发式距离get_h(.)，考虑了静态障碍物，但是没有考虑动态障碍物（其他agent），这或许是TFO这篇文章所能带给吾人的启发。TFO文章通过增加边权重的方式减少了拥堵，不知道这个增加了拥堵系数的边权重有没有在get_h中体现出来？

plan具有SharedEnvironment的参数，这意味着吾人可以从中推断出路径的拥堵、地图的分布情况。

先不用担心超时的问题。把random map的几个算例搞好。

flow是在TrajLNS中定义的。env可以抵达这里吗？从include关系上看是不可以的。

可以推断出agent最多有25000个。

初始任务可以预分配。=> 预分配不了，连任务是哪些都不知道。

Agent数量 * 仿真时间 = 完成任务数量 * 任务平均完成时间

取法其上得乎其中，如果把竞赛当成论文的标准来做，就能取得更好的效果。

### 匈牙利算法

在横竖相等的匈牙利算法中，不会有多余的free agent或者free task产生出来。

如果num agent > num task, 有可能产生新的free agent.

如果num agent < num task, 有可能产生新的free task. 

### 拥堵系数

拥堵时间 = 任务实际运行时间（只考虑静态障碍物） - 事先估计的启发式时间（还考虑了其他agent）。拥堵系数由拥堵时间反向推断。

## Path planning

吾人觉得，不必被几个组卷帙浩繁的paper数量感到悲观。真正和问题相关的paper只有同样单步规划的LACAM和highway。

2024 Guidance Graph Optimization for Lifelong Multi-Agent Path Finding

https://arxiv.org/abs/2402.01446

使用了引导图加大吞吐量，而且提供了代码。

2022 Priority inheritance with backtracking for iterative multi-agent path finding

https://www.sciencedirect.com/science/article/pii/S0004370222000923

PIBT (1, 2)算法。官方提供的默认路径规划算法。

Lacam(star) 算法。测试一下效果。

实现上修改planner->plan(time_limit,plan);的代码。

traj是路径的意思。

### Q: 在一个空旷的grid-map中。假设每个grid只能朝东南西北四个方向中的一个移动（原地等待也可以）。有若干agent在这个map中运送货物。怎样设置每个grid的朝向，可以让agent送货流程最短（由于规划时间有限，次优解也可以）？

为了求解一个次优解，可以采用以下方法：

#### **1. 基于最短路径生成的启发式方法**

- **计算最短路径：** 对所有 agent，假设 grid 没有方向限制，计算每个 agent 的独立最短路径（如 A* 或 Dijkstra 算法）。
- **统计路径流量：** 记录每个 grid-cell 被经过的次数，以及经过的方向。
- **设置朝向：** 根据路径流量统计，为每个 grid-cell 设置流量最大的方向作为朝向。

**优点：**

- 快速得到合理的方向配置。
- 使用统计结果可避免方向冲突，适合多 agent 场景。

#### **2. 基于概率图模型生成朝向**

- **定义方向变量：** 将每个 grid 的方向（朝东、南、西、北或原地等待）建模为概率变量。
- **构建目标函数：** 使用所有 agent 的独立最短路径信息，定义目标函数，最大化 agent 在 grid 上流量的朝向匹配度。
- **求解问题：** 使用图模型推断方法（如 Gibbs 采样或变分推断）确定每个 grid 的最优朝向。

**优点：**

- 更智能地综合全局信息。
- 可以生成平滑的方向配置，避免局部冲突。

#### **3. 基于强化学习（RL）优化方向**

- **状态：** 以 grid-map 的方向配置和所有 agent 的位置为状态。
- **动作：** 每次调整一个或多个 grid 的方向。
- **奖励：** 根据所有 agent 完成任务的总路径长度或时间定义奖励函数。
- **训练：** 使用 Q-Learning 或 Policy Gradient 方法优化 grid 的方向配置。

**优点：**

- 能处理复杂情况，例如 agent 动态变化或 grid 上存在障碍物。

#### **4. 基于局部搜索的优化方法**

- **初始方向：** 随机生成或使用启发式生成初始方向配置。
- **局部调整：** 针对局部区域（例如一个 grid 或 3x3 的区域），调整朝向并评估总路径长度变化。
- **迭代优化：** 反复进行局部调整，直到收敛或达到时间限制。

**优点：**

- 算法简单且易于实现。
- 能快速得到次优解。

**示例方法：统计路径流量**

假设有两个 agent，它们的起点和终点分别是：

- Agent 1：起点 (1, 1)，终点 (5, 5)
- Agent 2：起点 (5, 1)，终点 (1, 5)

**步骤：**

1. **计算最短路径：**
   - Agent 1：沿对角线移动，从 (1,1) 到 (5,5)。
   - Agent 2：从 (5,1) 沿对角线到 (1,5)。
2. **统计流量：**
   - 统计每个 grid 的通行方向，例如 (2,2) 被两个 agent 从不同方向穿过。
3. **设置方向：**
   - 对每个 grid，选择通行流量最大的方向。例如，如果 (2,2) 的通行流量偏向东南方向，则设置其朝向为东南。

**关键优化技巧**

- **动态调整：** 在任务执行过程中，根据 agent 的实际分布动态调整 grid 朝向。
- **区域化规划：** 将 grid-map 划分为多个区域，分别优化每个区域的方向配置以适应局部需求。
- **交通规则引导：** 在 grid-map 上设置“单向通道”或“高速通道”，减少冲突并引导 agent 流量。

**总结**

优化 grid-map 的方向配置是一种 NP 难问题，通过上述方法可以高效获得次优解。具体选择哪种方法取决于：

1. 任务规模（agent 数量与 grid-map 大小）。
2. 算法实现难度（如统计方法易于实现，而强化学习需要更多计算资源）。
3. 实时性要求（如路径流量统计和局部搜索方法适合快速决策场景）。

## Instance

### Random: random_32_32_20_100.json

map: 32x32

agent: 100

initial task: 150

free task: 150 - 100 = 50

### City: paris_1_256_250.json

map: 256x256

agent: 250

initial task: 375

free task: 375 - 250 = 125

### Game: brc202d_500.json

map: 481x530

agent: 500

initial task: 750

free task: 750 - 250 = 500

### Sortation: sortation_large_2000.json

map: 140x500

agent: 2000

initial task: 3000

free task: 3000 - 2000 = 1000

comment:  sorting centor不是吾人想象的走廊图，而是点状图。这就意味着没有必要计算周围的静态障碍了，因为都是一样的。

### Warehouse: warehouse_large_5000.json

map: 140x500

agent: 5000

initial task: 7500

free task: 7500 - 5000 = 2500

comment:  是匀质的走廊图。这就意味着没有必要计算周围的静态障碍了，因为都是一样的。

## Comment

用mysql管理论文比用markdown更好一些。因为markdown里面重复的太多。

