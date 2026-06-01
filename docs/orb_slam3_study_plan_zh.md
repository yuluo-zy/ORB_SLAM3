# ORB_SLAM3 源码精读顺序图

这份文档的目标不是把所有文件列一遍，而是给出一条适合第一遍精读的主线。建议你先按“两周版”走完整个系统，再回头补数学细节和 IMU 推导。

## 学习原则

1. 第一遍先追主流程，不要一开始就陷进 `Optimizer.cc` 和 `ImuTypes.cc`。
2. 每天只解决一个核心问题，比如“当前帧是怎么变成位姿的”“关键帧为什么会被插入”。
3. 阅读时优先看“谁调用谁”，其次才是“公式为什么这样写”。
4. 每天结束前，至少画一张自己的调用图或数据流图。

## 总体顺序图

```text
示例入口
  -> System 系统装配与线程启动
  -> Tracking 前端主流程
  -> Frame / KeyFrame / MapPoint 核心数据结构
  -> Map / Atlas 地图组织
  -> ORBextractor / ORBmatcher 前端支撑模块
  -> LocalMapping 局部建图线程
  -> Optimizer 局部 BA / 位姿优化
  -> KeyFrameDatabase / LoopClosing 回环与重定位
  -> Atlas 多地图与地图合并
  -> IMU 预积分与视觉惯导扩展
  -> CameraModels / Settings / G2oTypes 收尾补强
```

## 建议先准备的调试入口

1. 示例主程序：[Examples/Monocular/mono_tum.cc](</C:/Users/liyu/CLionProjects/ORB_SLAM3/Examples/Monocular/mono_tum.cc:33>)
2. 系统入口：[include/System.h](</C:/Users/liyu/CLionProjects/ORB_SLAM3/include/System.h:1>) 和 [src/System.cc](</C:/Users/liyu/CLionProjects/ORB_SLAM3/src/System.cc:41>)
3. 前端入口：[include/Tracking.h](</C:/Users/liyu/CLionProjects/ORB_SLAM3/include/Tracking.h:1>) 和 [src/Tracking.cc](</C:/Users/liyu/CLionProjects/ORB_SLAM3/src/Tracking.cc:1566>)

建议第一天就把下面这条链路单步走一遍：

```text
mono_tum.cc
  -> System::TrackMonocular
  -> Tracking::GrabImageMonocular
  -> Tracking::Track
```

对应代码位置：

1. [mono_tum.cc 调用 TrackMonocular](</C:/Users/liyu/CLionProjects/ORB_SLAM3/Examples/Monocular/mono_tum.cc:109>)
2. [System::TrackMonocular](</C:/Users/liyu/CLionProjects/ORB_SLAM3/src/System.cc:399>)
3. [Tracking::GrabImageMonocular](</C:/Users/liyu/CLionProjects/ORB_SLAM3/src/Tracking.cc:1566>)
4. [Tracking::Track](</C:/Users/liyu/CLionProjects/ORB_SLAM3/src/Tracking.cc:1794>)

## 两周版精读计划

### Day 1: 先建立全局心智模型

目标：知道这个项目从外部怎么被调用，以及内部启动了哪些线程。

必看文件：

1. [README.md](</C:/Users/liyu/CLionProjects/ORB_SLAM3/README.md:1>)
2. [Examples/Monocular/mono_tum.cc](</C:/Users/liyu/CLionProjects/ORB_SLAM3/Examples/Monocular/mono_tum.cc:33>)
3. [include/System.h](</C:/Users/liyu/CLionProjects/ORB_SLAM3/include/System.h:1>)
4. [src/System.cc](</C:/Users/liyu/CLionProjects/ORB_SLAM3/src/System.cc:41>)

重点问题：

1. `System` 构造时创建了哪些模块。
2. 哪些模块是线程，哪些模块在主线程里。
3. 外部程序为什么只需要调用 `TrackMonocular` 这类接口。

重点代码：

1. [LocalMapping 线程启动](</C:/Users/liyu/CLionProjects/ORB_SLAM3/src/System.cc:197>)
2. [LoopClosing 线程启动](</C:/Users/liyu/CLionProjects/ORB_SLAM3/src/System.cc:214>)
3. [Viewer 线程启动](</C:/Users/liyu/CLionProjects/ORB_SLAM3/src/System.cc:233>)

当天产出：

1. 画一张 4 模块图：`Tracking`、`LocalMapping`、`LoopClosing`、`Atlas`
2. 写清楚“主线程做什么，后台线程做什么”

### Day 2: 把单目主流程走通

目标：看懂一帧图像进入系统后，前端主流程怎么跑起来。

必看文件：

1. [include/Tracking.h](</C:/Users/liyu/CLionProjects/ORB_SLAM3/include/Tracking.h:1>)
2. [src/Tracking.cc](</C:/Users/liyu/CLionProjects/ORB_SLAM3/src/Tracking.cc:1566>)
3. [src/Tracking.cc Track 主循环](</C:/Users/liyu/CLionProjects/ORB_SLAM3/src/Tracking.cc:1794>)

重点问题：

1. `GrabImageMonocular` 做了哪些输入准备。
2. `Track()` 里状态机怎么切换。
3. 初始化、正常跟踪、丢失、重定位的分支长什么样。

当天产出：

1. 画一张 `Track()` 的状态流转图
2. 列出前端最核心的 5 个步骤

### Day 3: 先吃透三个核心对象

目标：理解整个系统围绕哪些核心对象组织。

必看文件：

1. [include/Frame.h](</C:/Users/liyu/CLionProjects/ORB_SLAM3/include/Frame.h:53>)
2. [include/KeyFrame.h](</C:/Users/liyu/CLionProjects/ORB_SLAM3/include/KeyFrame.h:52>)
3. [include/MapPoint.h](</C:/Users/liyu/CLionProjects/ORB_SLAM3/include/MapPoint.h:44>)
4. [src/Frame.cc](</C:/Users/liyu/CLionProjects/ORB_SLAM3/src/Frame.cc:45>)
5. [src/KeyFrame.cc](</C:/Users/liyu/CLionProjects/ORB_SLAM3/src/KeyFrame.cc:1>)
6. [src/MapPoint.cc](</C:/Users/liyu/CLionProjects/ORB_SLAM3/src/MapPoint.cc:1>)

重点问题：

1. `Frame` 和 `KeyFrame` 的根本区别是什么。
2. `MapPoint` 里保存了哪些几何和观测信息。
3. 为什么 `KeyFrame` 是图优化和回环的核心节点。

当天产出：

1. 画一张 `Frame -> KeyFrame -> MapPoint` 关系图
2. 写清楚每个对象的生命周期

### Day 4: 读地图组织方式

目标：理解 ORB_SLAM3 为什么不是“只有一张地图”。

必看文件：

1. [include/Map.h](</C:/Users/liyu/CLionProjects/ORB_SLAM3/include/Map.h:41>)
2. [include/Atlas.h](</C:/Users/liyu/CLionProjects/ORB_SLAM3/include/Atlas.h:49>)
3. [src/Map.cc](</C:/Users/liyu/CLionProjects/ORB_SLAM3/src/Map.cc:1>)
4. [src/Atlas.cc](</C:/Users/liyu/CLionProjects/ORB_SLAM3/src/Atlas.cc:1>)

重点问题：

1. `Map` 管的是一张什么意义上的地图。
2. `Atlas` 为什么要管理多个 `Map`。
3. ORB_SLAM3 相比 ORB-SLAM2 多了什么系统级能力。

当天产出：

1. 用一句话分别定义 `Map` 和 `Atlas`
2. 画出“新地图创建、切换、合并”的概念图

### Day 5: 前端初始化与局部地图关联

目标：看懂前端如何从“没地图”进入“可跟踪”状态。

必看文件：

1. [src/Tracking.cc](</C:/Users/liyu/CLionProjects/ORB_SLAM3/src/Tracking.cc:1794>)
2. [include/TwoViewReconstruction.h](</C:/Users/liyu/CLionProjects/ORB_SLAM3/include/TwoViewReconstruction.h:1>)
3. [src/TwoViewReconstruction.cc](</C:/Users/liyu/CLionProjects/ORB_SLAM3/src/TwoViewReconstruction.cc:1>)

重点问题：

1. 单目初始化为什么最难。
2. 初始化阶段是怎么从两帧恢复结构的。
3. 初始化成功后，第一批关键帧和地图点怎么生成。

当天产出：

1. 画出单目初始化流程图
2. 记下“初始化失败”的几个常见条件

### Day 6: 前端正常跟踪、重定位、插入关键帧

目标：理解前端在已建图情况下怎么持续工作。

必看文件：

1. [src/Tracking.cc](</C:/Users/liyu/CLionProjects/ORB_SLAM3/src/Tracking.cc:1794>)
2. [include/KeyFrameDatabase.h](</C:/Users/liyu/CLionProjects/ORB_SLAM3/include/KeyFrameDatabase.h:47>)
3. [src/KeyFrameDatabase.cc](</C:/Users/liyu/CLionProjects/ORB_SLAM3/src/KeyFrameDatabase.cc:100>)
4. [src/KeyFrameDatabase.cc 重定位候选](</C:/Users/liyu/CLionProjects/ORB_SLAM3/src/KeyFrameDatabase.cc:733>)

重点问题：

1. 运动模型跟踪和参考关键帧跟踪分别在什么时候用。
2. 重定位依赖什么候选机制。
3. 当前帧满足什么条件才会插入关键帧。

当天产出：

1. 列出“关键帧插入条件”
2. 列出“丢失后恢复”的关键调用链

### Day 7: 特征提取与匹配机制

目标：把前端的“输入信号源”和“匹配工具箱”补齐。

必看文件：

1. [include/ORBextractor.h](</C:/Users/liyu/CLionProjects/ORB_SLAM3/include/ORBextractor.h:1>)
2. [src/ORBextractor.cc](</C:/Users/liyu/CLionProjects/ORB_SLAM3/src/ORBextractor.cc:1086>)
3. [src/ORBextractor.cc ComputePyramid](</C:/Users/liyu/CLionProjects/ORB_SLAM3/src/ORBextractor.cc:1170>)
4. [src/ORBextractor.cc ComputeKeyPointsOctTree](</C:/Users/liyu/CLionProjects/ORB_SLAM3/src/ORBextractor.cc:781>)
5. [include/ORBmatcher.h](</C:/Users/liyu/CLionProjects/ORB_SLAM3/include/ORBmatcher.h:36>)
6. [src/ORBmatcher.cc](</C:/Users/liyu/CLionProjects/ORB_SLAM3/src/ORBmatcher.cc:43>)
7. [SearchForInitialization](</C:/Users/liyu/CLionProjects/ORB_SLAM3/src/ORBmatcher.cc:648>)
8. [SearchByBoW](</C:/Users/liyu/CLionProjects/ORB_SLAM3/src/ORBmatcher.cc:223>)
9. [SearchByProjection](</C:/Users/liyu/CLionProjects/ORB_SLAM3/src/ORBmatcher.cc:1676>)

重点问题：

1. ORB 特征金字塔和分层采样怎么实现。
2. 系统里常见的匹配方式有哪些。
3. 为什么不同阶段会用不同的匹配器入口。

当天产出：

1. 画一张“初始化匹配 / 投影匹配 / BoW 匹配”对照表
2. 标记哪些匹配主要服务于前端，哪些服务于回环

### Day 8: LocalMapping 线程

目标：理解关键帧一旦入队，局部建图线程会做什么。

必看文件：

1. [include/LocalMapping.h](</C:/Users/liyu/CLionProjects/ORB_SLAM3/include/LocalMapping.h:1>)
2. [src/LocalMapping.cc](</C:/Users/liyu/CLionProjects/ORB_SLAM3/src/LocalMapping.cc:64>)

重点问题：

1. `InsertKeyFrame` 之后，关键帧如何被消费。
2. 新地图点如何三角化出来。
3. 地图点剔除、关键帧剔除、邻居搜索各自解决什么问题。

当天产出：

1. 画出 `LocalMapping::Run()` 的循环骨架
2. 记清楚 Local Mapping 的 4 个主要任务

### Day 9: 位姿优化与局部 BA

目标：只抓核心入口，不逐行硬读整个优化器。

必看文件：

1. [include/Optimizer.h](</C:/Users/liyu/CLionProjects/ORB_SLAM3/include/Optimizer.h:1>)
2. [src/Optimizer.cc PoseOptimization](</C:/Users/liyu/CLionProjects/ORB_SLAM3/src/Optimizer.cc:814>)
3. [src/Optimizer.cc LocalBundleAdjustment](</C:/Users/liyu/CLionProjects/ORB_SLAM3/src/Optimizer.cc:1116>)
4. [include/G2oTypes.h](</C:/Users/liyu/CLionProjects/ORB_SLAM3/include/G2oTypes.h:1>)
5. [src/G2oTypes.cc](</C:/Users/liyu/CLionProjects/ORB_SLAM3/src/G2oTypes.cc:1>)

重点问题：

1. 位姿优化输入输出是什么。
2. 局部 BA 优化哪些变量，固定哪些变量。
3. `Tracking` 和 `LocalMapping` 各自调用哪些优化入口。

当天产出：

1. 写一张“优化入口 -> 调用场景”对照表
2. 只总结图优化变量，不展开数学细节

### Day 10: 回环检测与闭环校正

目标：理解系统如何发现“我来过这里”，以及发现后怎么纠错。

必看文件：

1. [include/LoopClosing.h](</C:/Users/liyu/CLionProjects/ORB_SLAM3/include/LoopClosing.h:1>)
2. [src/LoopClosing.cc](</C:/Users/liyu/CLionProjects/ORB_SLAM3/src/LoopClosing.cc:90>)
3. [src/Optimizer.cc OptimizeEssentialGraph](</C:/Users/liyu/CLionProjects/ORB_SLAM3/src/Optimizer.cc:1501>)
4. [include/Sim3Solver.h](</C:/Users/liyu/CLionProjects/ORB_SLAM3/include/Sim3Solver.h:1>)
5. [src/Sim3Solver.cc](</C:/Users/liyu/CLionProjects/ORB_SLAM3/src/Sim3Solver.cc:1>)

重点问题：

1. 回环候选从哪里来。
2. 为什么要做 Sim3 验证。
3. 回环后图结构和地图点会怎么被修正。

当天产出：

1. 画一张回环处理链路图
2. 分清“候选发现”“几何验证”“图优化”三步

### Day 11: 多地图与地图合并

目标：理解 ORB_SLAM3 的一个核心升级点。

必看文件：

1. [include/Atlas.h](</C:/Users/liyu/CLionProjects/ORB_SLAM3/include/Atlas.h:49>)
2. [src/Atlas.cc](</C:/Users/liyu/CLionProjects/ORB_SLAM3/src/Atlas.cc:1>)
3. [src/LoopClosing.cc](</C:/Users/liyu/CLionProjects/ORB_SLAM3/src/LoopClosing.cc:90>)

重点问题：

1. 地图合并和普通回环有什么关系。
2. 为什么丢失后重新建图而不是强行接回旧图。
3. `Atlas` 如何支撑多地图并行存在。

当天产出：

1. 画出 `Map` 与 `Atlas` 的层级关系
2. 用你自己的话解释“merge”相比“loop correction”多做了什么

### Day 12: 多传感器输入与配置系统

目标：理解单目、双目、RGB-D、视觉惯导在系统入口上的差别。

必看文件：

1. [include/System.h](</C:/Users/liyu/CLionProjects/ORB_SLAM3/include/System.h:91>)
2. [src/System.cc](</C:/Users/liyu/CLionProjects/ORB_SLAM3/src/System.cc:244>)
3. [include/Settings.h](</C:/Users/liyu/CLionProjects/ORB_SLAM3/include/Settings.h:1>)
4. [src/Settings.cc](</C:/Users/liyu/CLionProjects/ORB_SLAM3/src/Settings.cc:148>)
5. [include/CameraModels/GeometricCamera.h](</C:/Users/liyu/CLionProjects/ORB_SLAM3/include/CameraModels/GeometricCamera.h:1>)
6. [include/CameraModels/Pinhole.h](</C:/Users/liyu/CLionProjects/ORB_SLAM3/include/CameraModels/Pinhole.h:1>)
7. [include/CameraModels/KannalaBrandt8.h](</C:/Users/liyu/CLionProjects/ORB_SLAM3/include/CameraModels/KannalaBrandt8.h:1>)

重点问题：

1. 三种视觉输入接口差在哪里。
2. 设置文件是如何把相机和 IMU 参数喂进系统的。
3. 相机模型抽象为什么要单独成层。

当天产出：

1. 列一张“Monocular / Stereo / RGB-D / IMU”输入差异表
2. 标记相机模型与前端投影、反投影的关系

### Day 13: IMU 扩展主链

目标：理解 IMU 是如何接入现有视觉前端和后端的。

必看文件：

1. [include/ImuTypes.h](</C:/Users/liyu/CLionProjects/ORB_SLAM3/include/ImuTypes.h:40>)
2. [src/ImuTypes.cc](</C:/Users/liyu/CLionProjects/ORB_SLAM3/src/ImuTypes.cc:107>)
3. [src/Tracking.cc ParseIMUParamFile](</C:/Users/liyu/CLionProjects/ORB_SLAM3/src/Tracking.cc:1301>)
4. [src/Tracking.cc PreintegrateIMU](</C:/Users/liyu/CLionProjects/ORB_SLAM3/src/Tracking.cc:1624>)
5. [src/Tracking.cc PredictStateIMU](</C:/Users/liyu/CLionProjects/ORB_SLAM3/src/Tracking.cc:1738>)
6. [src/LocalMapping.cc InitializeIMU](</C:/Users/liyu/CLionProjects/ORB_SLAM3/src/LocalMapping.cc:1173>)

重点问题：

1. IMU 预积分对象保存了什么。
2. 视觉前端在哪些时刻会用 IMU 做预测。
3. IMU 初始化为什么放在 `LocalMapping` 中完成。

当天产出：

1. 画出“IMU 数据流”：采样 -> 队列 -> 预积分 -> 预测 -> 图优化
2. 只总结接口和数据流，不急着推公式

### Day 14: IMU 优化、收尾补强、知识回放

目标：把视觉版和视觉惯导版统一成一张系统图。

必看文件：

1. [src/Optimizer.cc GlobalBundleAdjustemnt](</C:/Users/liyu/CLionProjects/ORB_SLAM3/src/Optimizer.cc:52>)
2. [src/Optimizer.cc](</C:/Users/liyu/CLionProjects/ORB_SLAM3/src/Optimizer.cc:2605>)
3. [src/Optimizer.cc](</C:/Users/liyu/CLionProjects/ORB_SLAM3/src/Optimizer.cc:3127>)
4. [include/G2oTypes.h](</C:/Users/liyu/CLionProjects/ORB_SLAM3/include/G2oTypes.h:93>)
5. [src/G2oTypes.cc](</C:/Users/liyu/CLionProjects/ORB_SLAM3/src/G2oTypes.cc:492>)
6. [src/Tracking.cc UpdateFrameIMU](</C:/Users/liyu/CLionProjects/ORB_SLAM3/src/Tracking.cc:3980>)

重点问题：

1. IMU 约束在图优化中是怎么接进去的。
2. 视觉 SLAM 和视觉惯导 SLAM 共享了哪些框架。
3. 还有哪些文件应该留到第二遍再精读。

当天产出：

1. 画一张最终系统图
2. 写一页自己的总结，回答“ORB_SLAM3 比 ORB-SLAM2 多了什么”

## 一周压缩版

如果你只有 7 天，可以把两周版压成下面这样：

1. Day 1: `mono_tum.cc` + `System`
2. Day 2: `Tracking`
3. Day 3: `Frame` / `KeyFrame` / `MapPoint` / `Map` / `Atlas`
4. Day 4: `ORBextractor` / `ORBmatcher` / `TwoViewReconstruction`
5. Day 5: `LocalMapping` + `Optimizer` 核心入口
6. Day 6: `KeyFrameDatabase` + `LoopClosing` + `Sim3Solver`
7. Day 7: `ImuTypes` + `Settings` + `CameraModels`，最后回画系统总图

## 第一遍建议暂缓深挖的文件

这些文件不是不重要，而是不适合一开始逐行硬读：

1. `src/Optimizer.cc`
2. `src/G2oTypes.cc`
3. `src/ImuTypes.cc`
4. `Thirdparty/`
5. `Vocabulary/`
6. `Viewer` 相关文件

## 每天阅读时建议固定回答的 5 个问题

1. 这个模块的输入是什么。
2. 这个模块的输出是什么。
3. 它依赖哪些核心对象。
4. 它在哪个线程里运行。
5. 它解决的是“跟踪、建图、优化、回环、融合”里的哪一类问题。

## 学完第一遍后你应该达到的程度

1. 能从示例入口一路讲清楚前端主链。
2. 能解释 `Frame`、`KeyFrame`、`MapPoint`、`Map`、`Atlas` 的职责。
3. 能说清楚三大线程如何配合。
4. 能解释 ORB_SLAM3 相比 ORB-SLAM2 的两大升级：`Atlas` 和 `IMU`。
5. 能知道哪些文件适合第二遍做数学级精读。

## 第二遍建议

第一遍完成后，建议按这个顺序进入第二遍：

1. `Optimizer.cc`
2. `G2oTypes.cc`
3. `ImuTypes.cc`
4. `CameraModels/`
5. `LoopClosing.cc` 中的地图合并细节

