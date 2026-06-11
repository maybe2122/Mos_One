# 参考论文汇总

本目录原存放的 4 篇 PDF 已删除，统一整理为本文档，附 arXiv 链接与各论文工作内容说明。
（2026-06-11 起，`doc/问题记录/` 里的论文 PDF 也合并到本文档：新增第 5、6 两篇综述；
其中 `scirobotics.adi7566.pdf` 为第 3 篇 ANYmal Parkour 的正式发表版，系重复，一并删除。）

---

## 1. Learning Agile Robotic Locomotion Skills by Imitating Animals

- **arXiv**: [2004.00784](https://arxiv.org/abs/2004.00784)
- **作者**: Xue Bin Peng, Erwin Coumans, Tingnan Zhang, Tsang-Wei Lee, Jie Tan, Sergey Levine
- **发表**: RSS 2020（2020 年 4 月提交）

**工作内容**：提出一个模仿学习系统，让足式机器人通过模仿真实动物的运动捕捉数据来学习敏捷运动技能。用单一的基于学习的框架，从参考运动数据自动合成多样化行为的控制器，替代耗时的手工控制器设计。训练中引入高样本效率的域适应（domain adaptation）技术，在仿真中学到自适应策略后可快速迁移到真实机器人。在 18 自由度四足机器人上实现了多种步态、动态跳跃和转向等敏捷行为。

**关键词**：模仿学习、运动重定向（motion retargeting）、域适应、sim-to-real

---

## 2. RMA: Rapid Motor Adaptation for Legged Robots

- **arXiv**: [2107.04034](https://arxiv.org/abs/2107.04034)
- **作者**: Ashish Kumar, Zipeng Fu, Deepak Pathak, Jitendra Malik
- **发表**: RSS 2021（2021 年 7 月提交）

**工作内容**：提出 RMA（快速运动适应）算法，解决四足机器人对地形变化、负载变化、机械磨损等未见场景的实时在线适应问题。架构分两部分：基础策略（base policy）+ 适应模块（adaptation module）——适应模块从近期状态-动作历史中在线估计环境隐变量（extrinsics），使机器人能在零点几秒内完成适应。完全在仿真中训练（多样化地形生成器 + 受生物能量学启发的奖励），不依赖参考轨迹或预定义足端轨迹生成器，无需微调即可直接部署到 Unitree A1 上，在岩石、湿滑、可变形表面、草地、楼梯、沙地等困难地形上达到 SOTA。

**关键词**：teacher-student / 隐变量估计、在线适应、零微调 sim-to-real

---

## 3. ANYmal Parkour: Learning Agile Navigation for Quadrupedal Robots

- **arXiv**: [2306.14874](https://arxiv.org/abs/2306.14874)
- **作者**: David Hoeller, Nikita Rudin, Dhionis Sako, Marco Hutter
- **发表**: Science Robotics 2024（2023 年 6 月提交 arXiv）
- **正式版**: [Sci. Robot. 9, eadi7566 (2024)](https://www.science.org/doi/10.1126/scirobotics.adi7566)，DOI `10.1126/scirobotics.adi7566`

**工作内容**：提出一套完全基于学习的分层方法，让 ANYmal 四足机器人完成跑酷式的高难度导航。先分别训练针对不同障碍的低层运动技能（行走、跳跃、攀爬、匍匐），再训练高层策略在地形中选择并调度这些技能；同时训练独立的感知模块，从高度遮挡、带噪声的传感器数据中重建障碍物几何，提供场景理解。整套方法无需专家示范、离线计算、环境先验或显式接触建模，仅用仿真数据训练即成功迁移到真实硬件，能以最高 2 m/s 的速度连续穿越高难度障碍。

**关键词**：分层策略、多技能调度、感知重建、跑酷

---

## 4. SoloParkour: Constrained Reinforcement Learning for Visual Locomotion from Privileged Experience

- **arXiv**: [2409.13678](https://arxiv.org/abs/2409.13678)
- **作者**: Elliot Chane-Sane, Joseph Amigo, Thomas Flayols, Ludovic Righetti, Nicolas Mansard
- **发表**: CoRL 2024（2024 年 9 月提交）

**工作内容**：提出在轻量四足机器人 Solo-12 上训练端到端视觉跑酷策略的新方法，直接从深度图像输出控制指令。将跑酷建模为约束强化学习（constrained RL）问题，在保证安全与物理限制的前提下最大化敏捷技能的涌现。训练分两步：先用环境特权信息训练一个无视觉策略；再用该特权策略生成经验，为基于深度图像的高样本效率离策略（off-policy）RL 提供热启动，从而把特权经验中的行为迁移到视觉策略，同时避免直接从像素做 RL 的高昂计算成本。真机上实现了行走、攀爬、跳跃、爬行等技能。

**关键词**：约束强化学习、特权学习、深度视觉策略、off-policy RL

---

## 5. Learning-based Legged Locomotion: State of the Art and Future Perspectives

- **arXiv**: [2406.01152](https://arxiv.org/abs/2406.01152)
- **作者**: Sehoon Ha, Joonho Lee, Michiel van de Panne, Zhaoming Xie, Wenhao Yu, Majid Khadiv
- **发表**: The International Journal of Robotics Research (IJRR) 2025, Vol. 44(8) 1396–1427，DOI `10.1177/02783649241312698`

**工作内容**：足式运动学习方向的权威综述。回顾该领域四十年发展史，系统总结近年来基于学习的四足运动技能研究——梳理硬件、物理仿真器、RL 算法、奖励设计、sim-to-real（域随机化/特权学习/在线适应）等关键要素的演进脉络；进一步延伸到人形/双足的类似方法；最后讨论开放问题（样本效率、安全性、感知融合、泛化）与社会影响。对刚进入该领域的研究者是很好的全景式入门材料，也正是本工程「RL 平地 → 地形泛化 → Sim2Real」路线的方法论背景。

**关键词**：综述、四足运动学习、强化学习、sim-to-real

---

## 6. Loco-Manipulation With Quadruped Robots: Modeling, Task Taxonomy, and Control Methods — A Critical Review

- **DOI**: [10.1109/ACCESS.2026.3672605](https://doi.org/10.1109/ACCESS.2026.3672605)
- **作者**: Md Hafizur Rahman, Muhammad Faizan Mysorewala, Muhammad Majid Gulzar, Sami El-Ferik, Tansu Sila Haque
- **发表**: IEEE Access, Vol. 14, 2026（Topical Review）

**工作内容**：四足 loco-manipulation（运动 + 操作一体化）的批判性综述。提出「建模—任务—控制」统一三元框架：① 浮动基座与接触建模假设；② loco-manipulation 任务域分类（六轴任务taxonomy）；③ 控制器家族——模型驱动的逆动力学全身控制（WBC）、优化驱动的全身 MPC、学习驱动的 RL/IL 以及混合架构，并用证据驱动的 trade-off 图与失效模式归纳做跨范式定量对比。指出最难 sim-to-real 的任务域集中在接触密集末端交互、不确定落足点与感知遮挡耦合的场景；强调高保真仿真与数字孪生管线是可复现与可靠迁移的实用使能项。对本工程后续「带臂/带载操作」类扩展有参考价值。

**关键词**：综述、loco-manipulation、全身控制 WBC、MPC、RL/IL 混合架构
