# 倒立摆模糊控制系统 (IPC_Fuzzy)

基于 MATLAB 的车-摆系统仿真项目，使用模糊逻辑控制器实现对倒立摆的稳定控制与可视化分析。

> 说明：本 README 仅描述当前仓库中模糊控制程序（`inverted_pendulum_Fuzzy.m`）的配置、参数与运行方式；PID 对比部分已移除。

## 快速开始

运行环境：MATLAB R2016b 或更高版本（无需额外工具箱）。

运行步骤：
```matlab
cd d:\300-code\IPC_Fuzzy
inverted_pendulum_Fuzzy
```

程序会在运行目录下生成 `figures/` 文件夹并保存仿真生成的图片。

## 文件结构（简要）

```
IPC_Fuzzy/
├── inverted_pendulum_Fuzzy.m    % 主程序（模糊控制）
├── README.md                     % 本文件
├── CLAUDE.md                     % AI 助手说明
├── figures/                      % 仿真输出图片
└── sub_functions/                % 子函数
    ├── fuzzification.m
    ├── fuzzy_inference.m
    ├── defuzzification.m
    └── diff_pendulum.m
```

## 当前关键参数（来自 `inverted_pendulum_Fuzzy.m`）

物理参数：
- 小车质量 M = 2 kg
- 摆杆质量 m = 0.8 kg
- 摆杆长度 l = 0.25 m
- 小车阻尼 bx = 0.005 kg/s
- 摆杆阻尼 bq = 0.0005 kg·m²/s
- 转动惯量 J = 0.0326 kg·m²

仿真设置：
- 仿真总时长 T_final = 20 s
- 控制步长 Ts = 0.01 s

归一化因子（映射到模糊域 [-1, 1]）：
- 小车位置：x_normal = 12 m
- 小车速度：dx_normal = 1.5 m/s
- 摆杆角度：q_normal = 360° (2π rad)
- 摆杆角速度：dq_normal = 180° (π rad)
- 控制力：u_normal = 1000 N

控制增益（双控制器架构）：
- 位置控制器增益 gain_x = 1
- 角度控制器增益 gain_q = 2

初始条件（当前）：
```matlab
% 状态向量 X = [cart_position; cart_velocity; pendulum_angle; angular_velocity]
X0 = [-0.5; 0; 20*pi/180; 0];  % 小车位于 -0.5 m，摆杆初始角度 20°
```

## 控制器与算法简述

控制流程（每个采样周期）：

1. 计算误差：ex = xd - x；eq = qd - θ
2. 归一化误差并进行模糊化（`fuzzification.m`）
3. 模糊推理（`fuzzy_inference.m`），使用 5×5 规则表
4. 去模糊化（`defuzzification.m`），采用重心法得到归一化输出 u_tilde
5. 将归一化输出还原为物理控制力并合成：F = -u_x + u_q

模糊集合：NB, NS, ZO, PS, PB（5 个集合）。

## 输出文件（figures）

仿真运行后会保存下列主要图片（位于 `figures/`）：

- `Fig1_System_Snapshots.png`：系统在若干关键时刻的几何快照（小车与摆杆）
- `Fig2_State_Response.png`：小车位置/速度、摆杆角度/角速度、控制力随时间的曲线
- `Fig3_Performance_Analysis.png`：误差曲线、调节时间、超调、累积控制能量等指标
- `Fig4_Fuzzy_System.png`：隶属函数、规则表、控制曲面、输入轨迹、相平面等可视化

这些图片适用于报告插图与性能分析。

## 核心子函数说明

`fuzzification.m`：将归一化误差（e, de）映射到 5×2 隶属度矩阵（NB..PB）。

`fuzzy_inference.m`：根据预先定义的规则表对模糊输入进行推理，输出被激活的模糊集合及其置信度。

`defuzzification.m`：对推理结果做聚合并用重心法计算归一化控制输出 u_tilde ∈ [-1, 1]。

`diff_pendulum.m`：系统动力学（拉格朗日方法）——用于 ODE 积分计算状态更新。

## 运行细节与数据记录

- 积分方法：`ode45`（每个采样周期用 `[0, Ts]` 进行一小段积分并取结束值）。
- 记录变量：`X_Fuzzy`（状态历史）、`time_Fuzzy`（时间）、`F_save`（控制力历史）、`u_save`（子控制器输出）、`fuzzy_data`（模糊输入数据）。
- 误差计算（代码中）：
  - 位置误差 ex = xd - X(1)
  - 位置变化 dex = x_prev - X(1)  （差分近似）
  - 角度误差 eq = qd - X(3)
  - 角度变化 deq = q_prev - X(3)  （差分近似）

> 注意：如果希望改进控制性能，可考虑修正误差导数的计算方式、调整归一化范围与增益，或引入积分补偿作为混合控制策略。

## 如何修改常用参数

在 `inverted_pendulum_Fuzzy.m` 中直接修改相应变量：
- 仿真时长：`T_final`
- 采样步长：`Ts`
- 归一化因子：`x_normal`, `dx_normal`, `q_normal`, `dq_normal`, `u_normal`
- 控制增益：`gain_x`, `gain_q`
- 初始状态：`X`（在脚本前部）

修改后运行脚本会重新生成 `figures/` 下的图片。

## 许可证 & 作者

原始作者：Hyosung Hong（2017）
本仓库修改：2025-11-10

本项目仅供学习与研究使用。

---

**最后更新**: 2025年11月10日
