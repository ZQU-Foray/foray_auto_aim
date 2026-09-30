# 关键工程决策日志

> 记录 fork patch、换版本、**破坏性接口变更**、架构取舍。
> 格式见 `foray_docs/repository_structure.md` §7.2。

---

## Decision

Date: 2026-02-19

Context: 本仓从 0 初始化。需要确定它在整体架构中的位置与依赖边界。

Decision: 本仓定位为 **L4** 层，owner 角色为「视觉」，
依赖层级 X2、L1、L2、L3、L4。

Reason:
- 切分判据 C1–C6 见 `foray_docs/repository_structure.md` §1
- 粒度结论：**一个仓 = 一个团队角色 + 一个变更率档位**
- 消费融合后的目标，**不关心来源**。跟踪环是硬实时

Alternatives:
- 并入相邻仓（减少仓库数量，但违反团队边界或变更率差异）

Rejected:
- 按 ROS 2 package 粒度切仓 —— 会导致版本矩阵地狱
- 按分层机械切仓 —— 会切断团队边界（Conway）

---

## Decision

Date: 2026-09-28

Context: 需要让 `colcon` 与组织 CI 识别本仓，并确定源码、参数与测试的目录约定。

Decision:
- 构建：**ament_cmake**，C++17
- 目录约定：`include/foray_auto_aim/`（对外头文件）· `src/`（实现）· `config/`（参数 YAML）· `test/`（每模块一个测试）
- C/C++ 风格沿用 `foray_interfaces` 的 `.clang-format`（4 空格 · LF · UTF-8）

Reason:
- 组织 `build-ros2` 作业以 `package.xml` 是否存在为开关；缺该文件时构建与测试会被整段跳过
- 兵种差异落在 `config/`，不进算法代码（`foray_docs/algorithm_structure.md` §1.2）
- 每模块带独立可执行测试是 `repository_structure.md` §9 验收标准 1 的落地方式

Alternatives:
- 沿用 `foray_interfaces` 的独立可执行测试 + 自定义 CI 作业（不建 ROS 2 包）
- 单一 `src/` 平铺，不区分 `include/` 与 `src/`

Rejected:
- 不建 `package.xml`、仅靠自定义脚本编译 —— 组织 CI 与 rosdep 均无法介入
- 自建一套代码风格 —— 与组织 §6.6 及其余仓分叉
