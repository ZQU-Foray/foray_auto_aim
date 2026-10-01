# 目录结构说明

## 目录结构

```
.
├── README.md                本仓定位、接口、上下游
├── AGENTS.md                AI 协作规范
├── plan.md                  开发计划与进度
├── decision.md              关键工程决策日志
├── tree.md                  本文件
├── CODEOWNERS               Review 自动分派
├── package.xml              ROS 2 包定义（ament_cmake）
├── CMakeLists.txt           构建与测试目标
├── .clang-format            组织 C/C++ 风格（CI 的 lint-c 依据）
├── .foray-layer             所属层与依赖边界（CI 校验）
├── .gitignore
├── include/
│   └── foray_auto_aim/      对外头文件
│       ├── gimbal_command.hpp     云台角指令结构
│       ├── aimer.hpp              云台角解算接口（当前直瞄；弹道解算待云台就绪）
│       └── target_candidate.hpp   候选目标结构 + 友军过滤接口
├── src/
│   ├── aimer.cpp            云台角解算实现（直瞄；弹道解算留待云台就绪）
│   └── filter_friendly.cpp  友军过滤实现
├── test/
│   ├── test_aimer.cpp             云台角解算用例（5 条）
│   └── test_filter_friendly.cpp   友军过滤用例（7 条）
└── .github/
    └── workflows/
        └── ci.yml           复制自组织 CI 模板
```

> 已落地两个模块：① **候选目标结构 + 友军过滤**（7 条测试）② **云台角解算（直瞄）**（5 条测试；**弹道解算待云台就绪后再实现**，接口已留）。
> `config/`（参数 YAML）随多源融合与云台参数落地后补记。

## 记录约束

MUST NOT 记录以下内容：

- `build/`、`install/`、`log/`、`.git/` 等依赖与产物目录
- 临时文件、缓存文件、日志文件
