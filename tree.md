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
├── CMakeLists.txt           构建与测试目标（源码、测试落地后在此追加）
├── .clang-format            组织 C/C++ 风格（CI 的 lint-c 依据）
├── .foray-layer             所属层与依赖边界（CI 校验）
├── .gitignore
└── .github/
    └── workflows/
        └── ci.yml           复制自组织 CI 模板
```

> 当前仅含包骨架（`package.xml` / `CMakeLists.txt` / `.clang-format`），**尚无源码目录**。
> 源码（`include/foray_auto_aim/`、`src/`）与测试（`test/`）随模块落地后在此补记。

## 记录约束

MUST NOT 记录以下内容：

- `build/`、`install/`、`log/`、`.git/` 等依赖与产物目录
- 临时文件、缓存文件、日志文件
