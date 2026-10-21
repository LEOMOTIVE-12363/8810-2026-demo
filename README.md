[English](README.en.md) | **简体中文**

# FRC 12363 LEOMOTIVE · 2026「REBUILT」机器人代码

![Java](https://img.shields.io/badge/Java-17-ED8B00?logo=openjdk&logoColor=white)
![WPILib](https://img.shields.io/badge/WPILib-2026-6E3496)
![AdvantageKit](https://img.shields.io/badge/AdvantageKit-26.0.2-1D9BF0)
![Phoenix 6](https://img.shields.io/badge/CTRE_Phoenix_6-TalonFX%20%2F%20Pigeon%202%20%2F%20CANrange-D2222D)
![maple-sim](https://img.shields.io/badge/maple--sim-0.4.0--beta-2EA043)

由 FIRST® Robotics Competition 战队 [**12363 LEOMOTIVE**](https://github.com/LEOMOTIVE-12363) 维护的 2026 赛季（游戏 **REBUILT**）机器人代码。本项目以 **8810 Alphabots 的机器人及其开源代码**为基础，由 LEOMOTIVE 持续优化与开发，形成我们自己的版本。技术基础：Java 17 · WPILib 命令式框架 · AdvantageKit TalonFX swerve 模板。

> **项目归属与上游来源**：本仓库属于 **12363 LEOMOTIVE**，是我们独立维护的衍生版本；**8810 Alphabots** 是原始机器人与上游代码的来源。代码 fork 自 [Alphabots-8810/8810-2026-offseason](https://github.com/Alphabots-8810/8810-2026-offseason)。`main` 保留上游竞赛代码基线及本 fork 的许可、文档与仓库卫生改动，`demo` 承载 LEOMOTIVE 当前的纯手动演示版本开发，详见[分支模型](#分支模型)。

## 功能亮点

以下为 `main` 保留的上游竞赛代码能力；LEOMOTIVE 当前在 `demo` 线上进行纯手动演示版本的适配与改进，两条分支的功能范围见分支模型：

- **Swerve 底盘** — CTRE TalonFX + Pigeon 2，硬件常量由 CTRE Tuner X 生成（`generated/TunerConstants`），Phoenix 高频里程计线程，自带 SysId 标定例程
- **FUEL 机构链** — 可展开 intake（位置控制臂 + 滚轮）→ hopper / indexer（CANrange 传感器检测存球）→ Drum 发射鼓 + 角度可调 Hood
- **战术程序** — HUB 射门（支持移动中射门）、中立区 Ferry 吊射、Trench 隧道通过（驾驶辅助版 + [Autopilot](https://github.com/therekrab/autopilot) 全自动版）
- **视觉** — Limelight（MegaTag2）全局位姿估计，融入 swerve 里程计
- **AdvantageKit 三模式** — REAL / SIM / REPLAY 共用一套代码，日志可确定性回放
- **maple-sim 桌面仿真** — 2026 REBUILT 场地（含 HUB / Trench 碰撞体）物理仿真
- **Elastic 自定义仪表盘** — 布局版本化于 `elastic-layout.json`；配套 NetworkTables 调参脚本 `scripts/nt_tune.py`、`scripts/pid_analysis.py`

## 分支模型

| 分支 | 内容 |
|---|---|
| `main` | 上游 Alphabots 8810 竞赛代码基线，保留本 fork 的 MIT LICENSE、双语 README 与私有配置退跟踪改动：完整自动程序（Choreo 轨迹 + PathPlanner）、视觉、战术程序 |
| `demo` | **LEOMOTIVE demo 展示线**（本仓库的日常开发线）：移除全部自动程序与视觉，纯手动驾驶；Elastic 滑条直控 Drum 转速 / Hood 俯仰，并新增 demo 调参页 |

- demo 展示线的功能改动**永不合入 `main`**；每个任务一个独立 PR，base 为 `demo`，自检通过后即合入。仓库共用的中英 README 同步维护：从 `demo` 发起文档改动，再以独立 PR 将相同文案同步至 `main`。
- `demo` 相对 `main` 的全部差异见 [PR #2](https://github.com/LEOMOTIVE-12363/Alphabots-2026-lm/pull/2)——长期保持 open，仅作差异总览。

## 构建与仿真

```bash
./gradlew build          # 编译 + spotless 格式检查
./gradlew spotlessApply  # 自动格式化（Google Java Format），提交前运行
./gradlew simulateJava   # 桌面仿真（maple-sim 物理）
./gradlew replayWatch    # AdvantageKit 日志回放
./gradlew deploy         # 部署到 roboRIO
```

仓库提供 GitHub Actions 构建工作流，配置为在 push / pull request 时运行 `./gradlew build`；实际是否运行取决于本 fork 的 Actions 启用状态。提交前需在本机完成构建与格式检查。

## 代码架构

- `frc.lib` — 团队通用硬件抽象层：`MotorIO`（电压 / 速度 / 位置 / 力矩电流控制，含 TalonFX 与仿真实现）、`SensorIO`（CANrange）、`MotorSubsystem` 基类；IO 输入以**弧度**记录日志，对子系统 API 以**圈数**暴露。
- `frc.robot.subsystems` — 单例模式（每个子系统 `public static final mInstance`），`X.java` + `XConstants.java` 成对（CAN ID、增益、仿真配置）。
- `frc.robot.commands` — 每个命令独立目录，各带 `*Constants.java`。
- 三种运行模式由 `Constants.currentMode`（REAL / SIM / REPLAY）切换，各子系统按模式选择 IO 实现。
- `elastic-layout.json` — Elastic 仪表盘布局，与代码内 NetworkTables 接口同步维护。

## 开发流程与决策记录

- **提交约定**：遵循 [Conventional Commits](https://www.conventionalcommits.org/)——`feat:` / `fix:` / `docs:` / `chore:`，可带 scope（如 `feat(drive): ...`）。
- **决策记录**：重大改动的原因（为何 demo 只保留手动、为何 LICENSE 常驻 main 等）见 [`docs/decisions/`](docs/decisions/)——每个决策一个短文件（日期 + 状态 + 背景 + 决定）。

## 致谢与参考

- [FRC 8810 Alphabots](https://github.com/Alphabots-8810/8810-2026-offseason) — 本项目所基于的原始机器人与上游代码；感谢其开源工作
- [WPILib](https://docs.wpilib.org) — FIRST® 机器人软件框架
- [AdvantageKit](https://docs.advantagekit.org) / [Team 6328 Mechanical Advantage](https://github.com/Mechanical-Advantage) — 日志与回放框架、swerve 模板出处
- [maple-sim](https://github.com/Shenzhen-Robotics-Alliance/maple-sim)（Shenzhen Robotics Alliance）— 2026 REBUILT 场地物理仿真
- [CTRE Phoenix 6](https://v6.docs.ctr-electronics.com) — TalonFX / Pigeon 2 / CANrange
- [PathPlanner](https://pathplanner.dev) · [Choreo](https://choreo.autos) — 路径规划与轨迹（`main` 线）
- [Autopilot](https://github.com/therekrab/autopilot) — Trench 自动通过运动库（`main` 线）
- [Elastic Dashboard](https://github.com/Gold872/elastic-dashboard) — 操控端仪表盘
- [FIRST® Robotics Competition](https://www.firstinspires.org/robotics/frc/game-and-season) — 2026 REBUILT 赛季与游戏手册

## 许可

本项目采用 [MIT License](LICENSE)，保留其中现有的 FRC Team 8810 Alphabots 版权声明。该声明用于保留来源归属；本衍生版本由 FRC 12363 LEOMOTIVE 维护和继续开发。依赖协议另见 [WPILib-License.md](WPILib-License.md) 与 [AdvantageKit-License.md](AdvantageKit-License.md)。
