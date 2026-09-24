# 2026 年电赛 H 题：MSPM0G3507 滚球控制固件

本项目是 2026 年全国大学生电子设计竞赛 H 题滚球控制系统的微控制器端固件，涵盖第 2 至第 6 问的控制功能。第 1 问属于独立视觉侧；本仓库包含微控制器端 UART3 接收与视觉帧有效性处理，不包含视觉模块算法或固件。

## 项目概览

- 微控制器：TI MSPM0G3507，采用裸机协作式主循环与定时器控制节拍。
- 执行器：X42S 步进电机，使用 X 固件协议；底盘驱动与摆杆执行器为独立软件路径。
- 应用层：`user/app/q3`、`user/app/q4`、`user/app/q6`；按键、显示与遥测适配模块位于 `user/app`；板级服务位于 `user/bsp`；设备驱动与协议代码位于 `user/code`。
- `main.c` 负责系统启动、按键到模式的路由，以及原有关键任务的调用顺序。中断事件去抖、Q4/Q5/Q6 状态显示和周期诊断调度分别封装在 `user/app/app_key_input.*`、`user/app/app_status_display.*` 和 `user/app/app_telemetry.*`。
- 外设配置以 `empty.syscfg` 为源；当前工程配置对应的 `ti_msp_dl_config.c/.h` 已纳入仓库，不应手工编辑。
- 构建系统：CMake、Ninja 与 Arm GNU 工具链；提供 Debug 和 Release 预设。

## 固件功能

| 题目 | 代码位置 | 功能范围 |
|---|---|---|
| 第 2 问 | `user/bsp/lap_trace.*`、`user/main.c` | 通过现有按键流程选择底盘绕行或路径运行任务。 |
| 第 3 问 | `user/app/q3/` | 球状态估计、位置控制、摆杆几何与标定、第 3 问状态流程。控制行为与已调参数保持不变，仅规范源码格式。 |
| 第 4 问 | `user/app/q4/q45_vehicle_control.*` | 静态底盘路径运行，以及滚球恢复与保持控制。 |
| 第 5 问 | `user/app/q4/q5_*`、`q45_vehicle_control.*` | 动态底盘与滚球控制，以及由运动状态产生的前馈。 |
| 第 6 问 | `user/app/q6/`、`q45_vehicle_control.*` | 捕获滚球稳定起始位置或接收显式目标，并进行相对目标的闭环控制。 |
| 硬件与协议 | `user/code/`、`user/bsp/` | X42S、视觉串口帧、按键、显示、电机、编码器、陀螺仪、时钟与调试遥测。 |

第 4、5、6 问共用车辆控制状态机。第 6 问的目标捕获与专属调节由第 6 问路径隔离；本项目没有重写已经验证的第 3 至第 5 问控制律。

第 2 问路径与第 6 问启动目前共用 PB24 输入，并由编译期宏 `Q6_TASK_ENABLE` 选择：启用第 6 问时，PB24 启动第 6 问；关闭时，PB24 进入原有独立第 2 问路径。因此同一固件配置下，这两个任务不会同时由该按键选择。

当前源码中的按键对应关系：PB21 停止，PB11 选择第 5 问，PB24 选择第 6 问或第 2 问（由编译期配置决定），PA28 选择第 4 问，PA31 执行第 3 问归零或启动流程。上电与接线前请阅读[硬件说明](docs/hardware.md)。

## 构建环境

安装 CMake 3.22 或更新版本、Ninja 与 Arm GNU 工具链。当前 CMake 预设面向 Windows。配置前在终端设置 `ARM_GCC_ROOT` 与 `NINJA_ROOT`；VS Code 的 SysConfig 辅助脚本还会从本地 `.env` 文件读取 `MSPM0_SDK_ROOT` 与 `SYSCONFIG_ROOT`。可参考 `.env.example` 中的变量名称；真实 `.env` 文件已忽略，不应提交。

`tools/` 下可选的串口采集工具使用 pySerial。可通过 `python -m pip install -r tools/requirements.txt` 安装固定版本依赖；公开源码中不包含 pySerial 副本。

在已设置上述环境变量的 PowerShell 中运行：

```powershell
$env:ARM_GCC_ROOT = "C:\Toolchains\arm-gnu-toolchain"
$env:NINJA_ROOT = "C:\Tools\ninja"
cmake --preset debug-armgcc
cmake --build --preset build-debug
```

当前 CMake 工具链文件面向 Windows。以上命令对应仓库内的构建入口。第 6 问台架宏与正常运行配置相互独立；生成用于实机的固件前，请检查 `user/app/q6/q6_config.h` 与 CMake 缓存。

运行软件回归程序前，通过 `python -m pip install -r tests/requirements.txt --target build/testdeps` 安装固定版本的 Python 仿真依赖，再运行[测试与证据说明](docs/testing.md)中列出的 PowerShell 脚本。该流程不覆盖真实传感器、电机、接线或实时运行时序。

## 硬件与安全

当前 SysConfig 中，UART0 对应 `PRINT`（PA0/PA1），UART1 对应 `DEBUG`（PB6/PB7），UART2 对应 `GYRO`（PA21/PA24），UART3 对应 `VISION`（PA25/PA26），TIMG6 对应 `Timer_Control`。连接外设前请查阅[硬件说明](docs/hardware.md)，并核对板卡接线；生成配置中的外设名称必须与实际接线一致，不能根据旧日志推测端口职责。

固件收到启动请求后可能立即驱动电机。测试时应固定底盘、保持滚球路径畅通，并确保物理断电手段可用。不要将带有台架自动启动配置的固件用于正式比赛。

## 验证状态

用户报告第 5 问与第 6 问已通过实机验收。本次代码审计中，已运行的主机回归程序结果为：按键输入 9 项、遥测 14 项、第 5 问 5,664 项、第 6 问目标与静态镜像测试分别为 235 项和 39 项；Debug ARM 固件构建成功，Flash 使用量为 100,968 / 131,072 字节，SRAM 使用量为 17,672 / 32,768 字节。

目前尚未将已验收的实机固件与本仓库源码快照建立可追溯对应关系；本次审计也未重新烧录或进行实机试验。验证范围与限制见[测试与证据说明](docs/testing.md)。

## 详细说明

- [架构与模块职责](docs/architecture.md)
- [源码与构建基线](docs/baseline.md)
- [硬件与外设归属](docs/hardware.md)
- [控制流程](docs/control-algorithm.md)
- [测试与证据](docs/testing.md)
- [代码风格与格式](docs/code-style.md)
- [调试案例](docs/debugging-case-studies.md)
- [已知限制](docs/known-limitations.md)
- [第三方声明](THIRD_PARTY_NOTICES.md)

## 项目经历简介

围绕 2026 年电赛 H 题，实现并调试基于 MSPM0G3507 的滚球控制系统，集成视觉串口反馈、X42S 步进电机控制、状态估计、目标捕获与第 2 至第 6 问的控制流程。具体性能数据应与对应日期的日志和实机条件一起说明。
