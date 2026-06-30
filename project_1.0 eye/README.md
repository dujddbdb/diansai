# STM32F407 电赛工程

## 快速开始（首次使用）

### 1. 双击运行 `setup_env.bat`

这会自动完成：
- 复制 ARM GCC 工具链到 `tools/arm-gcc/`
- 创建 Python 虚拟环境并安装 pyOCD
- 安装 STM32F407 CMSIS Pack

> 如果脚本提示找不到 ARM GCC，按提示下载后放到 `tools/arm-gcc/` 目录。

### 2. 安装 VSCode 插件

在 VSCode 中安装以下插件：
- **Cortex-Debug** (`marus25.cortex-debug`)

### 3. 连接硬件，开始调试

1. 用 DAPLink 连接 STM32F407 开发板
2. 打开 VSCode，按 **F5** 开始调试
3. 程序会自动烧录并运行到 `main()` 断点

---

## 目录结构

```
project_1.0/
├── .vscode/              # VSCode 配置
│   ├── launch.json       # 调试启动配置
│   ├── settings.json     # 工作区设置
│   ├── tasks.json        # 构建任务
│   └── STM32F40x.svd    # 外设寄存器描述文件
├── app/                  # 应用层代码
├── board/                # 板级驱动
├── bsp/                  # 底层驱动 (外设/传感器)
├── libraries/            # 标准库 (CMSIS/STD Periph)
├── module/               # 功能模块
├── project/              # Keil MDK 工程
│   └── MDK(V5)/
│       └── Objects/
│           └── Project.axf  # 调试用的可执行文件
├── tools/                # 便携开发工具 (git 忽略)
│   ├── arm-gcc/          # ARM GCC 工具链
│   └── python_venv/      # Python 虚拟环境 + pyOCD
├── setup_env.bat         # 环境一键配置脚本
└── README.md
```

---

## 调试配置说明

`launch.json` 所有路径都使用 `${workspaceFolder}` 相对变量，任意电脑上无需改配置即可使用：

- **调试器**：pyOCD (通过 DAPLink/CMSIS-DAP)
- **芯片**：STM32F407VETx
- **频率**：100kHz (稳定兼容 DAPLink v1)
- **入口**：自动运行到 `main()`

---

## 常见问题

### Q: 调试报 "Failed to launch PyOCD GDB Server: Timeout"
- 检查 DAPLink 是否正确连接
- 检查 USB 线是否支持数据传输（非仅充电线）
- 在设备管理器确认能看到 "CMSIS-DAP" 设备

### Q: 调试报 "No probes found"
- 重新插拔 DAPLink
- 运行 `tools\python_venv\Scripts\pyocd.exe list` 查看探针状态

### Q: 需要在新电脑上使用
- 拷贝整个工程文件夹
- 双击运行 `setup_env.bat`
- 完成！
