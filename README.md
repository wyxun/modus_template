# MODUS Bare-Metal Template

本工程是一个面向 ARM Cortex-M（当前支持 STM32G431 与 AT32F421，支持多芯片无缝扩展）的纯裸机抽象开发模板。

此外，本模板采用 **Git Submodule (子模块)** 机制来统一管理第三方依赖（包括 CMSIS 核心库、厂家芯片库代码、以及 MODUS 框架本身）。这也是目前业界的主流演进趋势：**后续所有原厂固件库和第三方组件都直接通过 Git 仓库的形式被项目引用和同步更新**，告别手工复制粘贴代码。

## MStudio 上位机调试工具

本工程配套 **MStudio** —— 一款基于 ImGui + ImPlot + SDL2 的 Windows 桌面调试工作台，通过 OpenOCD Telnet / RTT 与目标板通信，提供以下六面板功能：

| 面板 | 功能 |
| :--- | :--- |
| **Dashboard** | 连接状态、采样率监控、波形控制、CSV 录制、Shell 宏管理 |
| **Waveform** | 实时多通道波形渲染与交互式测量（Space 差值、十字准心） |
| **Shell Terminal** | RTT Ch0 双向终端，支持过滤、自动滚动、历史指令 |
| **Registers** | 通过 OpenOCD Telnet 读取 Core/Special/FPU 寄存器，1s 自动刷新 |
| **Variables** | 加载 ELF 文件，按地址读取内存变量值，收藏夹 + 搜索 + 自动刷新 |
| **Map Analyzer** | 解析 GNU linker .map 文件，展示 Flash/RAM 占比、Section/File 明细、符号搜索 |

构建与运行：`tools/mstudio/` 下执行 `mingw32-make`，依赖 MSYS2 + mingw-w64-clang + SDL2。详细文档见 `tools/mstudio/docs/`。

---

## 1. 目录组织架构 (Directory Structure)
整个工程的目录被严格划分为以下几个层次，以保证架构的芯片解耦与高内聚：

- **`modus/`**：MODUS 核心框架模块（Git Submodule）。包含了 MDI（通用设备接口 Modular Device Interface）定义、底层协程、队列、以及内嵌的 `plooc`（面向对象 C 宏）、`perf_counter`（性能测量库）和 `Segger RTT`（日志输出库）、`mshell`（调试Shell）、`mwaveform`（实时波形）。
- **`vendor/`**：第三方厂商代码（Vendor SDKs），按内核架构分目录。
  - `vendor/cortex-m/` — ARM Cortex-M 芯片的官方固件库（CMSIS Core、STM32G4 HAL、AT32F421 标准库）及内核调试模块 `core_debug/`。
  - `vendor/riscv/` — 预留 RISC-V 架构支持。
- **`peripheral/`**：**底层硬件适配层**。该目录及其子目录隔离所有芯片依赖，上层业务只使用
  MDI 资源或 FOC 端口契约，不直接调用厂商库。
  - **`peripheral.h`**：暴露 `peripheral_Init()`、时钟查询等板级启动入口。
  - **`peripheral/<芯片代号>/mdi/`**：按模板组织 `backend.h`、`instance.h`、`state.c` 和
    `service.c`，用编译期资源绑定提供 GPIO、ADC/DMA、PWM、总线和字节流接口。
  - **`port_sys.c`**：只负责系统时钟、引脚和芯片外设初始化，以及启用中断；不创建运行时
    `HW` 资源池，也不承载 MDI 服务调度。
  - `mdi/service.c` 的 `mdi_Service()` 和 `mdi_Clock()` 由 MODUS 调用，负责 ADC DMA 发布、
    均值处理和轻量周期维护。旧的 `port_mdi.c`/`mdi_hw.h` 仅供尚未迁移的旧目标使用。
- **`src/` & `class/`**：**纯上层应用业务层**。
  - 包含主入口 `main.c` 以及各种高阶的抽象控制逻辑。
  - **开发禁令**：在这个层级**绝对禁止**包含诸如 `at32f421.h`、`stm32xxx.h` 这类厂商专有库，也**禁止**直接调用 `gpio_bits_write` 这类特有底层函数！
  - 业务逻辑初始化只需调用 `peripheral_Init()`；外设交互通过目标芯片提供的 `mdi/instance.h`
    资源 token 和 `MDI_*` 宏完成。FOC 业务只依赖 `foc_port.h`。
- **`build/`**：LLVM 工具链构建输出目录，生成最终的固件 (`.elf`, `.bin`, `.hex`)。
- **`tools/mstudio/`**：MStudio 上位机调试工具（Windows 桌面应用），包含波形分析、Shell 终端、寄存器查看、变量监控、Map 分析等面板。

---

## 2. 命名规范与风格 (Naming Conventions)
为了保持代码库的高度一致性和可读性，本项目强制推行以下 C 语言命名规范（主要影响全局/局部变量及结构体成员）：

### 变量前缀风格 (匈牙利命名法变体)
所有变量必须添加类型前缀，以小写字母开头，后接大驼峰（CamelCase），明确指示变量的基础类型或性质：

| 前缀 | 代表类型 | 示例 | 说明 |
|------|---------|------|------|
| `ch` | `char` 或 `uint8_t` | `chState` | 单字节字符、状态字或微小计数器 |
| `hw` | `uint16_t` (Half-Word) | `hwBufferSize` | 16 位整数 |
| `w`  | `uint32_t` (Word) | `wEvent` | 32 位无符号整数 |
| `n`  | `int32_t` / `int` | `nResult` | 32 位有符号整数，通常用于带负数的运算或错误码 |
| `l`  | `int64_t` (Long) | `lLastHeartbeat`| 64 位宽整数，如系统时间戳 |
| `f`  | `float` | `fAngle` | 单精度浮点数 |
| `d`  | `double` | `dTarget` | 双精度浮点数 |
| `b`  | `bool` | `bIsRunning` | 布尔值标志 |
| `p`  | Pointer (一般指针) | `pBuffer` | 一般类型的指针（基准前缀） |
| `pch`| `char *` / `uint8_t *` | `pchData` | 指向单字节/字符串的指针 |
| `phw`| `uint16_t *` | `phwBuffer` | 指向 16 位数据的指针 |
| `pw` | `uint32_t *` | `pwAdcRaw`| 指向 32 位数据的指针 |
| `pq` | `q_type *` | `pqResult` | 指向定/浮点数的指针 |
| `pt` | Pointer to Type | `ptThis`, `ptMotor`| 指向特定 `struct` 或 `typedef` 的对象指针 |
| `pfcn`| Function Pointer | `pfcnSetDuty` | 函数指针（较 `fn` 更强调指针属性，或直接用 `fn`） |
| `e`  | Enum | `eRunState` | 枚举类型的变量 |
| `q`  | `q_type` (本项目特有) | `qSpeedRef` | FOC 数学库的多态定/浮点数 |

### 修饰前缀
- `s_`：静态变量 (Static)，只在文件内部可见，必须与类型前缀组合使用（例：`s_lTimestamp`, `s_tFocAppBase`）。
- `g_`：全局变量 (Global)，必须与类型前缀组合使用（例：`g_chSystemData`）。
- `t` / `_t`：结尾表示 Type（如 `motor_handle_t`），作为变量名前缀表示这是一个实例化对象本身而非指针（特例，视习惯而定，如 `tGmsi`）。

### 函数风格
- **模块/类前缀**：大写或首字母大写下划线，如 `motor_Init()` 或 `foc_app_Start()`。
- **私有函数**：文件内 `static` 函数，正常小写下划线或前缀名加内部动作。

---

## 3. 跨平台框架如何开发？ (How to Develop)
基于本模板开发应用程序，你只需要遵循“**配置外设 -> 绑定 MDI -> 纯应用态编写**”规范：

### 步骤 1：针对目标芯片实现底层细节 (MDI Porting)
1. 在 `peripheral/<芯片代号>/mdi/instance.h` 中声明实际资源 token，并在 `backend.h` 中绑定
   寄存器、DMA 和外设能力。不要创建运行时设备编号、函数表或全局 `HW` 对象。
2. 在 `port_sys.c` 中完成时钟、GPIO、外设和中断初始化；在 `mdi/service.c` 中实现
   `mdi_Service()`/`mdi_Clock()`，把 DMA 发布、均值滤波和周期维护放在服务层。
3. 应用只包含 `mdi/instance.h`，直接使用对应的 `MDI_*` 宏。FOC 应用只包含 `foc_port.h`，
   芯片差异由目标目录下的 `foc_port.h` 和适配器处理。

### 步骤 2：上层应用业务逻辑与模块化 (Application Logic & Modularization)

本工程采用 **面向对象 C 语言 (OOPC)** 的思想。每个业务模块（如 `template_class.c`）都被视为一个独立功能的“类”。

#### 1. 核心接口分工 (Lifecycle)
每个业务模块应遵循标准的接口实现规范：
- **`Init`**: 初始化入口。在 `modus_Init()` 时被自动调用，用于挂载配置、初始化私有变量等。
- **`Run`**: 轮询入口。在 `main` 循环的 `modus_Run()` 中被调用。适合执行非阻塞的状态机或后台任务。
- **`Clock`**: 定时入口。由 `SysTick` (1ms) 驱动，在 `modus_Clock()` 中被调用。适合处理高精度计数或定时逻辑。

#### 2. OOPC 语法糖：使用 `this` 指针
为了提高代码可读性，建议在模块 `.c` 文件头部定义 `this` 宏：
```c
#undef  this
#define this (*ptThis)   /* 启用语法糖：this.member 等同于 ptThis->member */
```

#### 3. 模块自动加载 (MODUS_DECLARE_OBJECT)
**禁止在 `main.c` 中显式创建大量业务对象。** 统一在模块内部通过 `MODUS_DECLARE_OBJECT` 宏完成实例化与注册。
```c
// 在模块末尾一键加载：系统启动时会自动调用 template_class_Init
MODUS_DECLARE_OBJECT(template_class, TemplateClass,
    .pchRingBuffer = s_chBuffer,
    .hwRingSize = 128
)
```
- **机制**：通过 `init_infos` 内存段实现自动注册，无需修改 `main.c` 即可增加新功能模块。

#### 4. 状态机开发规范 (FSM)
应用层模块的状态机**必须**采用 **`perf_counter` 库的 PERF_COUNTER_FSM** 框架，**严禁使用裸 `switch-case`**。
```c
fsm_rt_t template_class_Run(template_class_t *ptThis)
{
    PERFC_PT_BEGIN(this.chState)    /* 恢复上次挂起点 */
    
    PERFC_PT_DELAY_MS(500)         /* 非阻塞延时 500ms */
    
    PERFC_PT_ENTRY(
        do_work();                  /* 进入该状态时执行一次 */
    )
    PERFC_PT_WAIT_UNTIL(is_done()); /* 等待条件成立 */
    
    PERFC_PT_YIELD(fsm_rt_on_going);/* 挂起并让出 CPU */
    
    PERFC_PT_END()                  /* 结束重置状态并返回 fsm_rt_cpl */
    return fsm_rt_cpl;
}
```

#### 5. 硬件调用：MDI 接口
业务层与硬件交互时，包含目标芯片的 `mdi/instance.h`，直接使用资源 token 和通用 MDI 宏：
```c
// 示例：状态灯和板级字节流
MDI_IO_Write(status_led, 1U);
MDI_STREAM_Write(board_stream, (const uint8_t *)"Hello", 5U);
```

| 常用宏 | 对应操作 |
|----|------|
| `MDI_IO_Write(resource, val)` | 写入 GPIO 或逻辑 IO |
| `MDI_IO_Read(resource)` | 读取 GPIO 或逻辑 IO |
| `MDI_STREAM_Write(resource, buf, len)` | 写入字节流 |
| `MDI_STREAM_Read(resource, buf, len)` | 读取字节流 |

> 完整宏定义参考 `modus/src/mdi/core/contract.h`、`modus/src/mdi/core/stream.h` 和对应 feature 头文件。
 
 ### 步骤 3：项目依赖与功能伸缩调整 (Makefile Toggling)
 工程采用了灵活自由的 Makefile 管理。如果你在开发中启用了诸如 I2C 或 SPI 这种原本关闭的内置外设：
 - 请在根目录下的 **`makefile`** 中，搜寻并把对应的 `USE_DRV_XXX` 标志置为 `1`。
 - Makefile 系统会自动引入对应厂家的源文件参与最终编译。
```makefile
TARGET_CHIP ?= at32f421
```
当后续添加如 `stm32g4` 时，只需：
1. 建一个 `peripheral/stm32g4/mdi/`，提供 `backend.h`、`instance.h`、`state.c` 和
   `service.c`；系统初始化放在同级 `port_sys.c`。
2. 在 `makefile` 中增加一段 `ifeq ($(TARGET_CHIP),stm32g4)` 用来限定包含了它的驱动底层源文件路径及宏定义。
3. **只需在命令行调用 `make TARGET_CHIP=stm32g4` 即可一键拉起编译，所有应用层代码完美重用！**

同时，若在使用 AT32 期间启用了诸如 I2C/SPI，只需在 Makefile 内的专属外设开关寻找 `USE_DRV_XXX=0` 修改为 `1` 即可链接对应厂商源文件。

---

## 4. 编译模式与优化等级 (Build Modes & Optimizations)

为了完美应对微秒级 FOC 电机驱动的时序要求和逻辑开发，项目独创性地支持 **“双轨制构建”** 体系：

| 构建模式 | 命令行参数 | 优化等级 (`OPT`) | 调试模块 (`mshell/MLOG/Waveform`) | 最佳应用场景 |
| :--- | :--- | :--- | :--- | :--- |
| **Debug (开发轨)** | `.\make.bat` / `make` | **`-O0`** (无优化) | 🟢 开启 | 适用于纯逻辑算法调试。GDB 单步不断点、不跳行，所有局部临时变量 100% 可读，绝无 `<optimized out>` 报错。 |
| **Debug-Rel (过渡轨)** | `.\make.bat debug-rel` | **`-Oz`** (极致体积/速度) | 🟢 开启 | **发版前黄金预检模式**。拥有与生产环境 100% 绝对一致的运行速度与指令时序，防范时序炸管，但同时保留 Shell、RTT 打印与 `mstudio` 高速波形捕获！ |
| **Release (生产轨)** | `.\make.bat release` | **`-Oz`** (极致体积/速度) | 🔴 彻底剥离 (无开销) | 最终量产发版。所有调试逻辑、打印代码在汇编层被物理切除，获得最小的体积与最高效的硬件执行速率。 |

---

## 5. 编译、烧录与一键流 (Compilation, Flashing & Workflows)

### 5.1 极简命令行流
在任何终端（PowerShell 或 CMD）直接执行以下快捷批处理指令：
```powershell
.\make.bat clean              # 清理 build 目录
.\make.bat [BUILD_MODE]       # 执行编译，不传参数默认是 debug 模式，可传入 release / debug-rel 
.\make.bat flash BUILD=...    # 将相应生成的 hex 直接烧录进入目标板 (默认烧录 debug 固件)
.\make.bat download           # 🚀 免除依赖检查：直接调用 OpenOCD 刷写已有的固件 hex
```

### 5.2 🚀 一键自动化测试流水线 (`.\make.bat auto`)
如果你希望以 **`-Oz` 强优化时序** 进行测试（最推荐的调试电机模式），只需运行一键自动化指令：
```powershell
.\make.bat auto
```
该指令将以极速在后台自动完成以下全闭环操作：
1. **自动强杀** 任何残留占用调试口的后台 OpenOCD 进程（杜绝端口冲突）。
2. **清理** 并将项目编译为 **`-Oz` 极致优化的 `debug-rel`** 固件。
3. **安全下载** 固件写入单片机 Flash。
4. **后台静默挂载** RTT 服务器，自动将电机波形数据映射至本地 **`127.0.0.1:9091`** 端口！
* **结果**：指令运行完毕后，不需手动运行任何程序，直接打开 **`mstudio`** 软件连接 `127.0.0.1:9091` 端口，最真实的电机高速 SVPWM、正余弦波形即刻直观呈现！

---

## 6. VSCode Cortex-Debug 调试说明

本工程已完美适配 VSCode 的 **Cortex-Debug** 插件，通过集成 RTT console 与 SVD 外设寄存器描述文件，实现极其便捷的图形化断点调试。

### 6.1 环境准备
- **插件安装**：请从 VSCode 插件市场安装 [Cortex-Debug](https://marketplace.visualstudio.com/items?itemName=marus25.cortex-debug)。
- **路径确认**：确保 `.vscode/launch.json` 中的 `serverpath` (OpenOCD 路径) 与 `armToolchainPath` (LLVM 环境路径) 指向你电脑上的实际安装目录。

### 6.2 开发与调试流程
1. **启动调试**：按下快捷键 **`F5`**。
2. **自动构建**：系统会自动触发 `Build ELF` 任务（调用 `make.bat build`）确保代码是最新的。
3. **固件植入**：底层自动启动 OpenOCD 将编译好的 `.elf` 固件下载至芯片（如 STM32G431），并默认停在 `main` 函数入口。
4. **实时日志**：在 VSCode 的 **DEBUG CONSOLE (调试控制台)** 中，可以直接实时查看 `MLOG` 产生的 RTT 日志。

### 6.3 特色高级功能
- **外设寄存器查看**：在调试模式下，左侧侧边栏底部会出现 **CORTEX PERIPHERALS** 面板。依靠项目自带 SVD 文件，你可以直观查看所有外设（如 CRM, TMR, ADC, GPIO）的实时寄存器位。
- **变量监控与断点**：支持标准的断点调试、全路径调用堆栈跟踪以及变量 Watch 监视（注：如需顺畅单步调试，请在 `BUILD=debug` 模式下运行，避免优化干扰）。

---

## 7. AI 辅助调试：`AITrace` 命令行调试工具 (AI-Driven Debugging with AITrace)

项目在 `./tools/aitrace.exe` 目录下内置了极其强大的无 GUI 命令行调试工具 `AITrace`。该工具不仅供工程师手工调测，更专为 AI 助理（如 Antigravity / Claude）提供直接窥视 MCU 内部运行状态的“天眼”。

### 7.1 侵入安全红线
由于 FOC 电机驱动系统属于大电流强实时功率电路，**严禁在电机旋转时进行 CPU 挂起**（否则会导致 MOS 管直通烧毁）。AITrace 严格划分了侵入等级：
* **A. 被动级 (无开销/绝对安全)**：`shell` 命令与 `wave` 采集，通过 RTT 通道零开销交互。**电机运行时首选**！
* **B. 暂停级 (暂停 1秒)**：`ocd` 指令读核心寄存器和内存，会短暂挂起 CPU。**仅在电机静止时允许使用**！
* **C. GDB级 (完全挂起)**：`gdb` 连接单步。**仅在逻辑仿真时允许使用**！

### 7.2 黄金被动诊断命令 (A 级安全)
```powershell
# 1. 抓取电机波形通道列表
./tools/aitrace.exe wave list

# 2. 捕获 5 秒钟高速电机波形输出并保存为 CSV (供 AI 调参或诊断 PID 震荡)
./tools/aitrace.exe wave capture 5 --output motor_wave.csv

# 3. 免挂起读取单片机当前挂载的 MODUS 活跃对象
./tools/aitrace.exe shell list

# 4. 被动查看故障状态寄存器
./tools/aitrace.exe shell cfsr
```

### 7.3 ⚡ HardFault 死机秒级定位
如果单片机在运行中偶发性死机：
1. 固件中的异常服务会自动捕获现场，并将崩溃现场的 `PC` (程序计数器)、`LR` (连接寄存器) 等通过 RTT 打印出来。
2. 将这几个十六进制寄存器发给你的 AI 助理，AI 将在终端一键解析：
   ```powershell
   ./tools/aitrace.exe crash report --pc=<PC> --lr=<LR> --sp=<SP> --elf=build/template.elf
   ```
3. **AI 将在一秒钟内直接指出是你的哪一个 C 语言源文件、第几行代码、因为何种硬件异常（除零/对齐错/野指针）导致的崩溃！**

---

## 8. Git 子模块与极速推拉管理 (Git Submodules & High-Speed Mirroring)

由于外设底层、核心 CMSIS、MODUS 均属于独立外链依赖仓库，采用子模块（Submodule）进行高度解耦管理。

### 8.1 首次克隆与极速更新

**首次克隆本仓库架构：**
若要获得极速拉取体验，强烈建议使用国内镜像源克隆，且加上 `--recursive` 与限制深度 `--depth=1` 参数：
```bash
# 使用镜像源进行极速首次克隆
git clone --recursive --depth=1 https://ghfast.top/https://github.com/wyxun/modus_template.git
```

**忘记带参的自救更新指令：**
如果克隆时漏掉了依赖子模块，或者要对它们进行强力提速拉取：
```bash
# 只拉取最新的一次 Commit，避免拉取数十兆的历史垃圾文件，速度暴涨 10 倍！
git submodule update --init --recursive --depth=1
```

**对底层库版本的强制追更：**
```bash
cd modus
git checkout dev
git pull origin dev
# 强制使用深度限制进行子模块更新
git submodule update --init --recursive --depth=1
cd ..
```

---

### 8.2 🚀 终极加速实践：Git “推拉分流” 双轨机制

在日常开发中，由于网络干扰，GitHub 经常面临“拉取极慢、推送卡死”的物理死穴。项目为此深度定制并推荐采用 **“推拉分流” (Fetch/Push Splitting)** 方案：

* **核心原理**：
  - **拉取 (Fetch / Pull)**：只读操作，直接走国内的高速加速镜像 `ghfast.top`，**免登录鉴权，实现 MB/s 级极速拉取子模块**。
  - **推送 (Push)**：必须有写权限，走 GitHub 官方物理写通道 `github.com`，搭配本地代理与凭据提供商设置，**实现秒级秒开验证，安全推送**。

**极速分流一键配置命令：**
```powershell
# 1. 设置 pull（fetch）使用镜像加速源，确保无感极速更新
git remote set-url github https://ghfast.top/https://github.com/wyxun/modus_template.git

# 2. 专门设置 push 推送目标为 GitHub 官方源（写物理通道）
git remote set-url --push github https://github.com/wyxun/modus_template.git

# 3. 指定 GitHub.com 的凭据提供商，跳过无谓的2秒网络嗅探，杜绝登录弹窗卡死
git config --global credential.https://github.com.provider github
```

运行完后，在根目录下输入 `git remote -v`，你将看到堪称完美的**推拉双轨状态**：
```text
github  https://ghfast.top/https://github.com/wyxun/modus_template.git (fetch)
github  https://github.com/wyxun/modus_template.git (push)
```

**配置本地代理通道 (如果你本地开着代理软件)：**
```powershell
# 让你的 Git 物理网络直连本地代理（如 Clash 默认的 7890 端口），让官方写通道推送瞬间起飞
git config --global http.proxy http://127.0.0.1:7890
```
