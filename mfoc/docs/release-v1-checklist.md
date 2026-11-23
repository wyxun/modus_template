# MFOC V1 来源、交付与复验

## 源码包内容

源码包根目录是 `mfoc/`。它包含 `mfoc.mk`、库配置、公共头文件、模块 C 源码、配置模板、接入和验证文档以及独立主机回放示例。各模块的 `include/` 中已文档化的入口构成源码接入面，`*_internal.h` 和实例内部字段由库实现维护。生成的对象文件、固件、台架记录原始数据、目标适配和第三方子模块留在主工程。验证文档里的某些历史图表或板级源码路径须回主工程查看。

`MFOC-PACKAGE.json` 记录导出工具可见的 Git `HEAD`、整个源码内容的 SHA-256 和各文件 SHA-256。Git 修订号用于定位来源，使用内容散列区分工作区改动和不同换行字节。文件路径用正斜杠，按路径的 Ordinal 顺序排序。内容散列来自不带 BOM 的 UTF-8 行：每行是小写散列、两个空格和路径，以 LF 结束。清单自身从内容散列中排除。需要确认交付身份时，和验收记录里的完整内容散列核对。

## 来源与许可证

| 组件 | 来源与包边界 |
| --- | --- |
| MFOC 库与主机示例 | 来自 `modus_template/mfoc`，确切内容由候选清单标识。现有仓库中尚无 MFOC 许可证正文。 |
| 独立构建工具与运行时 | 需要 C11、GNU Make、host GCC 及 C/数学运行库。验证记录应写明工具版本。这些工具和运行库由消费者提供。 |
| 可选 perfc PT 和 FSM | 来自 `modus/lib/perf_counter`。参考主工程 `.gitmodules` 和锁定的 `modus` SHA。仅启用 perfc 模式时由集成项目提供相应头、库及移植。 |
| G431 板级 SDK | 主工程使用 ST 的 `cmsis_device_g4`、`stm32g4xx_hal_driver` 和本地 `vendor/cortex-m/cmsis_core`。板测时记录主仓和子模块修订。 |
| AT32F413 板级 SDK | 主工程使用雅特力 `AT32F413_Firmware_Library` 子模块和本地 CMSIS Core。板测时记录主仓和子模块修订。 |

板级验收留档时用 `git rev-parse HEAD` 和 `git submodule status` 记录源码与子模块修订。CMSIS Core 随主工程提交记录。perfc 和 CMSIS Core 的部分头文件标注 Apache-2.0。板级 SDK 以各自代码及厂商声明为准。上述组件的许可声明不构成 MFOC 自身的许可。

独立源码包会原样复制 `mfoc/` 根的 `LICENSE`、`LICENCE`、`COPYING`、`NOTICE` 文件（如存在）。项目所有者须在对外发包前确认 MFOC 正式版本、许可证正文、第三方声明和分发渠道。主工程的旧 tag 不能作为 MFOC V1 版本。

## 导出并验实际文件

在主工程根目录运行：

```powershell
$candidate = Join-Path ([IO.Path]::GetTempPath()) 'mfoc-v1-candidate'
.\mfoc\tests\scripts\check_standalone.ps1 -OutputDirectory $candidate -ExportOnly
.\mfoc\tests\scripts\check_standalone.ps1 -VerifyPackage (Join-Path $candidate 'mfoc')
```

导出目录需为仓库之外的新目录。复验先核对完整文件清单，再从指定包复制到新的临时目录，仅使用副本运行主机示例、支持的 FLOAT/FIXED 配置和主机链接期配置错配用例。编译产物留在复验目录。省略参数会在一次调用里导出并复验。带 `-Quick` 只运行 FLOAT/FIXED 的 POSITION 组合及相应 Rs 回放，适合快速检查。

若以 ZIP 交付，复验先解包，并将解包后包含清单的 `mfoc/` 目录传给 `-VerifyPackage`。验收记录同时保存 ZIP 的文件散列和源码内容散列。

在另一台开发机重验时从待发布包的同一内容散列开始，先记录编译器和 GNU Make 版本，再核对 [接入步骤](getting-started.md)。最终文档、台架实测和源码包必须能够定位到同一候选及其板级构建配置。[主机回放说明](../examples/host_replay/README.md)

## 发布前核对

- [ ] 冻结 [支持矩阵](support-matrix.md)，明确每个组合的台架证据及使用范围。
- [ ] 部署同一最终候选并完成 G431 与 AT32F413 实机验收。记录板型、电机、编译器、配置和测量日志。
- [ ] 验证链接配置错配拒绝、Motor 初始化失败回滚、Init 栈和辨识公开入口。记录首次接入需要的变更。[扩展与迁移](extension-guide.md)
- [ ] 从最终分发物复验独立构建、链接和运行，记录包内容散列及 [变更记录](../CHANGELOG.md)。
- [ ] 由项目所有者确定正式版本、许可证和第三方声明后，执行实际版本标记和分发。
