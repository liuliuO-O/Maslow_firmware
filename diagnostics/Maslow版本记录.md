# Maslow 固件版本记录

更新于 2026-11-12，时间均为北京时间。范围包括固件库可追溯的源码提交、保留的 S3 应用镜像、网页部署和有日志支持的实机校准。Git 提交标识源码，镜像 SHA-256 标识实际构建产物。候选版、已刷写、已完成实机校准和已验证实际运动分别按各自证据记录。

## 源码提交

| 日期 | 提交 | 改动 |
|---|---|---|
| 10-29 | `e55fd97` | 导入现有 Maslow 固件基线。 |
| 11-01 | `e9a39bf` | 保存右上编码器方向修正及被动校准诊断，记录各阶段和四带反馈。 |
| 11-01 | `b553820` | 修复水平校准重新驱动已拉紧皮带的问题，加入回归检查。 |
| 11-02 | `172ec33` | 保存左右上组件互换后的方向设置、AS5600 和校准诊断。 |
| 11-02 | `b095b68` | 调整高速编码器转动的跨圈累计逻辑并添加测试。 |
| 11-02 | `fa3a241` | 移除根据电机方向推断编码器跨圈的逻辑。 |
| 11-02 | `f517ddf` | 恢复左右上组件正常接线对应的编码器方向。 |
| 11-02 | `0e70bc9` | 在角度跳变时记录 AS5600 磁场状态，辅助区分编码器和机械异常。 |
| 11-03 | `631a427` | 修正右下编码器方向。11-12 上午的旧版回退日志也显示 `631a427-dirty`，该标识包含未提交的构建差异。 |
| 11-11 | `fe52ac1` | 汇总 17 点建锚、采样窗口、水平前六点固定参考保位、已保存锚点恢复、五轴边界与停止/通道修复，并保留相关回归。早期日志中的 `ff9a57a` 与此提交的 Git 源码树一致。 |
| 11-12 | `a705b50` | 汇总运行日志精简、网页 UTF-8、状态消息、校准移动异常摘要以及已保存锚点恢复改进。`aa41885` 与此提交的 Git 源码树一致。 |

## 镜像和部署

下面的 `release/` 指 `D:/MaslowDev/projects/Maslow_firmware/release`，`backups/` 指 `D:/MaslowDev/backups/Maslow_firmware`，`logs/` 指 `D:/MaslowDev/logs`。11-11 的三个 `r1` 独立镜像另存于 `C:/Users/66976/Documents/ChatGPT/优化`。同一提交之后的多次构建可以包含不同的工作树内容，以对应镜像和实机日志为准。

| 日期 | 版本 | 改动与已知验证 |
|---|---|---|
| 11-06 | `calibration17` | 增加 `bootstrap17` 固定 17 组路径与粗步长限制，修复校准重置和拟合流程。重复采样相邻差 2.5 → 5 mm，拟合 RMS 5 → 10 mm，最大残差 15 → 30 mm。`release/` 有镜像、校验和与上机说明。 |
| 11-06 | `calibration17-trfix` | 17 点候选版增加右上编码器方向修正。`release/` 有镜像、校验和与上机说明。 |
| 11-06 | `bootstrap17-repeat15` | 将同点采样相邻差进一步放宽到 15 mm，保留拟合 10/30 mm。成功轮单独将张紧判定从 970 改为 850 ADC，实机完成 17 点，复算 RMS 6.135 mm、最大残差 20.379 mm。固件、真实配置和采样日志见 `D:/MaslowDev/已校准成功/20261109-17点校准成功基线`。 |
| 11-06 | `axisbounds` | 修复五轴机器数组越界及未初始化的额外坐标，记录 ESP32 复位原因。`release/` 有镜像、ELF 和上机说明。 |
| 11-06 | `motion-channel-fix` | 合入五轴修复，修复消息排空与输入通道生命周期问题。12 项主机回归通过，USB 写入并校验；实机文件运动尚未通过该版本的验收。 |
| 11-09 | `stop-abort-candidate` / `rect200-candidate` | 同一镜像，修复紧急停止后的程序中止、报警保持、正常结束误报及软重置重复初始化。停止候选记录 17 项主机回归通过，随后以 `rect200-candidate` 名称刷入并通过写入校验；200 mm 矩形与专门实机停止测试未验收。 |
| 11-09 | `encoder-no-reverse` | 试验关闭 TL、TR、BR 软件反向，保留该候选的编码器反馈保护。19 项主机测试通过并刷写，首次收带触发反馈保护，随后看门狗复位。 |
| 11-09 | `success-baseline-no-reverse` | 从冻结的成功基线源码重建无反向版本，回退并校验刷写。构建说明见 `release/` 对应清单，刷机证据见 `logs/success-baseline-rollback-flash-20261109.log`。 |
| 11-10 | 采样质量窗口 | 重复性由相邻差改为四个有效样本的最大值减最小值，超限时滚动替换最旧样本。初始档 15 mm、精度档 2.5 mm。源码快照和构建/测试记录见 `logs/calibration-quality-*`。 |
| 11-10 | 固定参考采样 | 水平前六点固定四带的保位参考，每次采样要求四带同时在 ±1 mm 内连续 750 ms，窗口波动 ≤0.5 mm，单次采样等候上限 8 秒。验证文件 `logs/calibration-fixed-reference-verification-20261110.json` 记录了构建、针对性测试和参数。 |
| 11-10 | `calibration-hold-damped` | 固定参考保位使用带速度阻尼的 PD，限制 PWM 和变化率，编码器反馈过期即停，零指令对应零驱动。已构建并做主机验证，该轮记录未刷写。 |
| 11-11 | `saved-anchor-restore` | Apply Tension 用保存的锚点和四带稳定采样恢复当前位置，减少重复拉紧并验证几何残差。已刷写，实机恢复定位成功，几何 RMS 0.749 mm、最大残差 1.062 mm。证据见 `logs/saved-anchor-recovery-verification-20261111.json`。 |
| 11-11 | `coordinated-tension-jog` | 试验协调预拉紧与点动恢复。OTA 成功，实机试验中右下触发 300 mm 收带上限。证据见 `logs/coordinated-tension-jog-v2-ota-20261111.log` 和对应实机日志。 |
| 11-11 | `load-balanced-restore` | 试验预拉紧的负载均衡策略。镜像已保留，有构建和主机测试日志 `logs/maslow-load-balanced-restore-*`；实机恢复结果未单独核实。 |
| 11-11 | `jog-fit-r1` | 使用保存锚点拟合点动/张紧后的实测位置，点动上限 500 mm/min。保留构建、测试及 USB 运动记录；实际 XY 尺寸未核实。 |
| 11-11 | `motion-trace-r1` | 在点动版上增加四带跟踪和停稳诊断，输出带长、目标、PWM、电流与超限带。保留构建及实机日志，部分运动触发停稳超时。 |
| 11-11 | `preload-guard-r1` | 增加 Apply Tension 预拉紧高输出、低电流且无带长响应时的停止判据。17:07 构建，11-12 11:29 冻结源码和镜像，见 `backups/20261112-112933-preload-guard-r1-昨日基线`。 |
| 11-12 | `112937` 运行日志精简 | 增加中文收放带、校准点和异常摘要，保留 Debug 细节。构建成功，主机测试 31/33，另两项是已有的编码器测试夹具错误。11:33 OTA 成功，四路电机和编码器启动检测通过。 |
| 11-12 | `114319` 网页控制台中文乱码修复 | 网页按连接用流式 UTF-8 解码 WebSocket 日志，验证逐字节分包、断线重连和行分隔。11:45 网页包部署并核对 SHA-256，应用镜像与 11:33 版相同。 |
| 11-12 | `122149` 恢复网页校准状态 | 恢复网页消费的 INFO 状态转换消息，包含此前中文日志和网页修复。已构建及通过两组针对性测试，独立镜像的备份记录为待刷写。 |
| 11-12 | `134019` 校准移动皮带诊断 | 校准移动时每秒最多输出一条异常摘要，报告达到 4 mm 的保位偏差及编码器过期、读取失败或跳变。包含网页状态修复。针对性测试通过，13:51 OTA 后重连、四路检测通过，配置与网页哈希核对通过。这是截至本记录已确认部署的最新应用镜像。 |

11-12 的各备份目录内有该版源码 ZIP、固件和说明 JSON。网页乱码修复还保留了新旧网页包和部署日志。11-06 至 11-11 的历史细节见 `D:/MaslowDev/docs/Maslow校准流程与17点实机复现.md`、`Maslow停止逻辑优化-20261109.md` 与 `重新校准采样偏差与优化-20261110.md`。

## 校准实机记录

| 时间 | 已确认的结果 | 依据 |
|---|---|---|
| 11-06 15:53 | 首次 17 点完成并保存锚点。当时尚无后来新增的 ±1 mm 固定参考保位检查。 | `logs/calibration17-usb-run8-20261106.log`、`calibration17-success-run8-verification-20261106.json` |
| 11-10 18:51 | 网页日志记录 17 点完成，报告 RMS 1.524 mm、最大残差 3.633 mm。采集从第 13 点开始，实际刷写的镜像身份及前六点保位条件未核实。 | `logs/calibration17-user-success-20261110-verification-20261111.json` |
| 11-11 15:28 | 前六点通过 ±1 mm 保位检查，全部 17 点完成并进入 Ready to Cut。第 7～17 点走另一条采样路径。 | `logs/calibration17-recheck-20261111.log` 的 `wp=0` 至 `wp=5`、`Find Anchors complete` |
| 11-12 10:42–10:46 | 前六点完成采样，后续记录急停。 | 工作区 `maslow-calibration17-wifi-20261112-104254.log` |
| 11-12 11:50 | 第 1 点采样超时，右上超时快照偏差 −2.920 mm。 | 工作区 `maslow-wifi-calibration-20261112-114706.log` |
| 11-12 13:51 刷机后排查 | 现场报告右下收带时 PWM −1023、电流读数 0，随后 `$BRI` 恢复响应。另一次校准在第 3 点采样超时，左上实测 444.041 mm、目标 442.957 mm，超时偏差 +1.084 mm。单条超时日志只能确定该时刻的误差。 | 现场回报及网页日志摘录 |

## 镜像 SHA-256

以下为归档文件实算值。重复镜像合并为一行，文件名和原备份目录保留。

| 文件 | SHA-256 |
|---|---|
| `release/maslow-s3-calibration17-20261106.bin` | `c18b8f182ab702b3a1f06e64d7c50df884d022e06fd572282355c2a5e461e78d` |
| `release/maslow-s3-calibration17-trfix-20261106.bin` | `7ff3685796f8533eedda54114ee02838e38a765cca7a19ff71b36f6205dfc9cd` |
| `release/maslow-s3-bootstrap17-repeat15-20261106.bin` | `f412ab4911cb445d35cbdeb840f04f8edea035620ad3dae8437d3278f970c5a6` |
| `release/maslow-s3-axisbounds-20261106.bin` | `c35ed6595956084498ad9c3e249d26d6e5d7fa938d88e2b9eb3ff6ed6674af0d` |
| `release/maslow-s3-motion-channel-fix-20261106.bin` | `302fe6e64568b173a56142e05501c14857bd52d282811449c209ece57b4ba677` |
| `release/maslow-s3-stop-abort-candidate-20261109.bin`、`maslow-s3-rect200-candidate-20261109.bin` | `0664a4d72a6aede27a591ba8467461b7cf66aa47cf40f041dd3151a69a8811a0` |
| `release/maslow-s3-encoder-no-reverse-20261109.bin` | `40da1e2007fe158da03d5c2a095d97c6b32937328a14c97ad877ada413888f99` |
| `release/maslow-s3-success-baseline-no-reverse-20261109.bin` | `1f827d357bead0a8a9dbd6c6510616402fe4d90e9b904d1ef7b412676b5e420f` |
| `release/maslow-s3-calibration-hold-damped-20261110.bin` | `0cd999d7110e8b003d9cec6576e2274ad362bb23bd2d77727b5306b06606f2a5` |
| `release/maslow-s3-saved-anchor-restore-20261111.bin` | `626637e98758a7dc796fd268f007634bb84f9fd112d2cfab455dc0e7aa3fc797` |
| `release/maslow-s3-coordinated-tension-jog-20261111.bin` | `51f10699178ae14d69a4b36b58003e6f3dca9c4c54a6db1dbfdbcc91e1a16d16` |
| `release/maslow-s3-load-balanced-restore-20261111.bin` | `f706f70b6a9f7dfa51d9f055d5886b879cf86af2639fe1cd1348e2960d48e9ef` |
| `maslow-s3-jog-fit-r1-20261111.bin` | `bf919ae84f92e819cf56962dc673de93f1116064498adaf255069e91c3c2fe26` |
| `maslow-s3-motion-trace-r1-20261111.bin` | `430aea9630429d5e9d194fe976b33f1e79a1c286fb98abf07efa3efed113137f` |
| `maslow-s3-preload-guard-r1-20261111.bin`、`20261112-112933-preload-guard-r1-昨日基线-firmware.bin` | `0a66f4296e84d80bc2af3cd3048401a88650fb637e2417d29fc0e84fbeea2851` |
| `20261112-112937-运行日志精简-firmware.bin`、`20261112-114319-网页控制台中文乱码修复-firmware.bin` | `2245f08430ce7f96325f126b7fd77fa14385bc90c3f7e34aa1596574e79a7702` |
| `20261112-122149-恢复网页校准状态-firmware.bin` | `25fdfbf1beaaece6bb840a88a126c3137caec9d97333256c872e7010a1b2432a` |
| `20261112-134019-校准移动皮带诊断-firmware.bin` | `5b444106961a8bc2ecdb83314f6b86a3bf5e3044c6ae15c2fe69660b71c125d2` |

以后每次变更记录日期、源码提交、改动目的、产物与 SHA-256、验证结果、部署对象和回退路径。已有实机结果沿用原日志，未完成的验证明确标注。
