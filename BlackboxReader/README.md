# ApexFly v2.0 黑匣子读取工具

交互照老工程 `ApexFly\BlackboxReader`：先连接串口，再读取，有进度条、重试和停止。  
协议改成 v2.0：USART3 发 `dump` / `dumpall`，不再发老的 `BB DUMP ALL`。

老 Python 窗口 `BlackboxDump` 已停用，不要再用。

## 准备

1. Keil 烧录**本仓库最新飞机固件**（带 `dump` / `dumpall` 和进度行）。
2. 飞机上电并**上锁**，电机停。
3. USB 转串口接 USART3，`115200 8N1`。**关掉 VOFA 和其他串口助手**。
4. 双击 `ApexFlyBlackboxReader.exe` 打开窗口。不要点 `run.bat`（那个只是黑窗口闪一下再启动 exe）。

## 窗口用法

1. 选 COM 口，点「连接」。口会一直开着，状态栏变绿。
2. 模式：
   - **场次 CSV**：只解析新格式场次（校验通过的 BLX2）。
   - **全部原始**：日志区每条 64 字节原样导出，含老数据。
3. 「最近」选 1000～50000 或「全部」。
4. 点「读取」，确认上锁后开始。命令逐字慢发，4 秒无回应会重试，最多 8 次。
5. **不要连着点两次读取。** dumpall 可能要好几分钟，必须等日志出现 `dumpall done` 再切「场次 CSV」去读。
6. 看进度条和「扫描 x/y」。不要解锁、不要断电。刚点完可能十几秒都没有 `dump start`，飞控还在扫 Flash，这是正常的。
7. 完成后点「打开导出目录」。

「打开已有文本」可以把以前的 `serial_raw.log` / `dumpall_raw.txt` 再解析一遍。

## 导出文件

每次读取单独建 `exports\blackbox_时间\`：

| 文件 | 何时有 | 内容 |
|---|---|---|
| `serial_raw.log` | 每次 | 串口原文，一行不丢 |
| `blackbox_data.csv` | 场次 CSV | 全部样本，带 session 列 |
| `session_XXX.csv` | 场次 CSV | 单场样本 |
| `session_XXX.vofa.csv` | 场次 CSV | VOFA FireWater |
| `session_XXX.pid.csv` | 场次 CSV | 本场 26 项 PID |
| `dumpall_raw.txt` | 全部原始 | 地址 + 十六进制 |
| `dumpall.bin` | 全部原始 | 拼回去的 Flash 字节 |
| `INDEX.txt` | 每次 | 场次摘要 |

工具只读 Flash，不擦黑匣子，不改 PID。

## 飞控命令（USART3）

飞机必须上锁。

```
dump
dump 3000
dumpall
dumpall 10000
help
```

也认 `BB DUMP ALL` 这种老写法（大小写不敏感）。  
`dump` 跳过 CRC 不对的记录；`dumpall` 不跳过。
