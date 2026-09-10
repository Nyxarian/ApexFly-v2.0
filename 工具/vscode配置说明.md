# VSCode IntelliSense 配置（c_cpp_properties.json）说明

## 症状
工程能正常编译、下载，但编辑器里**所有头文件标红**，`Ctrl+Click` 跳转函数/结构体/宏全部失效。

## 根因
构建系统（EIDE → Keil AC5 / armcc）用的是 `MDK-ARM/build/ApexFly v2.0/compile_commands.json`，
**编译不受影响**；但编辑器的 IntelliSense 读的是 `MDK-ARM/.vscode/c_cpp_properties.json`，
而该文件缺失（且根 `.gitignore` 第 84 行忽略了整个 `.vscode/`，所以没法从 git 恢复）。

## 恢复方法
1. 把本目录下的 `c_cpp_properties.json` 复制到 `ApexFly v2.0\MDK-ARM\.vscode\`；
2. VSCode 里 `Ctrl+Shift+P` → **Developer: Reload Window**；
3. 打开任意 `.c` 文件，确认右下角 C/C++ 配置选中 **ApexFly v2.0 (Keil AC5 / armcc)**。

## 注意
- VSCode 的工作区根目录是 **`ApexFly v2.0\MDK-ARM\`**（用 `ApexFly v2.0.code-workspace` 打开），
  所以配置里的 `${workspaceFolder}` 指向 `MDK-ARM`，包含路径写成 `../Core/Inc` 这种形式。
- 若以后在 `Modules\` 下新增子目录，需要在这里同步加一条 `includePath`。
- `RTE` 的真实目录名是 `_ApexFly_v2.0`（编译命令里写的 `RTE/_ApexFly v2.0` 带空格，是不存在的旧名）。
