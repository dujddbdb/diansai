# 2026-06-19 - 修复 track.h 乱码注释

## 涉及文件
- `bsp/track.h`

## 修改前状态
`bsp/track.h` 中的中文注释显示为乱码（例如 `ѭ������ģ��ӿ�`）。原因是文件以 GBK 编码保存，但读取时被解释为 UTF-8，导致中文三字节序列被错误解码成 Cyrillic/拉丁扩展字符。

同期文件 `bsp/track_config.h` 已经是正确的 UTF-8（无 BOM）编码。

## 修改内容
将 `track.h` 从 GBK 编码转换为 UTF-8（无 BOM），与 `track_config.h` 保持一致。

使用 PowerShell 命令:
```powershell
$bytes = [System.IO.File]::ReadAllBytes('bsp/track.h')
$text = [System.Text.Encoding]::GetEncoding('GBK').GetString($bytes)
$utf8NoBom = New-Object System.Text.UTF8Encoding $false
[System.IO.File]::WriteAllText('bsp/track.h', $text, $utf8NoBom)
```

**未修改任何代码、函数声明或宏值，仅修正常量注释。**

## 测试建议
- 在 Keil 中重新编译项目，确认无编译错误
- 用 VS Code 打开 `bsp/track.h`，确认所有中文注释正常显示
